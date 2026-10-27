"""SLAM 2D: ICP "recortado" barrido-contra-mapa + grilla de ocupación log-odds + mapa estático.

Portado de legacy/integracion_juan_v1 (LidarSLAM2D) con estos cambios:
  - Convención del resto del proyecto (x adelante, θ antihorario). Antes 0° = +Y.
  - Recibe puntos YA en el sistema del robot (el driver LD06 ya aplicó el montaje frontal
    y borró la huella del propio robot).
  - El ICP parte de la predicción de la ODOMETRÍA (EKF con el Mega), no de extrapolar el
    paso anterior: mucho más robusto en giros.
  - Sin dependencia de scikit-learn.
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.spatial import cKDTree

from cafebot.navegacion.geometria import envolver, rectangulo, transformar


class SlamLidar:
    def __init__(self, lado_m: float = 10.0, res_m: float = 0.04, huella=None, origen_sensor=(0.0, 0.0)):
        self.res = res_m
        self.dim = int(round(lado_m / res_m))
        self.centro = self.dim // 2                # el robot arranca en el centro del mapa
        self.log_odds = np.zeros((self.dim, self.dim), np.float32)
        self.impactos = np.zeros((self.dim, self.dim), np.uint16)
        self.suma_x = np.zeros((self.dim, self.dim), np.float64)   # posición media sub-celda
        self.suma_y = np.zeros((self.dim, self.dim), np.float64)
        self.cuenta = np.zeros((self.dim, self.dim), np.float32)

        self.pose = np.zeros(3)
        self.trayectoria = deque(maxlen=2000)
        self.inicializado = False
        self.congelado = False          # tecla M: solo localizar, no mapear
        self.pausar_integracion = False  # p. ej. mientras el brazo trabaja frente al LiDAR
        self.huella = huella            # ((xmin,xmax),(ymin,ymax)) del robot: celdas libres
        self._huella_pts = None
        self.origen_sensor = np.asarray(origen_sensor, float)   # LiDAR en el sistema del robot

        self.umbral_estatico = 1.2
        self.min_impactos = 3
        self.n_barridos = 0
        self.calidad = 1.0
        self.calidad_min = 0.30
        self.calentamiento = 10
        self._cache_ref = None

    # ------------------------------------------------------------------ grilla
    def a_celda(self, pts: np.ndarray):
        gx = np.floor(pts[:, 0] / self.res + self.centro).astype(np.int32)
        gy = np.floor(pts[:, 1] / self.res + self.centro).astype(np.int32)
        ok = (gx >= 0) & (gx < self.dim) & (gy >= 0) & (gy < self.dim)
        return gx, gy, ok

    def mascara_estatica(self) -> np.ndarray:
        return (self.log_odds > self.umbral_estatico) & (self.impactos >= self.min_impactos)

    def puntos_ocupados(self) -> np.ndarray:
        """Referencia del ICP: posición MEDIA de los impactos de cada celda ocupada.
        Usar el centro de la celda sesga el ICP hasta media celda (2 cm) en la dirección peor vista."""
        gy, gx = np.where(self.log_odds > 0.8)
        c = self.cuenta[gy, gx]
        hay = c > 0
        seg = np.maximum(c, 1)
        return np.column_stack((np.where(hay, self.suma_x[gy, gx] / seg, (gx - self.centro + 0.5) * self.res),
                                np.where(hay, self.suma_y[gy, gx] / seg, (gy - self.centro + 0.5) * self.res)))

    def puntos_estaticos(self) -> np.ndarray:
        """Posición media de los impactos de cada celda del mapa estático (precisión sub-celda)."""
        gy, gx = np.nonzero(self.mascara_estatica())
        c = self.cuenta[gy, gx]
        hay = c > 0
        seg = np.maximum(c, 1)
        px = np.where(hay, self.suma_x[gy, gx] / seg, (gx - self.centro + 0.5) * self.res)
        py = np.where(hay, self.suma_y[gy, gx] / seg, (gy - self.centro + 0.5) * self.res)
        return np.column_stack((px, py))

    def cerca_de_estatico(self, p, mascara, r_celdas: int = 2) -> bool:
        gx = int(np.floor(p[0] / self.res + self.centro))
        gy = int(np.floor(p[1] / self.res + self.centro))
        if not (0 <= gx < self.dim and 0 <= gy < self.dim):
            return False
        return bool(mascara[max(gy - r_celdas, 0):gy + r_celdas + 1, max(gx - r_celdas, 0):gx + r_celdas + 1].any())

    def alternar_congelado(self):
        self.congelado = not self.congelado
        self._cache_ref = None

    def guardar(self, ruta="mapa_estatico.npz"):
        np.savez_compressed(ruta, log_odds=self.log_odds, impactos=self.impactos, suma_x=self.suma_x,
                            suma_y=self.suma_y, cuenta=self.cuenta, res=self.res, dim=self.dim)

    def cargar(self, ruta="mapa_estatico.npz") -> bool:
        """El robot debe arrancar en el mismo lugar y orientación en que se guardó."""
        try:
            d = np.load(ruta)
            if int(d["dim"]) != self.dim or abs(float(d["res"]) - self.res) > 1e-9:
                return False
            self.log_odds, self.impactos = d["log_odds"].astype(np.float32), d["impactos"].astype(np.uint16)
            self.suma_x, self.suma_y, self.cuenta = d["suma_x"], d["suma_y"], d["cuenta"].astype(np.float32)
        except (OSError, KeyError, ValueError):
            return False
        self.pose = np.zeros(3)
        self.trayectoria.clear()
        self.inicializado = True
        self.congelado = True
        self._cache_ref = None
        return True

    # ------------------------------------------------------------------ ICP
    def _icp(self, pts, ref, pose0, arbol, iteraciones=18):
        """(pose, calidad). Calidad = fracción de puntos a < 10 cm del mapa."""
        if len(ref) < 30 or len(pts) < 30:
            return pose0.copy(), 1.0, np.zeros((3, 3))
        pose = pose0.copy()
        for _ in range(iteraciones):
            pw = transformar(pts, pose)
            d, idx = arbol.query(pw, k=1)
            validos = d < 0.35
            if validos.sum() < 20:
                break
            corte = np.percentile(d[validos], 80)
            inl = validos & (d <= corte)
            if inl.sum() < 15:
                break
            src, dst = pw[inl], ref[idx[inl]]
            ms, md = src.mean(0), dst.mean(0)
            U, _, Vt = np.linalg.svd((src - ms).T @ (dst - md))
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1] *= -1
                R = Vt.T @ U.T
            dth = np.arctan2(R[1, 0], R[0, 0])
            dt = md - R @ ms
            pose[:2] = R @ pose[:2] + dt
            pose[2] = envolver(pose[2] + dth)
            if np.linalg.norm(dt) < 1e-4 and abs(dth) < 1e-4:
                break
        pw = transformar(pts, pose)
        d, idx = arbol.query(pw, k=1)
        return pose, float(np.mean(d < 0.10)), self._informacion(pw, ref, idx, d < 0.10, pose, arbol)

    def _informacion(self, pw, ref, idx, inl, pose, arbol, sigma=0.02):
        """Matriz de información 3x3 (x, y, θ) del alineamiento, con las NORMALES del mapa.
        Si todo lo visible son caras paralelas al eje y, la información en y es casi nula:
        el filtro (EKF) entonces se queda con la odometría en esa dirección."""
        if inl.sum() < 10:
            return np.zeros((3, 3))
        q = ref[idx[inl]]
        _, vec = arbol.query(q, k=min(8, len(ref)))
        vec = ref[vec] - q[:, None, :]                      # vecinos del mapa alrededor de cada punto
        cxx = (vec[:, :, 0] ** 2).mean(1)
        cyy = (vec[:, :, 1] ** 2).mean(1)
        cxy = (vec[:, :, 0] * vec[:, :, 1]).mean(1)
        a = 0.5 * np.arctan2(2 * cxy, cxx - cyy)            # dirección principal de la superficie
        n = np.column_stack((-np.sin(a), np.cos(a)))        # normal
        p = pw[inl] - pose[:2]
        J = np.column_stack((n[:, 0], n[:, 1], n[:, 0] * -p[:, 1] + n[:, 1] * p[:, 0]))
        return (J.T @ J) / sigma ** 2

    # ------------------------------------------------------------------ mapa
    def _integrar(self, pw: np.ndarray):
        if self.congelado or self.pausar_integracion or len(pw) == 0:
            return
        # Espacio libre: muestras cada ~0,7 celda desde el LiDAR hasta una celda antes del impacto
        # (el código antiguo empezaba al 15 % del rayo y dejaba un hueco "desconocido" delante del robot)
        o = transformar(self.origen_sensor[None], self.pose)[0]
        v = pw - o
        d = np.hypot(v[:, 0], v[:, 1])
        s = np.arange(0.0, float(d.max()) if len(d) else 0.0, 0.7 * self.res)
        m = s[None, :] < (d[:, None] - self.res)
        f = np.where(m, s[None, :] / np.maximum(d[:, None], 1e-6), np.nan)
        lx, ly = o[0] + f * v[:, 0:1], o[1] + f * v[:, 1:2]
        libres = np.column_stack((lx[m], ly[m]))
        gx, gy, ok = self.a_celda(libres)
        celdas = np.unique(np.column_stack((gy[ok], gx[ok])), axis=0)   # una vez por celda y barrido
        self.log_odds[celdas[:, 0], celdas[:, 1]] -= 0.12
        if self.huella is not None:                         # bajo el robot no hay obstáculos
            if self._huella_pts is None:
                (x0, x1), (y0, y1) = self.huella
                xs, ys = np.meshgrid(np.arange(x0, x1, self.res / 2), np.arange(y0, y1, self.res / 2))
                self._huella_pts = np.column_stack((xs.ravel(), ys.ravel()))
            gx, gy, ok = self.a_celda(transformar(self._huella_pts, self.pose))
            celdas = np.unique(np.column_stack((gy[ok], gx[ok])), axis=0)
            self.log_odds[celdas[:, 0], celdas[:, 1]] -= 0.12
        gx, gy, ok = self.a_celda(pw)
        self.log_odds[gy[ok], gx[ok]] += 0.65
        self.impactos[gy[ok], gx[ok]] = np.minimum(self.impactos[gy[ok], gx[ok]], 60000) + 1
        idx = (gy[ok], gx[ok])
        np.add.at(self.suma_x, idx, pw[ok, 0])
        np.add.at(self.suma_y, idx, pw[ok, 1])
        np.add.at(self.cuenta, idx, 1.0)
        np.clip(self.log_odds, -4.0, 4.5, out=self.log_odds)

    @staticmethod
    def _mascara_dinamica(pts, zonas):
        m = np.zeros(len(pts), bool)
        for c, r in zonas:
            m |= np.linalg.norm(pts - c, axis=1) < r
        return m

    def localizar(self, pts_robot: np.ndarray, pose_prediccion=None, zonas_dinamicas=(), quieto=False):
        """Alinea el barrido con el mapa. Devuelve (pose_icp, calidad, información 3x3).
        quieto: el Mega confirma que el robot no se mueve -> no se recalcula la pose.
            En esta cancha casi todo lo que se ve son caras paralelas (frente de árboles y
            albercas): el ICP determina mal el desplazamiento lateral; ver _informacion()."""
        pred = np.array(pose_prediccion if pose_prediccion is not None else self.pose, float)
        if not self.inicializado:
            return pred, 1.0, np.zeros((3, 3))
        if self.congelado and self._cache_ref is not None:
            ref, arbol = self._cache_ref
        else:
            ref = self.puntos_ocupados()
            arbol = cKDTree(ref) if len(ref) >= 30 else None
            if self.congelado and arbol is not None:
                self._cache_ref = (ref, arbol)
        pts_icp = pts_robot
        if zonas_dinamicas:
            pts_icp = pts_robot[~self._mascara_dinamica(transformar(pts_robot, pred), zonas_dinamicas)]
        if arbol is None:
            return pred, 1.0, np.zeros((3, 3))
        if quieto:
            _, q, info = self._icp(pts_icp, ref, pred, arbol, iteraciones=0)
            return pred, q, np.zeros((3, 3))
        nueva, q, info = self._icp(pts_icp, ref, pred, arbol)
        # Un ICP que se va muy lejos de la odometría casi siempre es un mal alineamiento
        if np.linalg.norm(nueva[:2] - pred[:2]) > 0.30 or abs(envolver(nueva[2] - pred[2])) > 0.35:
            return pred, 0.0, np.zeros((3, 3))
        return nueva, q, info

    def integrar(self, pts_robot: np.ndarray, pose, calidad: float, zonas_dinamicas=()):
        """Fija la pose (idealmente la FUSIONADA) y agrega el barrido al mapa.
        Devuelve (puntos en el mapa, candidatos a objetos no estáticos)."""
        self.pose = np.asarray(pose, float).copy()
        self.calidad = calidad
        pw = transformar(pts_robot, self.pose)
        if not self.inicializado:
            self._integrar(pw)
            gx, gy, ok = self.a_celda(pw)
            self.log_odds[gy[ok], gx[ok]] = 2.0
            self.impactos[gy[ok], gx[ok]] = self.min_impactos
            self.inicializado = True
            self.n_barridos = 1
            self.trayectoria.append(self.pose[:2].copy())
            return pw, np.empty((0, 2))
        if not self.trayectoria or np.linalg.norm(self.pose[:2] - self.trayectoria[-1]) > 0.02:
            self.trayectoria.append(self.pose[:2].copy())
        gx, gy, ok = self.a_celda(pw)
        candidato = np.ones(len(pw), bool)
        candidato[ok] = self.log_odds[gy[ok], gx[ok]] < 2.2
        if calidad >= self.calidad_min or self.n_barridos < self.calentamiento:
            estaticos = pw[~self._mascara_dinamica(pw, zonas_dinamicas)] if zonas_dinamicas else pw
            if not self.congelado:
                self._integrar(estaticos)
                self.n_barridos += 1
        return pw, pw[candidato]

    def paso(self, pts_robot, pose_prediccion=None, zonas_dinamicas=(), quieto=False):
        """Localizar + integrar con la pose del ICP (uso sin odometría)."""
        pose, q, _ = self.localizar(pts_robot, pose_prediccion, zonas_dinamicas, quieto)
        return self.integrar(pts_robot, pose, q, zonas_dinamicas)

    def esquinas_huella(self, pose=None) -> np.ndarray | None:
        if self.huella is None:
            return None
        (x0, x1), (y0, y1) = self.huella
        return transformar(rectangulo((x0 + x1) / 2, (y0 + y1) / 2, (x1 - x0) / 2, (y1 - y0) / 2),
                           self.pose if pose is None else pose)
