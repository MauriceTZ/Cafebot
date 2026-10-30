"""Fotos SINTÉTICAS del árbol (para pruebas sin cámara y para `tools.cosecha --sim`): el árbol del modelo dibujado
con una cámara dada, con la oclusión del soporte sobre la parte de abajo de cada pelota, un brillo en cada pelota
y ruido. También un "LiDAR" que solo ve la cara del pie del árbol (y una pared al fondo).

La cámara con que se dibuja es la REAL del mundo simulado: puede ser distinta de la configurada (así se prueba
la calibración y la corrección de inclinación).
"""
from __future__ import annotations

import time

import cv2
import numpy as np

from cafebot.vision.arbol import ModeloArbol
from cafebot.vision.camara_modelo import CamaraPinhole

BGR = {"ROJO": (40, 40, 200), "NARANJO": (30, 130, 240), "AMARILLO": (40, 220, 235), "VERDE": (60, 170, 60),
       "AZUL": (190, 90, 30), "NEGRO": (35, 35, 35)}
MADERA, MADERA_OSCURA, SOPORTE = (115, 150, 185), (95, 128, 160), (45, 60, 85)
PARED, PISO = (200, 202, 198), (150, 152, 150)


def _poligono(img, cam: CamaraPinhole, P, color):
    uv, z = cam.proyectar(np.asarray(P, float))
    if np.any(z <= 0.02) or not np.all(np.isfinite(uv)):
        return
    cv2.fillPoly(img, [np.round(uv).astype(np.int32)], color, lineType=cv2.LINE_AA)


def foto_arbol(cam: CamaraPinhole, modelo: ModeloArbol, pose_pie, colores: dict, rng=None, ruido: float = 4.0):
    """Imagen BGR (cam.alto × cam.ancho). colores: {"baja-2": "ROJO", ...} (None o ausente = soporte vacío)."""
    rng = rng or np.random.default_rng()
    img = np.empty((cam.alto, cam.ancho, 3), np.uint8)
    img[:] = PARED
    x0, y0, ang = pose_pie
    u, t = ModeloArbol.ejes(ang)
    # piso (desde 30 cm delante de la cámara hasta 6 m)
    c = cam.pos
    _poligono(img, cam, [[c[0] + 0.3, c[1] + 3, 0], [c[0] + 6, c[1] + 6, 0], [c[0] + 6, c[1] - 6, 0], [c[0] + 0.3, c[1] - 3, 0]], PISO)
    # tablero y pie
    plano = np.array([x0, y0]) - modelo.grano_delante_m * u + modelo.tablero_detras_m * u      # cara del tablero
    z_min = min(modelo.z_cara.values()) - 0.08
    z_max = max(modelo.z_cara.values()) + 0.12
    w = modelo.tablero_ancho_m / 2
    centro_copa = plano + modelo.copa_y_m * t
    esq = [centro_copa + s * w * t for s in (1, -1)]
    _poligono(img, cam, [[*esq[0], z_max], [*esq[1], z_max], [*esq[1], z_min], [*esq[0], z_min]], MADERA)
    pie = [np.array([x0, y0]) + s * modelo.pie_ancho_m / 2 * t for s in (1, -1)]
    _poligono(img, cam, [[*pie[0], z_min], [*pie[1], z_min], [*pie[1], 0.0], [*pie[0], 0.0]], MADERA_OSCURA)
    # granos y soportes, del más lejano al más cercano
    pts = modelo.en_robot(pose_pie)
    orden = sorted(pts, key=lambda k: -np.linalg.norm(cam.a_camara(pts[k])[0]))
    for clave in orden:
        p = pts[clave]
        nombre = ModeloArbol.nombre_de(clave)
        clase = colores.get(nombre)
        if clase:
            uv, z = cam.proyectar(p[None, :])
            if z[0] > 0.05:
                r = float(cam.radio_px(p[None, :], modelo.radio_grano_m)[0])
                cu, cv_ = int(round(uv[0, 0])), int(round(uv[0, 1]))
                luz = rng.uniform(0.85, 1.1)
                col = tuple(int(min(255, x * luz)) for x in BGR[clase])
                cv2.circle(img, (cu, cv_), int(round(r)), col, -1, lineType=cv2.LINE_AA)
                brillo = tuple(int(0.4 * x + 0.6 * 255) for x in col)
                cv2.circle(img, (int(cu - 0.35 * r), int(cv_ - 0.35 * r)), max(1, int(0.22 * r)), brillo, -1,
                           lineType=cv2.LINE_AA)
        # el soporte: una placa delante del tablero, con su cara de arriba en z_cara (tapa la parte de abajo)
        uv, z = cam.proyectar(modelo.esquinas_soporte(p, ang))
        if np.all(z > 0.02) and np.all(np.isfinite(uv)):
            hull = cv2.convexHull(np.round(uv).astype(np.int32))
            cv2.fillConvexPoly(img, hull, SOPORTE, lineType=cv2.LINE_AA)
    img = img.astype(np.float32) + rng.normal(0, ruido, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


class LidarPie:
    """'LiDAR' simulado: puntos de la cara del pie del árbol (y de una pared detrás), en el sistema del ROBOT, para
    la pose del pie que devuelva pose_pie_fn() (cambia si el robot se mueve)."""

    def __init__(self, modelo: ModeloArbol, pose_pie_fn, ruido_m: float = 0.004, rng=None):
        self.modelo = modelo
        self.pose_pie_fn = pose_pie_fn
        self.ruido = ruido_m
        self.rng = rng or np.random.default_rng()

    def puntos(self, paso_deg: float = 0.8, campo_deg: float = 70.0) -> np.ndarray:
        """Un barrido como el del LD06 (un punto cada ~0,8°): cada rayo da en lo más cercano (la cara del pie tapa
        la pared de atrás)."""
        x0, y0, ang = self.pose_pie_fn()
        u, t = ModeloArbol.ejes(ang)
        w = self.modelo.pie_ancho_m / 2
        base = np.array([x0, y0])
        segmentos = [(base - w * t, base + w * t), (base + 0.8 * u - 1.2 * t, base + 0.8 * u + 1.2 * t)]
        a = np.radians(np.arange(-campo_deg, campo_deg + 1e-9, paso_deg))
        d = np.column_stack((np.cos(a), np.sin(a)))
        mejor = np.full(len(a), np.inf)
        for p, q in segmentos:
            e = q - p
            den = d[:, 0] * (-e[1]) - d[:, 1] * (-e[0])
            with np.errstate(divide="ignore", invalid="ignore"):
                rr = (p[0] * (-e[1]) - p[1] * (-e[0])) / den          # distancia por el rayo
                s = (d[:, 0] * p[1] - d[:, 1] * p[0]) / den            # posición en el segmento (0..1)
            ok = np.isfinite(rr) & (rr > 0.02) & (s >= 0) & (s <= 1)
            mejor = np.where(ok & (rr < mejor), rr, mejor)
        ok = np.isfinite(mejor) & (mejor < 6.0)
        rr = mejor[ok] + self.rng.normal(0, self.ruido, ok.sum())
        return d[ok] * rr[:, None]

    def ultimo_scan(self):
        from cafebot.drivers.ld06 import Scan
        p = self.puntos()
        ang = np.arctan2(p[:, 1], p[:, 0])
        o = np.argsort(ang)
        return Scan(time.monotonic(), ang[o], np.hypot(p[:, 0], p[:, 1])[o], np.full(len(p), 200), 600.0)

    def esperar_scan(self, timeout=1.0):
        return self.ultimo_scan()

    def estadisticas(self):
        return {"ok": 1, "crc_malos": 0, "scans": 1}

    def iniciar(self):
        pass

    def detener(self):
        pass


class CamaraSintetica:
    """Misma interfaz que vision.camara.Camara: cada foto se dibuja con la escena ACTUAL (los granos tomados ya
    no están, el robot pudo moverse)."""

    def __init__(self, cam_real: CamaraPinhole, modelo: ModeloArbol, pose_pie_fn, colores: dict, semilla=None):
        self.cam_real, self.modelo, self.pose_pie_fn, self.colores = cam_real, modelo, pose_pie_fn, colores
        self.rng = np.random.default_rng(semilla)

    def enfocar(self, dist_m):
        pass

    def aplicar(self, ajustes):
        pass

    def bloquear_auto(self, espera_s=0.0):
        return {}

    def capturar(self):
        return foto_arbol(self.cam_real, self.modelo, self.pose_pie_fn(), self.colores, self.rng)

    def cerrar(self):
        pass
