"""Visión para la misión SIN LiDAR (cafebot/mision/mision_ir.py, docs/mision_ir.md).

albercas(pose): con la cámara mirando el piso (X 2 = mision_ir.inclinacion_albercas_deg), qué tramos en y de la
    franja de albercas (x 1,55–1,75) son AZULES. Cada píxel azul se proyecta al piso (rayo ∩ z = 0) con el modelo
    de la cámara; se cuentan por franjas de 2 cm en y. Devuelve (azules [(y0, y1)], visible (y_min, y_max)) en el
    marco de la misión (el de `pose`).
arbol(prior): fotos del árbol + granos.analizar: la pose del pie que mejor calza con las pelotas que se ven (sobre
    todo su posición LATERAL) y el color de cada soporte. Devuelve (tablero (x, y, ang) en el robot | None,
    {(fila, col): (categoría, confianza)}).
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

from cafebot.mundo import cancha
from cafebot.vision.camara_modelo import CamaraPinhole, cortar_plano

# Azul de las albercas (HSV de OpenCV: H 0..180). 5 nov: el mismo azul que la canasta azul   [AJUSTAR en la cancha]
H_AZUL = (95, 135)
S_MIN, V_MIN = 90, 40
BIN_M = 0.02
MIN_PUNTOS_BIN = 6


ALTO_ALBERCA = 0.09     # reglamento: contenedores azules de 720 x 200 x 90 mm


def puntos_piso(cam: CamaraPinhole, uv: np.ndarray, z: float = 0.0) -> np.ndarray:
    """Píxeles -> puntos del plano horizontal a altura z, en el sistema del robot (N, 3); nan los que no lo cortan."""
    if len(uv) == 0:
        return np.zeros((0, 3))
    return cortar_plano(cam.pos, cam.rayos(uv), np.array([0.0, 0.0, z]), np.array([0.0, 0.0, 1.0]))


def a_local(p_robot: np.ndarray, pose) -> np.ndarray:
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return np.column_stack((x + c * p_robot[:, 0] - s * p_robot[:, 1], y + s * p_robot[:, 0] + c * p_robot[:, 1]))


def intervalos(ys: np.ndarray, lo: float, hi: float, bin_m=BIN_M, min_n=MIN_PUNTOS_BIN, unir_m=0.04):
    """Tramos [y0, y1] con al menos min_n puntos por franja de bin_m (huecos de menos de unir_m se rellenan)."""
    if len(ys) == 0 or hi <= lo:
        return []
    bordes = np.arange(lo, hi + bin_m, bin_m)
    n, _ = np.histogram(ys, bordes)
    ocupado = n >= min_n
    out = []
    i = 0
    while i < len(ocupado):
        if ocupado[i]:
            j = i
            while j + 1 < len(ocupado) and ocupado[j + 1]:
                j += 1
            out.append([bordes[i], bordes[j + 1]])
            i = j + 1
        else:
            i += 1
    unidos = []
    for a, b in out:
        if unidos and a - unidos[-1][1] < unir_m:
            unidos[-1][1] = b
        else:
            unidos.append([a, b])
    return [(float(a), float(b)) for a, b in unidos]


def albercas_en_imagen(img_bgr, cam: CamaraPinhole, pose, x0=cancha.ALBERCA_X0, x1=cancha.ALBERCA_X1, paso_px=4,
                       z=ALTO_ALBERCA):
    """(azules, visible) en el marco de `pose`: tramos en y de la franja x0..x1 que la cámara ve azules, y desde dónde
    hasta dónde (en y) se ve esa franja (para no suponer libre lo que no se ve). Los píxeles se proyectan al plano
    del BORDE de arriba de las albercas (z = 9 cm): ese borde da el ancho exacto; las paredes, vistas de lado, caen
    hacia adentro (proyectadas al piso, en cambio, la alberca se vería ~12 cm más ancha a 40 cm de lado)."""
    import cv2
    if (img_bgr.shape[1], img_bgr.shape[0]) != (cam.ancho, cam.alto):
        cam = cam.escalada(img_bgr.shape[1], img_bgr.shape[0])
    hsv = cv2.cvtColor(img_bgr[::paso_px, ::paso_px], cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    azul = (h >= H_AZUL[0]) & (h <= H_AZUL[1]) & (s >= S_MIN) & (v >= V_MIN)
    filas, cols = np.nonzero(azul)
    uv = np.column_stack((cols * paso_px + paso_px / 2, filas * paso_px + paso_px / 2)).astype(float)
    p = a_local(puntos_piso(cam, uv, z), pose) if len(uv) else np.zeros((0, 2))
    en_franja = (p[:, 0] >= x0 - 0.02) & (p[:, 0] <= x1 + 0.02) if len(p) else np.zeros(0, bool)
    # lo visible de la franja: todos los píxeles de la imagen proyectados, los que caen en ella
    us, vs = np.meshgrid(np.arange(0, cam.ancho, 16), np.arange(0, cam.alto, 16))
    q = a_local(puntos_piso(cam, np.column_stack((us.ravel(), vs.ravel())).astype(float), z), pose)
    ok = np.isfinite(q[:, 0]) & (q[:, 0] >= x0) & (q[:, 0] <= x1)
    if not ok.any():
        return [], (0.0, 0.0)
    visible = (float(q[ok, 1].min()) + 0.03, float(q[ok, 1].max()) - 0.03)
    ys = p[en_franja, 1] if len(p) else np.zeros(0)
    # la densidad de puntos por franja depende de la distancia: se exige un mínimo relativo a lo que se vería
    return intervalos(ys, visible[0] - 0.05, visible[1] + 0.05), visible


def buscar_lateral(dets: list, modelo, cam: CamaraPinhole, prior, rango_m: float = 0.18, paso_m: float = 0.01,
                   pitch_deg: float = 4.0, paso_pitch_deg: float = 1.0, rango_x_m: float = 0.075, paso_x_m: float = 0.015):
    """Sin LiDAR la y del árbol se sabe a ±10–15 cm: búsqueda FINA de corrimiento lateral (y de inclinación de la
    cámara) que más pelotas detectadas calce con los soportes del modelo. Puntaje = calzadas - 0,5 x detecciones que
    no calzan con nada: un árbol corrido una columna (12 cm) calza casi igual, pero deja una columna de pelotas
    sin soporte. También la distancia (la odometría la sabe a ~3 %: unos cm). Devuelve (dx, dy, dpitch (rad),
    calzadas, puntaje, mejor puntaje con otro dy)."""
    from cafebot.vision import granos
    mejores = []
    for dp in np.radians(np.arange(-pitch_deg, pitch_deg + 1e-9, paso_pitch_deg)):
        c = cam.con(pitch=cam.pitch + dp)
        for dx in np.arange(-rango_x_m, rango_x_m + 1e-9, paso_x_m):
            for dy in np.arange(-rango_m, rango_m + 1e-9, paso_m):
                proy = granos.proyectar_soportes(c, modelo, (prior[0] + dx, prior[1] + dy, prior[2]))
                a = granos.asignar(dets, proy, gate=0.6)
                err = sum(math.hypot(dets[i].u - proy[k][0], dets[i].v - proy[k][1]) / proy[k][2] for k, i in a.items())
                puntaje = len(a) - 0.5 * (len(dets) - len(a)) - 0.3 * err
                mejores.append((puntaje, float(dx), float(dy), float(dp), len(a)))
    mejores.sort(reverse=True)
    p0, dx0, dy0, dp0, n0 = mejores[0]
    otro = next((m[0] for m in mejores if abs(m[2] - dy0) > 0.04), -math.inf)
    return dx0, dy0, dp0, n0, p0, otro


class VisionIRCamara:
    """Con la cámara real (misma interfaz que VisionIRSimulada)."""

    def __init__(self, cfg, camara=None, clasificador=None, log=print):
        from cafebot.vision import clasificador as cl
        self.cfg, self.cc, self.log = cfg, cfg.camara, log
        self._cam = camara
        self.clasif = clasificador if clasificador is not None else cl.Clasificador.cargar(cl.ARCHIVO_REFERENCIAS)
        self.carpeta = cl.CARPETA_CAPTURAS

    @property
    def cam(self):
        if self._cam is None:
            from cafebot.vision.camara import abrir_para_mision
            self._cam = abrir_para_mision(self.log)
        return self._cam

    def iniciar(self):
        _ = self.cam

    def preparar_arbol(self):
        try:
            self.cam.enfocar(self.cfg.geometria.largo / 2 + self.cfg.mision.distancia_foto_m - self.cc.x)
        except Exception:
            pass

    def _guardar(self, img, nombre):
        if not self.cc.guardar_fotos:
            return
        import cv2
        os.makedirs(self.carpeta, exist_ok=True)
        cv2.imwrite(os.path.join(self.carpeta, time.strftime(f"%Y%m%d_%H%M%S_{nombre}.jpg")), img)

    def albercas(self, pose):
        try:
            img = self.cam.capturar()
        except Exception as e:
            self.log(f"[visión] sin foto de las albercas: {e}")
            return None
        cam = CamaraPinhole.desde_config(self.cc, self.cfg.mision_ir.inclinacion_albercas_deg)
        azules, visible = albercas_en_imagen(img, cam, pose)
        self._guardar(img, "albercas")
        return azules, visible

    def arbol(self, prior):
        from cafebot.vision import granos
        from cafebot.vision.arbol import ModeloArbol
        try:
            imgs = [self.cam.capturar() for _ in range(max(1, self.cc.fotos))]
        except Exception as e:
            self.log(f"[visión] sin foto del árbol: {e}")
            return None, {}
        modelo = ModeloArbol.cargar(self.cc.arbol)
        cam = CamaraPinhole.desde_config(self.cc, self.cc.inclinacion_arbol_deg)
        if (imgs[0].shape[1], imgs[0].shape[0]) != (cam.ancho, cam.alto):
            cam = cam.escalada(imgs[0].shape[1], imgs[0].shape[0])
        r = [p[2] for p in granos.proyectar_soportes(cam, modelo, prior).values()] or [40.0]
        dets = granos.detectar(imgs[0], 0.5 * min(r), 1.6 * max(r), self.cc.sat_min, self.cc.val_min)
        t = time.monotonic()
        dx, dy, dp, n, p0, otro = buscar_lateral(dets, modelo, cam, prior)
        self.log(f"[visión] búsqueda: {len(dets)} pelotas, {n} calzan corriendo el árbol {100 * dx:+.1f} / "
                 f"{100 * dy:+.1f} cm y la cámara {math.degrees(dp):+.0f}° ({time.monotonic() - t:.1f} s)")
        if n < 3 or p0 - otro < 1.0:
            self.log(f"[visión] árbol: {len(dets)} pelotas detectadas, {n} calzan; "
                     + ("no se distingue la columna (otro corrimiento calza casi igual)" if n >= 3 else "muy pocas")
                     + ": no le creo a esta foto")
            self._guardar(imgs[0], "arbol_dudoso")
            return None, {}
        prior = (prior[0] + dx, prior[1] + dy, prior[2])
        cam = cam.con(pitch=cam.pitch + dp)
        sig = (0.02, 0.015, math.radians(3))      # ya centrado por la búsqueda fina
        sig_cam = tuple(math.radians(v) for v in (1.5, 2.0, 2.0))
        a = granos.analizar(imgs, cam, modelo, prior, sig, clasif=self.clasif if self.clasif.muestras else None,
                            sat_min=self.cc.sat_min, val_min=self.cc.val_min, buscar=False, sig_cam=sig_cam)
        for m in a.mensajes:
            self.log(f"[visión] {m}")
        self._guardar(imgs[0], "arbol")
        lect = {g.clave: (g.categoria, float(g.confianza)) for g in a.granos.values()}
        tablero = a.pose if a.n_asignadas >= 3 else None
        self.log(f"[visión] {a.n_asignadas} pelotas calzan con el árbol (error {a.error_px:.1f} px)")
        return tablero, lect
