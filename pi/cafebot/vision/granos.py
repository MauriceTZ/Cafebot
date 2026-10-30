"""Los granos del árbol en 3D a partir de la foto: detección, a qué soporte corresponde cada uno, pose del árbol
(cámara + LiDAR) y la posición de cada grano en el sistema del robot (lo que necesita el brazo).

Por qué así y no con una red entrenada: la escena es conocida. Los 16 soportes están en posiciones fijas del
árbol (cafebot/vision/arbol.py), las pelotas miden 42 mm y sus colores son distintos de la madera. Entonces:

  1. DETECTAR: píxeles saturados (la madera tiene saturación ~85–110, las pelotas >140), manchas redondeadas, y un
     círculo ajustado SOLO al arco visible: con la cámara más abajo que los granos, el soporte tapa la parte de
     abajo de la pelota (borde inferior recto).
  2. ASIGNAR: cada mancha al soporte cuya proyección queda más cerca (con la pose del árbol que da el LiDAR).
  3. AJUSTAR la pose del árbol (x, y, ángulo) y una corrección de la inclinación de la cámara (pegada con
     silicona) por mínimos cuadrados: error de reproyección de los granos (centro y radio, en píxeles) + la medida
     del LiDAR como "prior" con su incertidumbre. Eso es la FUSIÓN cámara + LiDAR (+ odometría, que da el
     prior cuando el LiDAR no ve el pie).
  4. PROYECCIÓN INVERSA de cada mancha: rayo del píxel ∩ plano vertical de los granos (p_rayo), y rayo + distancia
     por el tamaño aparente (p_tamano). Con el árbol ajustado, la posición final de cada grano es la de su
     soporte (combina todas las medidas); las otras dos sirven de control.
  5. COLOR: Lab en la parte de arriba de la pelota (lejos del soporte) contra las referencias tomadas en el lugar
     (clasificador de Pablo, con "no sé"). Sin referencias, por matiz HSV (provisorio, para las primeras pruebas).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares

from cafebot.vision import clasificador as cl
from cafebot.vision.arbol import MedidaPie, ModeloArbol
from cafebot.vision.camara_modelo import CamaraPinhole, cortar_plano, distancia_por_tamano

CATEGORIA = {"ROJO": "MADURO", "NARANJO": "MADURO", "AMARILLO": "MADURO", "AZUL": "SOBREMADURO",
             "NEGRO": "SOBREMADURO", "VERDE": "VERDE", "VACIO": "VACIO", cl.NO_SE: "NO_SE"}


@dataclass
class Deteccion:
    u: float
    v: float
    r: float                       # px (círculo ajustado al arco visible)
    area: float
    relleno: float                 # área / círculo: < 1 si el soporte tapa la parte de abajo
    cortada: bool                  # tiene un borde recto (tapada por el soporte, o en el borde de la imagen)
    cobertura: float = 1.0         # fracción de la circunferencia que se ve (< 0,5: casquete, centro menos seguro)
    zona: tuple | None = None      # (u, v, r) donde se mide el color: círculo inscrito en lo que se ve
    clave: tuple | None = None     # (fila, col) del soporte asignado
    clase: str = cl.NO_SE
    distancia: float = math.inf    # al color de referencia más cercano
    lab: tuple | None = None
    por_matiz: bool = False        # clasificada por matiz (sin referencias calibradas)
    p_rayo: np.ndarray | None = None
    p_tamano: np.ndarray | None = None
    error_px: float = math.nan     # distancia (px) entre la detección y su soporte con el árbol ajustado


@dataclass
class Grano:
    clave: tuple
    nombre: str
    p: np.ndarray                  # centro en el robot (m): el del soporte con el árbol ajustado, corregido con
                                   # lo que se vio de ESA pelota (si el árbol no es exactamente el del modelo)
    q_arbol: np.ndarray            # lo mismo en el marco del árbol (sirve después de mover el robot)
    categoria: str                 # MADURO / SOBREMADURO / VERDE / VACIO / NO_SE / FUERA
    clase: str
    confianza: float
    det: Deteccion | None = None
    uv: tuple | None = None        # dónde se ve (modelo ajustado)
    r_px: float | None = None
    tomado: bool = False


@dataclass
class Analisis:
    pose: tuple                    # pose del pie ajustada (x de la cara, y del centro, ángulo)
    dpitch: float                  # corrección de la inclinación de la cámara (rad) (= dcam[0])
    cam: CamaraPinhole             # cámara con la corrección
    granos: dict                   # clave -> Grano
    detecciones: list
    lidar: MedidaPie | None
    error_px: float
    n_asignadas: int
    mensajes: list = field(default_factory=list)
    dcam: tuple = (0.0, 0.0, 0.0)  # corrección de la cámara (pitch, yaw, roll; rad) que salió del ajuste

    def seguros(self, categorias=("MADURO", "SOBREMADURO"), confianza_min=0.5) -> list:
        return [g for g in self.granos.values() if g.categoria in categorias and g.confianza >= confianza_min
                and not g.tomado]


# ---------------------------------------------------------------------------- 1. detectar
def circulo_kasa(pts: np.ndarray):
    """Círculo por mínimos cuadrados algebraicos (Kasa): (cx, cy, r)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack((x, y, np.ones_like(x)))
    sol, *_ = np.linalg.lstsq(A, x * x + y * y, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    return cx, cy, math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))


def circulos_3p(a: np.ndarray, b: np.ndarray, c: np.ndarray):
    """Círculos por 3 puntos, vectorizado ((N,2) cada uno): (cx, cy, r), nan si son colineales."""
    ax, ay, bx, by, cx, cy = a[:, 0], a[:, 1], b[:, 0], b[:, 1], c[:, 0], c[:, 1]
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    a2, b2, c2 = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    with np.errstate(divide="ignore", invalid="ignore"):
        ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / d
        uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / d
    return ux, uy, np.hypot(ax - ux, ay - uy)


def circulo_geometrico(pts: np.ndarray, u: float, v: float, r: float, iteraciones: int = 10):
    """Refina (u, v, r) minimizando la distancia de cada punto a la circunferencia (Gauss-Newton)."""
    p = np.array([u, v, r], float)
    for _ in range(iteraciones):
        dx, dy = pts[:, 0] - p[0], pts[:, 1] - p[1]
        d = np.maximum(np.hypot(dx, dy), 1e-9)
        J = np.column_stack((-dx / d, -dy / d, -np.ones_like(d)))
        paso, *_ = np.linalg.lstsq(J, p[2] - d, rcond=None)
        p += paso
        if np.abs(paso).max() < 1e-3:
            break
    return p


def puntos_de_arco(contorno: np.ndarray):
    """Puntos del contorno que sirven para ajustar el círculo: los que quedan SOBRE la envolvente convexa (las
    muescas que deja un brillo o una sombra no) y no sobre un tramo recto largo de ella (el borde del soporte que
    tapa la parte de abajo, o el borde de la imagen). Devuelve (puntos, cortada, radio del círculo mínimo)."""
    pts = contorno.reshape(-1, 2).astype(float)
    hull = cv2.convexHull(contorno.reshape(-1, 1, 2).astype(np.int32)).reshape(-1, 2).astype(float)
    _, r0 = cv2.minEnclosingCircle(hull.astype(np.float32))
    if len(hull) < 3:
        return pts, False, float(r0)
    a, ab = hull, np.roll(hull, -1, axis=0) - hull
    largo = np.hypot(ab[:, 0], ab[:, 1])
    ap = pts[:, None, :] - a[None, :, :]
    t = np.clip(np.einsum("nmk,mk->nm", ap, ab) / np.maximum(largo ** 2, 1e-9), 0.0, 1.0)
    dist = np.hypot(*(ap - t[..., None] * ab[None]).transpose(2, 0, 1))          # (N puntos, M lados)
    recto = largo > max(5.0, 0.5 * r0)        # en un círculo digital los lados de la envolvente son cortos (~√(2r))
    sobre = dist.min(axis=1) < 1.5
    en_recto = (dist[:, recto] < 1.5).any(axis=1) if recto.any() else np.zeros(len(pts), bool)
    return pts[sobre & ~en_recto], bool(recto.any()), float(r0)


def circulo_visible(contorno: np.ndarray, r_min: float = 0.0, r_max: float = math.inf, semilla: int = 0):
    """(u, v, r, cortada, cobertura) de una mancha redonda cuya parte de abajo puede estar tapada por un borde
    recto (el soporte) y que puede tener muescas (el brillo, que tiene poca saturación).
    Círculo por RANSAC sobre los puntos del arco (puntos_de_arco) y refinado geométrico con los que calzan.
    cobertura = fracción de la circunferencia con puntos (1 = pelota entera; < 0,5 = casquete, centro menos seguro)."""
    arco, cortada, r0 = puntos_de_arco(contorno)
    fallback = None
    if len(arco) >= 8:
        rng = np.random.default_rng(semilla)
        idx = rng.integers(0, len(arco), size=(150, 3))
        ux, uy, rr = circulos_3p(arco[idx[:, 0]], arco[idx[:, 1]], arco[idx[:, 2]])
        valido = np.isfinite(rr) & (rr > max(0.8 * r0, r_min)) & (rr < min(2.0 * r0, r_max))
        if valido.any():
            ux, uy, rr = ux[valido], uy[valido], rr[valido]
            tol = max(1.2, 0.04 * r0)
            dist = np.abs(np.hypot(arco[None, :, 0] - ux[:, None], arco[None, :, 1] - uy[:, None]) - rr[:, None])
            k = int(np.argmax((dist < tol).sum(axis=1)))
            sel = dist[k] < tol
            if sel.sum() >= 8:
                u, v, r = circulo_geometrico(arco[sel], ux[k], uy[k], rr[k])
                sel = np.abs(np.hypot(arco[:, 0] - u, arco[:, 1] - v) - r) < 1.5 * tol
                if sel.sum() >= 8:
                    u, v, r = circulo_geometrico(arco[sel], u, v, r)
                    ang = np.arctan2(arco[sel, 1] - v, arco[sel, 0] - u)
                    cobertura = len(np.unique(np.floor((ang + math.pi) / (2 * math.pi) * 36))) / 36
                    if np.isfinite(r) and r > 0:
                        return float(u), float(v), float(r), cortada, float(cobertura)
    (u, v), r = cv2.minEnclosingCircle(contorno.astype(np.float32))
    return float(u), float(v), float(r), cortada, 0.5 if cortada else 1.0


def detectar(img_bgr, r_min: float, r_max: float, sat_min: int = 130, val_min: int = 60) -> list:
    """Pelotas de color en la imagen: manchas saturadas y redondeadas de radio entre r_min y r_max px."""
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 1] > sat_min) & (hsv[..., 2] > val_min)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    k = max(3, int(r_min / 3) | 1)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    out = []
    for c in contornos:
        if len(c) < 8 or cv2.contourArea(c) < math.pi * (0.4 * r_min) ** 2:
            continue
        hull = cv2.convexHull(c)
        area = cv2.contourArea(hull)                       # con las muescas del brillo rellenas
        u, v, r, cortada, cobertura = circulo_visible(c, 0.8 * r_min, 1.25 * r_max)
        if not r_min <= r <= r_max:
            continue
        relleno = area / (math.pi * r * r)
        if relleno > 1.25 or relleno < (0.10 if cortada else 0.45):       # tapada por el soporte: solo un casquete
            continue
        out.append(Deteccion(u, v, r, float(area), float(relleno), cortada, cobertura, zona=zona_inscrita(hull)))
    return sorted(out, key=lambda d: d.u)


def zona_inscrita(poligono: np.ndarray, fraccion: float = 0.8):
    """(u, v, r) del círculo más grande dentro de un polígono (px), achicado a `fraccion`: donde medir el color de lo
    que se ve de la pelota (lejos de sus bordes, del soporte y del fondo)."""
    pts = poligono.reshape(-1, 2).astype(np.int32)
    x0, y0 = pts.min(axis=0) - 2
    w, h = pts.max(axis=0) - (x0, y0) + 3
    m = np.zeros((int(h), int(w)), np.uint8)
    cv2.fillPoly(m, [pts - (x0, y0)], 255)
    dt = cv2.distanceTransform(m, cv2.DIST_L2, 5)
    j, i = np.unravel_index(int(np.argmax(dt)), dt.shape)
    return float(x0 + i), float(y0 + j), float(max(1.0, fraccion * dt[j, i]))


# ---------------------------------------------------------------------------- 2-3. asignar y ajustar
def proyectar_soportes(cam: CamaraPinhole, modelo: ModeloArbol, pose) -> dict:
    """{clave: (u, v, r_px, p_robot)} de los soportes que caen delante de la cámara."""
    pts = modelo.en_robot(pose)
    claves = list(pts)
    P = np.array([pts[c] for c in claves])
    uv, z = cam.proyectar(P)
    r = cam.radio_px(P, modelo.radio_grano_m)
    return {c: (uv[i, 0], uv[i, 1], r[i], P[i]) for i, c in enumerate(claves) if z[i] > 0.05 and np.all(np.isfinite(uv[i]))}


def asignar(dets: list, proy: dict, gate: float = 0.9) -> dict:
    """{clave: índice de la detección}: la más cercana a cada soporte, a menos de gate·radio (sin repetir)."""
    pares = []
    for i, d in enumerate(dets):
        for c, (u, v, r, _) in proy.items():
            dist = math.hypot(d.u - u, d.v - v)
            if dist < gate * max(r, 4.0) and 0.55 * r < d.r < 1.8 * r:
                pares.append((dist / max(r, 4.0), i, c))
    pares.sort()
    out, usados = {}, set()
    for _, i, c in pares:
        if c not in out and i not in usados:
            out[c] = i
            usados.add(i)
    return out


def incertidumbre_relativa(d: Deteccion) -> float:
    """Factor (≥ 1) de la incertidumbre del centro de una detección según cuánto arco se ve."""
    return 1.0 if d.cobertura >= 0.6 else 1.5 if d.cobertura >= 0.42 else 3.0


SIG_CAM_DEF = (math.radians(4.0), math.radians(3.0), math.radians(3.0))   # pitch, yaw, roll sin calibrar


def camara_corregida(cam: CamaraPinhole, dcam) -> CamaraPinhole:
    return cam.con(pitch=cam.pitch + dcam[0], yaw=cam.yaw + dcam[1], roll=cam.roll + dcam[2])


def _residuos(theta, dets, asig, cam, modelo, prior, sig):
    x, y, ang = theta[:3]
    proy = proyectar_soportes(camara_corregida(cam, theta[3:]), modelo, (x, y, ang))
    res = []
    for clave, i in asig.items():
        d = dets[i]
        if clave not in proy:
            res += [10.0, 10.0, 10.0]
            continue
        u, v, r, _ = proy[clave]
        s = 1.5 + 0.03 * r
        k = incertidumbre_relativa(d)                       # casquete chico: centro y radio menos seguros
        res += [(d.u - u) / (k * s), (d.v - v) / (k * s), (d.r - r) / (k * (1.5 + 0.08 * r))]
    res += [(x - prior[0]) / sig[0], (y - prior[1]) / sig[1], (ang - prior[2]) / sig[2]]
    res += [theta[3] / sig[3], theta[4] / sig[4], theta[5] / sig[5]]
    return np.array(res)


def completar_sigmas(sig, sig_cam=None):
    """(σx, σy, σang[, σpitch]) + σ de la cámara -> 6 valores (x, y, ang, pitch, yaw, roll)."""
    sc = list(sig_cam or SIG_CAM_DEF)
    if len(sig) >= 4 and sig_cam is None:
        sc[0] = sig[3]
    return tuple(float(v) for v in (*sig[:3], *sc))


def ajustar_pose(dets: list, modelo: ModeloArbol, cam: CamaraPinhole, prior, sig, buscar=True, dp_max_deg=8.0,
                 sig_cam=None):
    """Pose del pie (x, y, ang) + corrección de la cámara (pitch, yaw, roll) que mejor explican las detecciones,
    con `prior` (x, y, ang) ± sig (σx, σy, σang) y la cámara ± sig_cam (σpitch, σyaw, σroll; chicos si la cámara
    está calibrada: entonces manda la imagen; grandes si no: manda el LiDAR en lo que la imagen no distingue, p. ej.
    un corrimiento lateral del árbol vs un yaw de la cámara). Devuelve (theta de 6, asignación)."""
    sig = completar_sigmas(sig, sig_cam)
    theta = np.array([prior[0], prior[1], prior[2], 0.0, 0.0, 0.0], float)
    if buscar and dets:
        # Búsqueda gruesa (corrimiento lateral y de inclinación): por si el prior está a varios cm
        mejor = None
        for dy in np.arange(-3, 3.01, 1.0) * min(sig[1], 0.04):
            for dp in np.radians(np.arange(-dp_max_deg, dp_max_deg + 0.01, 1.0)):
                t = theta + np.array([0.0, dy, 0.0, dp, 0.0, 0.0])
                proy = proyectar_soportes(camara_corregida(cam, t[3:]), modelo, t[:3])
                a = asignar(dets, proy, gate=0.6)
                err = sum(math.hypot(dets[i].u - proy[c][0], dets[i].v - proy[c][1]) / proy[c][2] for c, i in a.items())
                puntaje = len(a) - 0.3 * err - 0.5 * (dy / sig[1]) ** 2 - 0.5 * (dp / sig[3]) ** 2
                if mejor is None or puntaje > mejor[0]:
                    mejor = (puntaje, t)
        theta = mejor[1]
    asig = {}
    for _ in range(4):
        asig = asignar(dets, proyectar_soportes(camara_corregida(cam, theta[3:]), modelo, theta[:3]))
        if not asig:
            break
        sol = least_squares(_residuos, theta, args=(dets, asig, cam, modelo, prior, sig), loss="soft_l1", f_scale=2.0)
        theta = sol.x
    return theta, asig


def posicion_grano(p_modelo, d: Deteccion | None, r_px: float, max_m: float = 0.03) -> np.ndarray:
    """Centro de un grano: el de su soporte (árbol ajustado) movido hacia donde se vio la pelota (rayo ∩ plano de los
    granos), con peso según cuánto arco se ve (casquete chico: poco). Así un árbol de prueba que no calza exacto con
    su modelo igual da la posición de cada pelota."""
    p_modelo = np.asarray(p_modelo, float)
    if d is None or d.p_rayo is None or not np.isfinite(d.error_px) or d.error_px > 0.6 * r_px:
        return p_modelo
    delta = np.asarray(d.p_rayo) - p_modelo
    if np.linalg.norm(delta) > max_m:
        return p_modelo
    return p_modelo + delta / incertidumbre_relativa(d) ** 2


# ---------------------------------------------------------------------------- 5. color
def clase_por_matiz(hsv_mediana) -> str:
    """Sin referencias calibradas: por el matiz (OpenCV, 0..180). Provisorio y conservador."""
    h, s, v = hsv_mediana
    if s < 110 or v < 50:
        return cl.NO_SE
    if h < 7 or h > 168:
        return "ROJO"
    if 9 <= h <= 18:
        return "NARANJO"
    if 21 <= h <= 32:
        return "AMARILLO"
    if 42 <= h <= 85:
        return "VERDE"
    if 95 <= h <= 130:
        return "AZUL"
    return cl.NO_SE


def zona_modelo(cam: CamaraPinhole, modelo: ModeloArbol, pose, p, uv, r):
    """(u, v, r) donde medir el color de un soporte SIN mancha detectada (negro, vacío o "no sé"): el círculo inscrito
    en la parte de la pelota que, según el modelo, no tapa su soporte."""
    lado = int(2 * r + 6)
    x0, y0 = int(uv[0] - lado / 2), int(uv[1] - lado / 2)
    m = np.zeros((lado, lado), np.uint8)
    cv2.circle(m, (int(round(uv[0] - x0)), int(round(uv[1] - y0))), max(1, int(round(r))), 255, -1)
    q, z = cam.proyectar(modelo.esquinas_soporte(p, pose[2]))
    if np.all(z > 0.02) and np.all(np.isfinite(q)):
        cv2.fillConvexPoly(m, cv2.convexHull(np.round(q - (x0, y0)).astype(np.int32)), 0)
    dt = cv2.distanceTransform(m, cv2.DIST_L2, 5)
    j, i = np.unravel_index(int(np.argmax(dt)), dt.shape)
    if dt[j, i] < 1.5:
        return uv[0], uv[1] - 0.5 * r, 0.3 * r
    return float(x0 + i), float(y0 + j), float(0.8 * dt[j, i])


def color_en(imgs: list, clasif: cl.Clasificador | None, u, v, r):
    """(clase, distancia, lab, por_matiz) con varias fotos: si no dan la misma clase en todas, NO_SE."""
    clases, dist, lab0, matiz = [], 0.0, None, False
    for img in imgs:
        h, w = img.shape[:2]
        uu, vv, rr = int(round(u)), int(round(v)), max(2, int(round(r)))
        if not (rr <= uu < w - rr and rr <= vv < h - rr):
            return cl.NO_SE, math.inf, None, False
        if clasif is not None and clasif.muestras:
            lab = cl.imagen_a_lab(img[vv - rr:vv + rr + 1, uu - rr:uu + rr + 1])
            config = dict(clasif.config, fraccion_radio=1.0)
            q = cl.color_zona(lab, rr, rr, rr, config)
            res = clasif.clasificar_lab(q)
            clases.append(res.clase)
            dist = max(dist, res.distancia)
            lab0 = lab0 or q
        else:
            hsv = cv2.cvtColor(img[vv - rr:vv + rr + 1, uu - rr:uu + rr + 1], cv2.COLOR_BGR2HSV).reshape(-1, 3)
            hsv = hsv[(hsv[:, 1] > 60)] if (hsv[:, 1] > 60).sum() > 0.3 * len(hsv) else hsv
            med = np.median(hsv.astype(float), axis=0)
            if 0 < med[0] < 10 and np.mean(hsv[:, 0] > 160) > 0.3:      # el rojo da la vuelta (0 = 180)
                med[0] = 179
            clases.append(clase_por_matiz(med))
            matiz = True
    clase = clases[0] if len(set(clases)) == 1 else cl.NO_SE
    return clase, dist, lab0, matiz


# ---------------------------------------------------------------------------- todo junto
def buscar_inclinacion(dets: list, modelo: ModeloArbol, cam: CamaraPinhole, prior, sig, rango_deg: float = 30.0):
    """Búsqueda gruesa de la inclinación de la cámara SIN su prior (y, si el árbol no lo midió el LiDAR, de su
    distancia): para una cámara montada a otro ángulo del que dice la config. -> (dpitch rad, dx, dy, n asignadas)."""
    if len(dets) < 3:
        return 0.0, 0.0, 0.0, 0
    sin_lidar = sig[0] >= 0.015                    # con el LiDAR, la posición del árbol no se toca
    dxs = np.arange(-4, 4.01) * 0.05 if sin_lidar else np.array([0.0])          # sin LiDAR: ±20 cm
    dys = np.arange(-3, 3.01) * min(max(sig[1], 0.01), 0.04) if sin_lidar else np.array([0.0])
    mejor = None
    for dp in np.radians(np.arange(-rango_deg, rango_deg + 0.01, 2.0)):
        c = cam.con(pitch=cam.pitch + dp)
        for dx in dxs:
            for dy in dys:
                proy = proyectar_soportes(c, modelo, (prior[0] + dx, prior[1] + dy, prior[2]))
                a = asignar(dets, proy, gate=0.6)
                if not a:
                    continue
                err = sum(math.hypot(dets[i].u - proy[k][0], dets[i].v - proy[k][1]) / proy[k][2] for k, i in a.items())
                puntaje = len(a) - 0.3 * err - 0.02 * abs(math.degrees(dp)) - 2.0 * (abs(dx) + abs(dy))
                if mejor is None or puntaje > mejor[0]:
                    mejor = (puntaje, float(dp), float(dx), float(dy), len(a))
    return (0.0, 0.0, 0.0, 0) if mejor is None else mejor[1:]


def analizar(imgs: list, cam: CamaraPinhole, modelo: ModeloArbol, prior, sig, clasif: cl.Clasificador | None = None,
             lidar: MedidaPie | None = None, sat_min=130, val_min=60, buscar=True, confianza_matiz=0.6,
             sig_cam=None) -> Analisis:
    """Fotos (BGR, del tamaño de `cam`) + pose esperada del pie (prior ± sig) -> Analisis.
    sig_cam = (σpitch, σyaw, σroll) de la cámara (rad): SIG_CAM_DEF sin calibrar, ~0,5° calibrada.
    `lidar`, si hay, es la medida que dio el prior (solo se guarda para mostrarla)."""
    img = imgs[0]
    if (img.shape[1], img.shape[0]) != (cam.ancho, cam.alto):
        cam = cam.escalada(img.shape[1], img.shape[0])
    proy0 = proyectar_soportes(cam, modelo, prior)
    radios = [p[2] for p in proy0.values()] or [cam.f * modelo.radio_grano_m / 0.6]
    dets = detectar(img, 0.5 * min(radios), 1.6 * max(radios), sat_min, val_min)
    mensajes = []
    dp0 = 0.0
    if buscar and sig_cam is None:
        # Cámara sin calibrar: ¿está inclinada distinto de lo que dice la config? (búsqueda amplia, ±30°)
        dp, dx, dy, n = buscar_inclinacion(dets, modelo, cam, prior, sig)
        n0 = len(asignar(dets, proyectar_soportes(cam, modelo, prior), gate=0.6))
        if n >= 3 and n > n0 and (sig[0] >= 0.015 or abs(dp) > math.radians(5)):
            dp0 = dp
            cam = cam.con(pitch=cam.pitch + dp)
            prior = (prior[0] + dx, prior[1] + dy, prior[2])
            if abs(dp) > math.radians(5):
                mensajes.append(f"la cámara está inclinada ~{math.degrees(cam.pitch):.0f}° y la config dice "
                                f"{math.degrees(cam.pitch - dp):.0f}°: se usó ~{math.degrees(cam.pitch):.0f}° (para no "
                                f"depender de esto: --inclinacion {math.degrees(cam.pitch):.0f} o calibrar con C)")
            if abs(dx) >= 0.05:
                mensajes.append(f"el árbol está ~{100 * dx:+.0f} cm más {'lejos' if dx > 0 else 'cerca'} de lo esperado "
                                "(¿--frente bien medido?)")
    theta, asig = ajustar_pose(dets, modelo, cam, prior, sig, buscar=buscar, sig_cam=sig_cam)
    pose, dcam = tuple(float(x) for x in theta[:3]), tuple(float(x) for x in theta[3:])
    cam_aj = camara_corregida(cam, dcam)
    dcam = (dcam[0] + dp0, dcam[1], dcam[2])          # corrección total respecto de la cámara de la config
    proy = proyectar_soportes(cam_aj, modelo, pose)
    sigs = completar_sigmas(sig, sig_cam)
    lejos = [n for n, a, b, s_ in zip(("x", "y", "ángulo"), pose, prior, sigs) if abs(a - b) > 3 * s_]
    if lejos and asig:
        mensajes.append("la foto y el LiDAR no coinciden en " + ", ".join(lejos) + " del árbol (¿cámara sin calibrar, "
                        "otro objeto delante, o el árbol no es el del modelo?)")
    # Con pocas pelotas, el árbol corrido UNA columna puede calzar igual: entonces decide el prior (LiDAR)
    pasos = [np.min(np.abs(np.diff(sorted(v)))) for v in modelo.columnas_y.values() if len(v) > 1]
    if asig and pasos and sigs[1] > 0.02:
        paso = float(min(pasos))
        _, t = ModeloArbol.ejes(pose[2])
        for s_ in (1, -1):
            alt = (pose[0] + s_ * paso * t[0], pose[1] + s_ * paso * t[1], pose[2])
            if len(asignar(dets, proyectar_soportes(cam_aj, modelo, alt), gate=0.6)) >= len(asig):
                mensajes.append(f"OJO: la foto calza igual con el árbol corrido una columna ({100 * paso:.0f} cm); la "
                                f"columna la decide el LiDAR (σ {100 * sigs[1]:.0f} cm): los NOMBRES de los soportes "
                                "pueden estar corridos (las posiciones de las pelotas vistas no)")
                break

    # Proyección inversa de cada detección (rayo ∩ plano de los granos, y por el tamaño)
    if dets:
        uv = np.array([[d.u, d.v] for d in dets])
        rayos = cam_aj.rayos(uv)
        punto, normal = modelo.plano_granos(pose)
        p_rayo = cortar_plano(cam_aj.pos, rayos, punto, normal)
        dist = distancia_por_tamano(np.array([d.r for d in dets]), cam_aj.f, modelo.radio_grano_m)
        for i, d in enumerate(dets):
            d.p_rayo = p_rayo[i] if np.all(np.isfinite(p_rayo[i])) else None
            d.p_tamano = cam_aj.pos + rayos[i] * dist[i]
    errores = []
    for clave, i in asig.items():
        d = dets[i]
        d.clave = clave
        u, v, r, _ = proy[clave]
        d.error_px = math.hypot(d.u - u, d.v - v)
        errores.append(d.error_px)

    # Color de cada soporte que se ve
    granos = {}
    for clave, (u, v, r, p) in proy.items():
        nombre = ModeloArbol.nombre_de(clave)
        i = asig.get(clave)
        p = posicion_grano(p, dets[i] if i is not None else None, r)
        q = modelo.a_arbol(p, pose)
        if not cam_aj.dentro(np.array([[u, v]]), margen=r)[0]:
            granos[clave] = Grano(clave, nombre, p, q, "FUERA", "FUERA", 0.0, None, (u, v), r)
            continue
        d = dets[i] if i is not None else None
        if d is None and not (clasif is not None and clasif.muestras):
            # Sin mancha de color y sin referencias para ver si es negro o vacío: se deja
            granos[clave] = Grano(clave, nombre, p, q, "VACIO", "VACIO", 0.5, None, (u, v), r)
            continue
        cu, cv_, cr = d.zona if d is not None and d.zona else zona_modelo(cam_aj, modelo, pose, p, (u, v), r)
        clase, distancia, lab, matiz = color_en(imgs, clasif, cu, cv_, cr)
        if d is None and clase not in ("NEGRO", "VACIO", cl.NO_SE):
            # Sin mancha de color solo puede ser una pelota NEGRA (poco saturada) o el soporte vacío: un grano de
            # color saturado el detector lo habría visto. Si el color dice otra cosa (la madera se parece al
            # amarillo, 30 oct), es el soporte: VACIO. Así el brazo nunca va a un soporte sin pelota.
            granos[clave] = Grano(clave, nombre, p, q, "VACIO", "VACIO", 0.5, None, (u, v), r)
            continue
        if d is not None:
            d.clase, d.distancia, d.lab, d.por_matiz = clase, distancia, lab, matiz
        if matiz:
            conf = confianza_matiz if clase != cl.NO_SE else 0.0
        else:
            umbral = clasif.config["umbral_distancia"]
            conf = 0.0 if clase == cl.NO_SE else max(0.0, 1.0 - 0.3 * distancia / umbral)
        granos[clave] = Grano(clave, nombre, p, q, CATEGORIA.get(clase, "NO_SE"), clase, conf, d, (u, v), r)
    if any(d.por_matiz for d in dets):
        mensajes.append("colores por MATIZ (provisorio): enseñar referencias con clic + 1..7")
    sin = [d for d in dets if d.clave is None]
    if sin:
        mensajes.append(f"{len(sin)} mancha(s) de color que no calzan con ningún soporte (otra cosa, o el árbol "
                        "no es el del modelo)")
    return Analisis(pose, dcam[0], cam_aj, granos, dets, lidar, float(np.mean(errores)) if errores else math.nan,
                    len(asig), mensajes, dcam)


# ---------------------------------------------------------------------------- calibrar la cámara
def calibrar_extrinsecos(P: np.ndarray, obs: np.ndarray, cam: CamaraPinhole, k=None, mover_pos: bool = False,
                         sig_pos: float = 0.02, sig_ang_deg: float = 20.0):
    """Pose de la cámara en el robot (inclinación, yaw, roll; y si `mover_pos`, también la posición del lente,
    ± sig_pos) que mejor proyecta los puntos 3D `P` (robot, m) en los píxeles `obs`. Mínimo 3 puntos.
    k = factor de incertidumbre de cada observación (1 = normal). Devuelve (cámara calibrada, error RMS en px)."""
    P, obs = np.asarray(P, float), np.asarray(obs, float)
    if len(P) < 3:
        raise ValueError("hacen falta al menos 3 puntos")
    k = np.ones(len(P)) if k is None else np.asarray(k, float)

    def armar(t):
        c = cam.con(pitch=t[0], yaw=t[1], roll=t[2])
        if mover_pos:
            c = c.con(pos=cam.pos + np.asarray(t[3:6]))
        return c

    def res(t):
        uv, _ = armar(t).proyectar(P)
        r = list(((uv - obs) / k[:, None]).ravel() / 2.0)
        s = math.radians(sig_ang_deg)
        r += [(t[1] - cam.yaw) / s, (t[2] - cam.roll) / s]
        if mover_pos:
            r += list(np.asarray(t[3:6]) / sig_pos)
        return np.array(r)

    t0 = [cam.pitch, cam.yaw, cam.roll] + ([0.0, 0.0, 0.0] if mover_pos else [])
    sol = least_squares(res, t0, loss="soft_l1", f_scale=3.0)
    c = armar(sol.x)
    uv, _ = c.proyectar(P)
    return c, float(np.sqrt(np.mean(np.sum((uv - obs) ** 2, axis=1))))


def calibrar_camara(dets: list, asig: dict, modelo: ModeloArbol, pose, cam: CamaraPinhole, mover_pos: bool = False,
                    sig_pos: float = 0.02):
    """Con el ÁRBOL: la cámara que mejor proyecta los granos detectados sobre su soporte, con el árbol en `pose`
    (medida con el LiDAR, o a mano con huincha). Mínimo 3 granos. Devuelve (cámara, error RMS en px).
    Ojo: un error en la pose del árbol pasa a la cámara (1 cm de lado a 60 cm ≈ 1° de yaw): conviene promediar
    varias posiciones o usar el brazo (calibrar_extrinsecos con puntos de la cinemática directa)."""
    if len(asig) < 3:
        raise ValueError("hacen falta al menos 3 granos detectados y asignados a su soporte")
    pts = modelo.en_robot(pose)
    claves = list(asig)
    P = np.array([pts[c] for c in claves])
    obs = np.array([[dets[asig[c]].u, dets[asig[c]].v] for c in claves])
    k = np.array([incertidumbre_relativa(dets[asig[c]]) for c in claves])
    return calibrar_extrinsecos(P, obs, cam, k, mover_pos, sig_pos)
