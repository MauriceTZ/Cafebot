"""Lo que vería la CÁMARA en el mundo simulado (la cancha del reglamento o la pista de taller), y la superposición
("realidad aumentada") de la cancha conocida sobre cualquier imagen de la cámara.

Es un dibujo 3D sencillo: polígonos planos y esferas, de lo más lejano a lo más cercano (algoritmo del pintor), con
el MISMO modelo de cámara que usa la visión (camara_modelo.CamaraPinhole: lente en el cabezal, inclinación del servo,
intrínsecos). Sin MuJoCo ni OpenGL: solo numpy y OpenCV, corre igual en la Pi y en el PC.

Marco del mundo: el de la cancha (cancha.py) o el de la pista de taller (taller.py); z hacia arriba desde el piso.
La pose del robot es (x, y, θ) en ese marco.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from cafebot.mundo import cancha
from cafebot.vision.arbol import ModeloArbol
from cafebot.vision.camara_modelo import CamaraPinhole
from cafebot.vision.sintetica import BGR, MADERA, MADERA_OSCURA, SOPORTE

# Colores (BGR)
FONDO = (88, 84, 80)               # paredes lejanas del recinto
PISO_SALA = (110, 112, 115)
PISO_CANCHA = (226, 230, 230)      # cancha clara, líneas negras                       [CONFIRMAR color real]
NEGRO = (28, 28, 28)
CAJA = {"ROJA": (50, 50, 205), "AZUL": (195, 100, 40)}
ALBERCA = (215, 175, 95)           # celeste                                           [CONFIRMAR color real]
PARED_TALLER = (190, 194, 198)
MUEBLE = (105, 125, 150)
ALTO_CAJA = 0.09                   # los costados que ve el LiDAR (cancha.py)
ALTO_ALBERCA = 0.09
# Lecturas de la misión (superposición), como en tools.cosecha
COLOR_CATEGORIA = {"MADURO": (60, 70, 255), "SOBREMADURO": (255, 130, 60), "VERDE": (70, 210, 70),
                   "VACIO": (150, 150, 150), "NO_SE": (0, 220, 255), "FUERA": (200, 90, 200)}
SIN_LEER = (235, 235, 235)
COLOR_LINEAS = (255, 230, 0)       # cian

Z_CERCA = 0.03                     # plano cercano (m): lo que está más cerca de la cámara no se dibuja
_LUZ = np.array([-0.45, 0.35, 0.82]) / np.linalg.norm([-0.45, 0.35, 0.82])


def _sombra(color, normal) -> tuple:
    """Un poco de luz según la orientación de la cara, para que las cajas se vean como cajas."""
    k = 0.62 + 0.38 * max(0.0, float(np.dot(normal, _LUZ)))
    return tuple(int(min(255, c * k)) for c in color)


# ---------------------------------------------------------------------------- la escena
@dataclass
class Pieza:
    tipo: str                       # "poli" (polígono plano), "casco" (envolvente convexa) o "esfera"
    P: np.ndarray                   # (N, 3) puntos en el mundo; la esfera: (1, 3) el centro
    color: tuple
    radio: float = 0.0              # esfera
    normal: np.ndarray | None = None   # poli: se dibuja solo si esa cara mira hacia la cámara


@dataclass
class Objeto:
    ref: np.ndarray                                     # punto para ordenar los objetos (pintor)
    fijas: list = field(default_factory=list)          # se dibujan en este orden
    grupos: list = field(default_factory=list)         # [(punto, [piezas])]: de lejos a cerca (granos y soportes)


def _rect_piso(r: cancha.Rect, color) -> Pieza:
    x0, x1 = r.cx - r.ancho_x / 2, r.cx + r.ancho_x / 2
    y0, y1 = r.cy - r.ancho_y / 2, r.cy + r.ancho_y / 2
    return Pieza("poli", np.array([[x0, y0, 0], [x1, y0, 0], [x1, y1, 0], [x0, y1, 0]], float), color)


def _prisma(r: cancha.Rect, alto: float, color, tapa=None, adentro=False) -> Objeto:
    """Caja de base r y alto `alto`. adentro=True: las caras se ven desde DENTRO (las paredes de una sala)."""
    x0, x1 = r.cx - r.ancho_x / 2, r.cx + r.ancho_x / 2
    y0, y1 = r.cy - r.ancho_y / 2, r.cy + r.ancho_y / 2
    s = -1.0 if adentro else 1.0
    lados = [((x0, y0), (x1, y0), (0, -1)), ((x1, y0), (x1, y1), (1, 0)),
             ((x1, y1), (x0, y1), (0, 1)), ((x0, y1), (x0, y0), (-1, 0))]
    o = Objeto(np.array([r.cx, r.cy, alto / 2]))
    for a, b, n in lados:
        normal = s * np.array([n[0], n[1], 0.0])
        P = np.array([[*a, 0], [*b, 0], [*b, alto], [*a, alto]], float)
        o.fijas.append(Pieza("poli", P, _sombra(color, normal), normal=normal))
    if tapa is not None:
        P = np.array([[x0, y0, alto], [x1, y0, alto], [x1, y1, alto], [x0, y1, alto]], float)
        o.fijas.append(Pieza("poli", P, _sombra(tapa, np.array([0, 0, 1.0])), normal=np.array([0, 0, 1.0])))
    return o


def objeto_arbol(modelo: ModeloArbol, pose_pie, colores: dict) -> Objeto:
    """Un árbol (tablero, pie, soportes y granos) con el pie en pose_pie = (x de la cara, y del centro, ángulo)
    del mundo. colores: {(fila, col): "ROJO" ...} de los soportes con grano (los demás, vacíos)."""
    x0, y0, ang = pose_pie
    u, t = ModeloArbol.ejes(ang)
    plano = np.array([x0, y0]) - modelo.grano_delante_m * u + modelo.tablero_detras_m * u   # cara del tablero
    z_min = min(modelo.z_cara.values()) - 0.08
    z_max = max(modelo.z_cara.values()) + 0.12
    w = modelo.tablero_ancho_m / 2
    centro = plano + modelo.copa_y_m * t
    o = Objeto(np.array([centro[0], centro[1], 0.5 * (z_min + z_max)]))
    e = [centro + s * w * t for s in (1, -1)]
    o.fijas.append(Pieza("poli", np.array([[*e[0], z_max], [*e[1], z_max], [*e[1], z_min], [*e[0], z_min]]),
                         MADERA))
    p = [np.array([x0, y0]) + s * modelo.pie_ancho_m / 2 * t for s in (1, -1)]
    o.fijas.append(Pieza("poli", np.array([[*p[0], z_min], [*p[1], z_min], [*p[1], 0.0], [*p[0], 0.0]]),
                         MADERA_OSCURA))
    for clave, P in modelo.en_robot(pose_pie).items():       # en el marco de pose_pie: aquí, el mundo
        piezas = []
        clase = colores.get(clave)
        if clase in BGR:
            piezas.append(Pieza("esfera", P[None, :], BGR[clase], radio=modelo.radio_grano_m))
        piezas.append(Pieza("casco", modelo.esquinas_soporte(P, ang), SOPORTE))   # tapa la parte de abajo
        o.grupos.append((P, piezas))
    return o


class MundoCancha:
    """La cancha de una ronda simulada. Los granos se leen de esc.granos en cada cuadro: los que el brazo
    simulado saca (BrazoSimulado los quita de ahí) desaparecen del árbol."""

    def __init__(self, esc: cancha.Escenario, modelo: ModeloArbol | None = None):
        self.esc = esc
        self.modelo = modelo or ModeloArbol()
        L = 4.0
        self.piso = [Pieza("poli", np.array([[-L, -L, 0], [cancha.LARGO_X + L, -L, 0],
                                             [cancha.LARGO_X + L, cancha.LARGO_Y + L, 0], [-L, cancha.LARGO_Y + L, 0]],
                                            float), PISO_SALA),
                     _rect_piso(cancha.Rect(cancha.LARGO_X / 2, cancha.LARGO_Y / 2, cancha.LARGO_X, cancha.LARGO_Y),
                                PISO_CANCHA)]
        self.piso += [_rect_piso(r, NEGRO) for _, r in cancha.lineas_cancha()]
        self.fijos = [_prisma(cancha.rect_alberca(a), ALTO_ALBERCA, ALBERCA, tapa=ALBERCA) for a in esc.albercas]
        for zona, color in ((esc.zona_caja_roja, "ROJA"), (esc.zona_caja_azul, "AZUL")):
            oscuro = tuple(int(0.55 * c) for c in CAJA[color])
            self.fijos.append(_prisma(cancha.rect_caja(zona), ALTO_CAJA, CAJA[color], tapa=oscuro))

    def objetos(self) -> list:
        out = list(self.fijos)
        granos = dict(self.esc.granos)                       # copia: el brazo simulado los saca desde otro hilo
        for arbol, yc in enumerate(cancha.centros_arboles(), start=1):
            colores = {(fila, col): color.upper() for (a, fila, col), (_, color) in granos.items() if a == arbol}
            out.append(objeto_arbol(self.modelo, (cancha.ARBOL_X_FRENTE, float(yc), 0.0), colores))
        return out


class MundoTaller:
    """La pista de taller de `tools.navegacion --sim --taller`: huincha negra, sala con paredes y muebles."""

    def __init__(self, largo: float, ancho: float, sala_m: float = 0.7):
        from cafebot.mundo import taller
        r = taller.sala(largo, ancho, sala_m)
        self.piso = [_rect_piso(r, PISO_CANCHA)] + [_rect_piso(h, NEGRO) for h in taller.lineas(largo, ancho)]
        self.fijos = [_prisma(r, 1.5, PARED_TALLER, adentro=True)]
        self.fijos += [_prisma(m, alto, MUEBLE, tapa=MUEBLE) for m, alto in taller.muebles(largo, ancho, sala_m)]

    def objetos(self) -> list:
        return list(self.fijos)


# ---------------------------------------------------------------------------- la cámara en el mundo
class Vista:
    """La cámara puesta en el mundo: el robot en `pose` (x, y, θ) y la cámara `cam` montada en él.
    margen: cuánto más allá del borde de la imagen se recortan los polígonos (en veces el semiancho)."""

    def __init__(self, cam: CamaraPinhole, pose, margen: float = 2.0):
        x, y, th = (float(v) for v in pose[:3])
        c, s = math.cos(th), math.sin(th)
        Rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.cam = cam
        self.R = Rz @ cam.R                                  # columnas: ejes de la cámara en el mundo
        self.pos = Rz @ np.asarray(cam.pos, float) + np.array([x, y, 0.0])
        kx = margen * max(cam.cx, cam.ancho - cam.cx) / cam.fx
        ky = margen * max(cam.cy, cam.alto - cam.cy) / cam.fy
        # semiespacios a·p + b >= 0 en la cámara (x derecha, y abajo, z adelante)
        self.planos = [(np.array([0.0, 0.0, 1.0]), -Z_CERCA),
                       (np.array([-1.0, 0.0, kx]), 0.0), (np.array([1.0, 0.0, kx]), 0.0),
                       (np.array([0.0, -1.0, ky]), 0.0), (np.array([0.0, 1.0, ky]), 0.0)]
        self.distorsion = bool(np.any(cam.dist))

    def a_cam(self, P) -> np.ndarray:
        return (np.atleast_2d(np.asarray(P, float)) - self.pos) @ self.R

    def uv(self, Pc) -> np.ndarray:
        """Puntos de la cámara (ya recortados: z > 0) -> píxeles (con la distorsión del lente si la tiene)."""
        xn, yn = Pc[:, 0] / Pc[:, 2], Pc[:, 1] / Pc[:, 2]
        if self.distorsion:
            xn, yn = self.cam.distorsionar(xn, yn)
        return np.column_stack((self.cam.fx * xn + self.cam.cx, self.cam.fy * yn + self.cam.cy))

    def recortar(self, Pc: np.ndarray) -> np.ndarray:
        """Sutherland–Hodgman: el polígono (N, 3) recortado a la pirámide de la cámara."""
        for a, b in self.planos:
            if len(Pc) == 0:
                break
            d = Pc @ a + b
            if np.all(d >= 0):
                continue
            if np.all(d < 0):
                return Pc[:0]
            out = []
            n = len(Pc)
            for i in range(n):
                j = (i + 1) % n
                if d[i] >= 0:
                    out.append(Pc[i])
                if (d[i] >= 0) != (d[j] >= 0):
                    out.append(Pc[i] + d[i] / (d[i] - d[j]) * (Pc[j] - Pc[i]))
            Pc = np.array(out)
        return Pc

    def dentro(self, Pc) -> bool:
        """¿El punto (de la cámara) está dentro de la pirámide?"""
        return all(float(np.atleast_2d(Pc)[0] @ a + b) >= 0 for a, b in self.planos)

    def recortar_segmento(self, p, q):
        """El trozo del segmento p–q (en la cámara) que cae dentro de la pirámide, o None."""
        t0, t1 = 0.0, 1.0
        for a, b in self.planos:
            dp, dq = p @ a + b, q @ a + b
            if dp < 0 and dq < 0:
                return None
            if dp < 0:
                t0 = max(t0, dp / (dp - dq))
            elif dq < 0:
                t1 = min(t1, dp / (dp - dq))
        if t0 >= t1:
            return None
        return p + t0 * (q - p), p + t1 * (q - p)


# ---------------------------------------------------------------------------- dibujo
_SH = 4                     # bits de subpíxel para OpenCV
_K = 1 << _SH


def _poli(img, vista: Vista, P, color, normal=None):
    if normal is not None and np.dot(normal, vista.pos - P.mean(axis=0)) <= 0:
        return                                               # la cara mira hacia el otro lado
    Pc = vista.recortar(vista.a_cam(P))
    if len(Pc) >= 3:
        cv2.fillPoly(img, [np.round(vista.uv(Pc) * _K).astype(np.int32)], color, cv2.LINE_AA, _SH)


def _casco(img, vista: Vista, P, color):
    Pc = vista.a_cam(P)
    if np.any(Pc[:, 2] < Z_CERCA):
        return
    uv = vista.uv(Pc)
    if np.any(np.abs(uv) > 4 * (vista.cam.ancho + vista.cam.alto)):
        return
    cv2.fillConvexPoly(img, cv2.convexHull(np.round(uv * _K).astype(np.int32)), color, cv2.LINE_AA, _SH)


def _esfera(img, vista: Vista, c, radio, color):
    Pc = vista.a_cam(c)
    d = float(np.linalg.norm(Pc[0]))
    if Pc[0, 2] < Z_CERCA + radio or d <= radio:
        return
    u, v = vista.uv(Pc)[0]
    r = vista.cam.f * radio / math.sqrt(d * d - radio * radio)
    if not (-r <= u < vista.cam.ancho + r and -r <= v < vista.cam.alto + r):
        return
    cv2.circle(img, (int(round(u * _K)), int(round(v * _K))), max(1, int(round(r * _K))), color, -1, cv2.LINE_AA, _SH)
    if r >= 3:                                                # un brillo, como en las fotos sintéticas
        brillo = tuple(int(0.4 * x + 0.6 * 255) for x in color)
        cv2.circle(img, (int(round((u - 0.35 * r) * _K)), int(round((v - 0.35 * r) * _K))),
                   max(1, int(round(0.22 * r * _K))), brillo, -1, cv2.LINE_AA, _SH)


def _pieza(img, vista: Vista, p: Pieza):
    if p.tipo == "poli":
        _poli(img, vista, p.P, p.color, p.normal)
    elif p.tipo == "casco":
        _casco(img, vista, p.P, p.color)
    else:
        _esfera(img, vista, p.P, p.radio, p.color)


def dibujar(mundo, pose_robot, cam: CamaraPinhole) -> np.ndarray:
    """Imagen BGR (cam.alto × cam.ancho) de lo que ve la cámara `cam` del robot en `pose_robot` (x, y, θ del mundo).
    Lo normal es pasar la cámara sin distorsión (cam.con(dist=np.zeros(5))): con distorsión, solo se aplica a los
    vértices (los bordes quedan rectos)."""
    vista = Vista(cam, pose_robot)
    img = np.empty((cam.alto, cam.ancho, 3), np.uint8)
    img[:] = FONDO
    for p in mundo.piso:
        _pieza(img, vista, p)
    objetos = sorted(mundo.objetos(), key=lambda o: -float(np.linalg.norm(o.ref - vista.pos)))
    for o in objetos:
        for p in o.fijas:
            _pieza(img, vista, p)
        for _, piezas in sorted(o.grupos, key=lambda g: -float(np.linalg.norm(g[0] - vista.pos))):
            for p in piezas:
                _pieza(img, vista, p)
    return img


# ---------------------------------------------------------------------------- superposición
def _linea(img, vista: Vista, p, q, color, grosor=1, pasos=24):
    s = vista.recortar_segmento(vista.a_cam(p)[0], vista.a_cam(q)[0])
    if s is None:
        return
    t = np.linspace(0.0, 1.0, pasos)[:, None]
    uv = vista.uv(s[0] + t * (s[1] - s[0]))                 # con distorsión, la recta sale curva: se muestrea
    cv2.polylines(img, [np.round(uv * _K).astype(np.int32)], False, color, grosor, cv2.LINE_AA, _SH)


def superponer_cancha(img, cam: CamaraPinhole, pose_cancha, lecturas: dict | None = None):
    """Dibuja sobre `img` (de la cámara `cam`, del tamaño de la imagen) dónde DEBERÍAN verse las líneas de la cancha
    y los 48 soportes si el robot está en pose_cancha. Cada soporte va con el color de su lectura en `lecturas`
    ({(arbol, fila, col): LecturaSoporte}, la de la misión) o blanco si no se ha leído."""
    vista = Vista(cam, pose_cancha, margen=1.08)            # con distorsión: solo dentro del campo de la imagen
    for _, r in cancha.lineas_cancha():                     # el eje de cada línea
        if r.ancho_x >= r.ancho_y:
            p, q = np.array([r.cx - r.ancho_x / 2, r.cy, 0.0]), np.array([r.cx + r.ancho_x / 2, r.cy, 0.0])
        else:
            p, q = np.array([r.cx, r.cy - r.ancho_y / 2, 0.0]), np.array([r.cx, r.cy + r.ancho_y / 2, 0.0])
        _linea(img, vista, p, q, COLOR_LINEAS)
    lecturas = lecturas or {}
    for arbol in (1, 2, 3):
        for s in cancha.soportes(arbol):
            Pc = vista.a_cam([s.x, s.y, s.z])
            if Pc[0, 2] < 0.1 or not vista.dentro(Pc):
                continue
            u, v = vista.uv(Pc)[0]
            d = float(np.linalg.norm(Pc[0]))
            r = max(2.5, cam.f * 0.021 / math.sqrt(max(d * d - 0.021 ** 2, 1e-6)))
            le = lecturas.get(s.clave)
            color = COLOR_CATEGORIA.get(le.categoria, SIN_LEER) if le is not None else SIN_LEER
            cv2.circle(img, (int(round(u * _K)), int(round(v * _K))), int(round(r * _K)), color,
                       2 if r > 6 else 1, cv2.LINE_AA, _SH)
