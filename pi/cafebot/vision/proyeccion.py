"""Dónde cae cada soporte del árbol en la imagen, a partir de la geometría (sin calibrar 16 posiciones a mano).

    1. El LiDAR mide la cara del tablero en el sistema del robot: (x de la cara, y del centro, ángulo).
       En el taller se puede dar a mano: el frente del robot a `frente` m del tablero, centrado.
    2. Los soportes están en posiciones conocidas respecto del tablero (Apéndice B, cafebot/mundo/cancha.py).
    3. Modelo de cámara estenopeica: lente en (x, y, z) del robot, mirando adelante e inclinada `incl` grados
       hacia arriba, con distancia focal fx (en píxeles).

Con 3 números bien ajustados (altura, inclinación y fx; `tools.camara zonas`) los 16 círculos caen sobre los
granos aunque el robot quede unos cm corrido: la corrección sale de la medida del LiDAR.
"""
from __future__ import annotations

import math

import numpy as np

from cafebot.mundo import cancha

RADIO_GRANO = 0.021


def tablero_frente(frente_m: float, largo_robot: float = 0.30, lateral_m: float = 0.0, ang_deg: float = 0.0):
    """Tablero "a mano": su cara a `frente_m` del frente del robot y su centro `lateral_m` a la izquierda."""
    return largo_robot / 2 + frente_m, lateral_m, math.radians(ang_deg)


def soportes_en_robot(arbol: int, tablero) -> dict:
    """{(arbol, fila, col): (x, y, z)} centro de cada grano en el sistema del robot."""
    x_cara, yc, ang = tablero[:3]
    u = np.array([math.cos(ang), math.sin(ang)])          # normal de la cara (alejándose del robot)
    t = np.array([-u[1], u[0]])                           # a lo largo del tablero, hacia la izquierda
    y_arbol = cancha.centros_arboles()[arbol - 1]
    out = {}
    for s in cancha.soportes(arbol):
        dy = s.y - y_arbol                                # ya incluye COPA_DESPLAZADA_M
        p = np.array([x_cara, yc]) - cancha.GRANO_DELANTE_M * u + dy * t
        out[s.clave] = (float(p[0]), float(p[1]), float(s.z))
    return out


class ModeloCamara:
    """Envoltorio de CamaraPinhole (camara_modelo.py: intrínsecos, distorsión y pose completa) con la interfaz de
    siempre: proyectar(p) -> (u, v, profundidad), posiciones_soportes(...), rect_en_imagen(...)."""

    def __init__(self, cc, incl_deg: float | None = None):
        from cafebot.vision.camara_modelo import CamaraPinhole
        self.cam = CamaraPinhole.desde_config(cc, incl_deg)
        self.c = self.cam.pos
        self.incl = self.cam.pitch
        self.fx = self.cam.fx
        self.cx, self.cy = self.cam.cx, self.cam.cy
        self.ancho, self.alto = self.cam.ancho, self.cam.alto

    def a_camara(self, p_robot) -> np.ndarray:
        """(adelante, izquierda, arriba) en el sistema de la cámara."""
        x, y, z = self.cam.a_camara(p_robot)[0]
        return np.array([z, -x, -y])

    def proyectar(self, p_robot) -> tuple[float, float, float] | None:
        """(u, v, profundidad) en píxeles; None si queda detrás de la cámara."""
        uv, z = self.cam.proyectar(np.asarray(p_robot, float)[None, :])
        if z[0] <= 0.02 or not np.all(np.isfinite(uv)):
            return None
        return float(uv[0, 0]), float(uv[0, 1]), float(z[0])

    def posiciones_soportes(self, arbol: int, tablero, fraccion_radio: float = 1.0, modelo=None) -> dict:
        """{"alta-1": [u, v, radio], ...} con los nombres del clasificador (de izquierda a derecha en la imagen).
        modelo: ModeloArbol (cafebot/vision/arbol.py) para un árbol que no es el de la competencia; entonces
        `tablero` es la pose de la cara del pie (x, y, ang)."""
        pos = {}
        if modelo is not None:
            puntos = {(arbol, f, c): p for (f, c), p in modelo.en_robot(tablero).items()}
        else:
            puntos = soportes_en_robot(arbol, tablero)
        for (a, fila, col), p in puntos.items():
            q = self.proyectar(p)
            if q is None:
                continue
            u, v, prof = q
            r = max(4, int(round(fraccion_radio * self.fx * RADIO_GRANO / prof)))
            if r <= u < self.ancho - r and r <= v < self.alto - r:
                pos[f"{fila}-{col + 1}"] = [int(round(u)), int(round(v)), r]
        return pos

    def rect_en_imagen(self, esquinas_robot) -> tuple[int, int, int, int] | None:
        """Rectángulo (u0, v0, u1, v1) que encierra puntos 3D del robot, recortado a la imagen."""
        uv = [q for q in (self.proyectar(p) for p in esquinas_robot) if q is not None]
        if not uv:
            return None
        us, vs = [q[0] for q in uv], [q[1] for q in uv]
        u0, u1 = max(0, int(min(us))), min(self.ancho - 1, int(max(us)))
        v0, v1 = max(0, int(min(vs))), min(self.alto - 1, int(max(vs)))
        return (u0, v0, u1, v1) if u1 > u0 and v1 > v0 else None


def nombre_a_clave(arbol: int, nombre: str) -> tuple:
    fila, n = nombre.split("-")
    return (arbol, fila, int(n) - 1)
