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
    def __init__(self, cc, incl_deg: float | None = None):
        self.c = np.array([cc.x, cc.y, cc.z])
        self.incl = math.radians(cc.inclinacion_arbol_deg if incl_deg is None else incl_deg)
        self.fx = cc.fx_px
        self.cx, self.cy = cc.ancho_px / 2, cc.alto_px / 2
        self.ancho, self.alto = cc.ancho_px, cc.alto_px

    def a_camara(self, p_robot) -> np.ndarray:
        """(adelante, izquierda, arriba) en el sistema de la cámara inclinada."""
        d = np.asarray(p_robot, float) - self.c
        ca, sa = math.cos(self.incl), math.sin(self.incl)
        return np.array([ca * d[0] + sa * d[2], d[1], -sa * d[0] + ca * d[2]])

    def proyectar(self, p_robot) -> tuple[float, float, float] | None:
        """(u, v, profundidad) en píxeles; None si queda detrás de la cámara."""
        x, y, z = self.a_camara(p_robot)
        if x <= 0.02:
            return None
        return self.cx - self.fx * y / x, self.cy - self.fx * z / x, x

    def posiciones_soportes(self, arbol: int, tablero, fraccion_radio: float = 1.0) -> dict:
        """{"alta-1": [u, v, radio], ...} con los nombres del clasificador (de izquierda a derecha en la imagen)."""
        pos = {}
        for (a, fila, col), p in soportes_en_robot(arbol, tablero).items():
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
