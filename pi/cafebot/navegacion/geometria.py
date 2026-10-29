"""Utilidades geométricas comunes. Convención: x adelante, y izquierda, θ antihorario."""
from __future__ import annotations

import math

import numpy as np


def envolver(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def transformar(pts: np.ndarray, pose) -> np.ndarray:
    """Puntos (N,2) del sistema del robot al del mapa, con pose = (x, y, θ)."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return pts @ np.array([[c, s], [-s, c]]) + np.array([x, y])


def componer(a, b) -> np.ndarray:
    """a ⊕ b: aplicar el desplazamiento b (en el sistema de a) a la pose a."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], envolver(a[2] + b[2])])


def relativo(a, b) -> np.ndarray:
    """Desplazamiento de a hasta b expresado en el sistema de a (inverso de componer)."""
    c, s = math.cos(a[2]), math.sin(a[2])
    dx, dy = b[0] - a[0], b[1] - a[1]
    return np.array([c * dx + s * dy, -s * dx + c * dy, envolver(b[2] - a[2])])


def rectangulo(cx, cy, hx, hy) -> np.ndarray:
    return np.array([[cx - hx, cy - hy], [cx + hx, cy - hy], [cx + hx, cy + hy], [cx - hx, cy + hy]])


def invertir(pose) -> np.ndarray:
    """Pose inversa: si `pose` es el marco B visto desde A, devuelve A visto desde B."""
    return relativo(pose, np.zeros(3))


def rotar(v, th: float) -> np.ndarray:
    c, s = math.cos(th), math.sin(th)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
