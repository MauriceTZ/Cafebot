"""Pista de TALLER para simular el recorrido del borde: un rectángulo de huincha negra (19 mm) sobre madera blanca,
dentro de una sala con paredes y algunos muebles que ve el LiDAR (la huincha no la ve: está en el piso).

Marco de la pista: origen en la esquina interior de la huincha (x a lo largo del lado 0, y hacia adentro).
El robot parte en la esquina (0, 0): borde de atrás x = 0, borde derecho y = 0, mirando hacia +x.
"""
from __future__ import annotations

import math

import numpy as np

from cafebot.mundo.cancha import Rect

HUINCHA = 0.019


def lineas(largo: float, ancho: float) -> list:
    """Las 4 huinchas, por FUERA del rectángulo interior [0, largo] × [0, ancho]."""
    h = HUINCHA
    return [Rect(largo / 2, -h / 2, largo + 2 * h, h), Rect(largo + h / 2, ancho / 2, h, ancho + 2 * h),
            Rect(largo / 2, ancho + h / 2, largo + 2 * h, h), Rect(-h / 2, ancho / 2, h, ancho + 2 * h)]


def sala(largo: float, ancho: float, sala_m: float = 0.7) -> Rect:
    """Las paredes de la sala, a sala_m de la huincha."""
    return Rect(largo / 2, ancho / 2, largo + 2 * sala_m, ancho + 2 * sala_m)


def muebles(largo: float, ancho: float, sala_m: float = 0.7) -> list:
    """[(Rect, alto en m)] de los muebles de la sala (el alto solo lo usa el dibujo de la cámara simulada)."""
    return [(Rect(largo + sala_m - 0.25, ancho * 0.3, 0.4, 0.6), 0.75),          # mesa contra la pared
            (Rect(largo * 0.35, -sala_m + 0.2, 0.5, 0.3), 0.30),                 # caja en el piso
            (Rect(-sala_m + 0.15, ancho * 0.75, 0.25, 0.25), 0.45),              # silla
            (Rect(largo * 0.55, ancho * 0.55, 0.30, 0.20), 0.15)]                # algo dentro de la pista (no estorba)


def segmentos(largo: float, ancho: float, sala_m: float = 0.7) -> np.ndarray:
    """Paredes de la sala (a sala_m de la huincha) y muebles para que el LiDAR tenga referencias."""
    return np.vstack([sala(largo, ancho, sala_m).segmentos()] + [r.segmentos() for r, _ in muebles(largo, ancho, sala_m)])


def inicio(sep_m: float = 0.12, torcido_deg: float = 4.0) -> np.ndarray:
    """Pose real de partida: en la esquina, a unos cm de cada borde y un poco torcido."""
    return np.array([0.1115 + sep_m + 0.05, 0.1085 + sep_m, math.radians(torcido_deg)])
