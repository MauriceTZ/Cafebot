"""Simula una vuelta del LD06 en la cancha (trazado de rayos 2D contra segmentos).

Sirve para probar localización y percepción sin el robot ni la cancha.
"""
from __future__ import annotations

import math

import numpy as np

from cafebot.drivers.ld06 import ConfigLD06, Scan


def trazar(origen: tuple[float, float], angulos: np.ndarray, segmentos: np.ndarray, alcance: float = 6.0) -> np.ndarray:
    """Distancia del primer impacto de cada rayo (inf si no toca nada). Vectorizado."""
    ox, oy = origen
    dx, dy = np.cos(angulos)[:, None], np.sin(angulos)[:, None]
    x1, y1, x2, y2 = (segmentos[:, k][None, :] for k in range(4))
    ex, ey = x2 - x1, y2 - y1
    den = dx * ey - dy * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((x1 - ox) * ey - (y1 - oy) * ex) / den      # distancia sobre el rayo
        u = ((x1 - ox) * dy - (y1 - oy) * dx) / den      # posición sobre el segmento
    ok = (np.abs(den) > 1e-12) & (t > 0) & (u >= 0) & (u <= 1)
    t = np.where(ok, t, np.inf)
    d = t.min(axis=1)
    d[d > alcance] = np.inf
    return d


def simular_scan(pose: tuple[float, float, float], segmentos: np.ndarray, cfg: ConfigLD06 | None = None,
                 rng: np.random.Generator | None = None, ruido_m: float = 0.010, paso_deg: float = 0.8,
                 fuera_de_cancha: bool = True) -> Scan:
    """Vuelta en el sistema del ROBOT, igual que la entrega el driver real.

    fuera_de_cancha: agrega "público" (puntos aleatorios a 3,5-6 m) para que la
    localización aprenda a ignorar lo que no está en el mapa.
    """
    cfg = cfg or ConfigLD06()
    rng = rng or np.random.default_rng()
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    lx, ly = x + c * cfg.x_m - s * cfg.y_m, y + s * cfg.x_m + c * cfg.y_m
    a_sensor = np.radians(np.arange(-cfg.fov_deg, cfg.fov_deg, paso_deg))
    a_mundo = th + math.radians(cfg.yaw_deg) + a_sensor
    d = trazar((lx, ly), a_mundo, segmentos)
    if fuera_de_cancha:
        gente = rng.random(d.shape) < 0.08
        d = np.where(np.isinf(d) & gente, rng.uniform(3.5, 6.0, d.shape), d)
    ok = np.isfinite(d) & (d >= cfg.dist_min_m)
    d = d[ok] + rng.normal(0, ruido_m, ok.sum())
    a = a_sensor[ok] + math.radians(cfg.yaw_deg)
    # sensor -> robot
    px, py = cfg.x_m + d * np.cos(a), cfg.y_m + d * np.sin(a)
    ang = np.arctan2(py, px)
    orden = np.argsort(ang)
    return Scan(0.0, ang[orden], np.hypot(px, py)[orden], np.full(orden.size, 200))
