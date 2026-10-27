"""¿Dónde partimos? Pose inicial y escena con UNA vuelta del LiDAR (robot quieto).

El reglamento prohíbe decirle al robot en qué sector parte (un solo botón de inicio),
pero deja libre la orientación: el capitán lo deja centrado en su sector y mirando
a los árboles. Con eso:
  1. Se prueban las 5 zonas de partida con pequeños errores de x, y, θ y se elige la
     pose cuyos puntos del LiDAR caen sobre los árboles (que nunca se mueven).
  2. Se afina con mínimos cuadrados punto-a-segmento.
  3. Con la pose conocida, se mira qué posiciones de alberca y qué sectores del
     beneficiadero tienen algo (cajas). El color de las cajas lo dice la cámara.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from cafebot.drivers.ld06 import Scan
from cafebot.mundo import cancha


@dataclass
class ResultadoInicio:
    x: float
    y: float
    th: float
    zona: int                    # sector de partida 1..5
    inliers: int                 # puntos que calzan con los árboles
    error_rms: float             # m, de esos puntos
    confianza: float             # 0..1: qué tanto mejor es que la 2ª zona más probable.
                                 # Baja (< ~0,15) = las albercas tapan los árboles: confirmar
                                 # con la cámara (los árboles se ven por encima) antes de moverse.
    albercas: list[int] = field(default_factory=list)       # posiciones 1..5 con alberca
    zonas_con_caja: list[int] = field(default_factory=list)  # sectores 1..5 con algo
    puntos_alberca: dict = field(default_factory=dict)       # diagnóstico


def _dist_punto_segmentos(px: np.ndarray, py: np.ndarray, seg: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Distancia mínima de cada punto a los segmentos y el índice del más cercano."""
    x1, y1, x2, y2 = (seg[:, k][None, :] for k in range(4))
    ex, ey = x2 - x1, y2 - y1
    l2 = ex * ex + ey * ey
    t = np.clip(((px[:, None] - x1) * ex + (py[:, None] - y1) * ey) / l2, 0, 1)
    dx = px[:, None] - (x1 + t * ex)
    dy = py[:, None] - (y1 + t * ey)
    d2 = dx * dx + dy * dy
    k = np.argmin(d2, axis=1)
    return np.sqrt(d2[np.arange(len(px)), k]), k


def _a_mundo(rx, ry, pose):
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    return x + c * rx - s * ry, y + s * rx + c * ry


def _puntaje(rx, ry, seg, poses: np.ndarray, umbral: float) -> np.ndarray:
    """Para un lote de poses (N,3): cantidad (suavizada) de puntos a menos de `umbral`."""
    out = np.empty(len(poses))
    for i, p in enumerate(poses):
        wx, wy = _a_mundo(rx, ry, p)
        d, _ = _dist_punto_segmentos(wx, wy, seg)
        out[i] = np.sum(np.clip(1.0 - d / umbral, 0, None))
    return out


def _refinar(rx, ry, seg, pose, iteraciones: int = 8, umbral: float = 0.05):
    """Gauss-Newton punto-a-recta sobre los puntos cercanos a los árboles."""
    x, y, th = pose
    for _ in range(iteraciones):
        wx, wy = _a_mundo(rx, ry, (x, y, th))
        d, k = _dist_punto_segmentos(wx, wy, seg)
        m = d < umbral
        if m.sum() < 6:
            break
        s = seg[k[m]]
        ex, ey = s[:, 2] - s[:, 0], s[:, 3] - s[:, 1]
        n = np.hypot(ex, ey)
        nx, ny = -ey / n, ex / n                       # normal de cada segmento
        r = (wx[m] - s[:, 0]) * nx + (wy[m] - s[:, 1]) * ny
        # derivadas de r respecto de (x, y, th)
        c, sn = math.cos(th), math.sin(th)
        dwx_dth = -sn * rx[m] - c * ry[m]
        dwy_dth = c * rx[m] - sn * ry[m]
        J = np.stack([nx, ny, nx * dwx_dth + ny * dwy_dth], axis=1)
        delta, *_ = np.linalg.lstsq(J, -r, rcond=None)
        x, y, th = x + delta[0], y + delta[1], th + delta[2]
        if np.abs(delta).max() < 1e-5:
            break
    wx, wy = _a_mundo(rx, ry, (x, y, th))
    d, _ = _dist_punto_segmentos(wx, wy, seg)
    m = d < umbral
    rms = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else float("inf")
    return (x, y, th), int(m.sum()), rms


def estimar_inicio(scan: Scan, max_th_deg: float = 12.0, max_desv_m: float = 0.12) -> ResultadoInicio:
    rx, ry = scan.xy()
    seg = cancha.segmentos_estaticos()
    zonas_y = cancha.centros_zonas_beneficiadero()

    # 1) búsqueda gruesa: 5 zonas × θ × desvío lateral × desvío en x
    ths = np.radians(np.arange(-max_th_deg, max_th_deg + 0.1, 1.5))
    des = np.arange(-max_desv_m, max_desv_m + 1e-9, 0.04)
    mejores = []
    for z, yz in enumerate(zonas_y, start=1):
        poses = np.array([(cancha.ZONA_X_CENTRO + dx, yz + dy, th) for th in ths for dy in des for dx in des])
        p = _puntaje(rx, ry, seg, poses, umbral=0.06)
        i = int(np.argmax(p))
        mejores.append((p[i], z, tuple(poses[i])))
    mejores.sort(reverse=True)
    puntaje1, zona, pose0 = mejores[0]
    puntaje2 = mejores[1][0]

    # 2) refinamiento contra los árboles
    pose, inl, rms = _refinar(rx, ry, seg, pose0)
    confianza = float(1.0 - puntaje2 / puntaje1) if puntaje1 > 0 else 0.0

    # 3) escena con la pose conocida
    wx, wy = _a_mundo(rx, ry, pose)
    d_arbol, _ = _dist_punto_segmentos(wx, wy, seg)
    libres = d_arbol > 0.06
    # Albercas: puntos sobre la franja de albercas, en el TRAMO CENTRAL de cada posición
    # (las 5 posiciones son contiguas: los extremos de una alberca caen en el borde de la vecina)
    albercas, diag = [], {}
    en_franja = (wx > cancha.ALBERCA_X0 - 0.05) & (wx < cancha.ALBERCA_X1 + 0.05) & libres
    for pos in range(1, 6):
        cy = (pos - 0.5) * cancha.ALBERCA_LARGO_Y
        n = int(np.sum(en_franja & (np.abs(wy - cy) < 0.25)))
        diag[pos] = n
        if n >= 5:
            albercas.append(pos)
    # 4) segunda pasada: afinar también con las albercas detectadas (su cara frontal fija x y θ)
    if albercas:
        seg2 = np.vstack([seg] + [cancha.rect_alberca(p).segmentos() for p in albercas])
        pose, inl, rms = _refinar(rx, ry, seg2, pose)
        wx, wy = _a_mundo(rx, ry, pose)
        libres = _dist_punto_segmentos(wx, wy, seg2)[0] > 0.06
    # Cajas: sectores del beneficiadero (distintos al propio) con puntos dentro del cuadrado.
    # OJO: desde la partida una caja puede tapar a la otra: esta lista puede quedar incompleta.
    zonas_caja = []
    for z in range(1, 6):
        if z == zona:
            continue
        r = cancha.rect_caja(z)
        if np.sum(r.contiene(wx, wy, margen=0.04) & ~r.contiene(wx, wy, margen=-0.06) & libres) >= 5:
            zonas_caja.append(z)
    return ResultadoInicio(pose[0], pose[1], pose[2], zona, inl, rms, confianza, albercas, zonas_caja, diag)
