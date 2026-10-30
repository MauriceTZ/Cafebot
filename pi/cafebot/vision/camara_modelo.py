"""Modelo de cámara estenopeica COMPLETO (Pi Camera Module 3): proyección 3D -> píxel y su inversa, píxel -> rayo.

Es el pipeline de un motor de render al revés: el render lleva un punto del mundo al píxel (vista ->
proyección -> distorsión del lente -> píxeles); aquí también se hace el camino de vuelta (píxel -> sin
distorsión -> dirección en la cámara -> rayo en el robot). Un rayo no dice a qué distancia está el objeto: la
profundidad sale de lo que se sabe del mundo (el plano donde están los granos, medido con el LiDAR; el tamaño
de la pelota, 42 mm; o la forma del árbol completo). Eso lo hace cafebot/vision/granos.py.

Sistemas:
  robot   x adelante, y izquierda, z arriba (origen: centro de la base, en el SUELO)
  cámara  (OpenCV) x a la derecha, y hacia abajo, z hacia adelante por el eje óptico
Pose de la cámara en el robot: posición del lente + inclinación (pitch, + = mirando hacia ARRIBA), yaw (+ = hacia
la izquierda) y roll (+ = la imagen gira en sentido antihorario).

Intrínsecos (en píxeles de la imagen de trabajo, 1536 × 864): fx, fy, cx, cy y distorsión (k1, k2, p1, p2, k3,
como OpenCV). De fábrica: 66° de campo horizontal (Camera Module 3 estándar) -> fx = 768 / tan(33°) = 1183 px.
Se calibran con un tablero de ajedrez (python3 -m tools.camara intrinseca).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

# Columnas: ejes de la cámara (x derecha, y abajo, z adelante) expresados en el robot, con la cámara derecha
R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
CAMPO_H_DEG = 66.0           # Camera Module 3 estándar (la Wide: 102°)
CAMPO_V_DEG = 41.0


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class CamaraPinhole:
    ancho: int = 1536
    alto: int = 864
    fx: float = 1183.0
    fy: float = 1183.0
    cx: float = 768.0
    cy: float = 432.0
    dist: np.ndarray = field(default_factory=lambda: np.zeros(5))
    pos: np.ndarray = field(default_factory=lambda: np.array([0.15, 0.0, 0.25]))   # lente, en el robot (m)
    pitch: float = 0.0           # rad, + = mirando hacia arriba
    yaw: float = 0.0             # rad, + = mirando hacia la izquierda
    roll: float = 0.0            # rad

    # ------------------------------------------------------------------ construcción
    @staticmethod
    def desde_config(cc, inclinacion_deg: float | None = None) -> "CamaraPinhole":
        """Desde ConfigCamara (pi/cafebot/config.py + robot_local.json)."""
        fx = float(cc.fx_px)
        return CamaraPinhole(
            ancho=int(cc.ancho_px), alto=int(cc.alto_px), fx=fx, fy=float(cc.fy_px or fx),
            cx=float(cc.cx_px or cc.ancho_px / 2), cy=float(cc.cy_px or cc.alto_px / 2),
            dist=np.array(list(cc.dist) + [0.0] * (5 - len(cc.dist)), float)[:5],
            pos=np.array([cc.x, cc.y, cc.z], float),
            pitch=math.radians(cc.inclinacion_arbol_deg if inclinacion_deg is None else inclinacion_deg),
            yaw=math.radians(cc.yaw_deg), roll=math.radians(cc.roll_deg))

    def con(self, **cambios) -> "CamaraPinhole":
        """Copia con algunos valores cambiados (p. ej. con(pitch=...))."""
        return replace(self, **cambios)

    def escalada(self, ancho: int, alto: int) -> "CamaraPinhole":
        """La misma cámara para una imagen de otro tamaño (p. ej. la foto a resolución completa)."""
        kx, ky = ancho / self.ancho, alto / self.alto
        return replace(self, ancho=ancho, alto=alto, fx=self.fx * kx, fy=self.fy * ky, cx=self.cx * kx, cy=self.cy * ky)

    # ------------------------------------------------------------------ geometría
    @property
    def R(self) -> np.ndarray:
        """Rotación cámara -> robot (columnas: ejes de la cámara en el robot)."""
        return rot_z(self.yaw) @ rot_y(-self.pitch) @ rot_x(self.roll) @ R0

    @property
    def f(self) -> float:
        return 0.5 * (self.fx + self.fy)

    def a_camara(self, P) -> np.ndarray:
        """Puntos (N,3) del robot -> sistema de la cámara (N,3)."""
        P = np.atleast_2d(np.asarray(P, float))
        return (P - self.pos) @ self.R

    def distorsionar(self, xn: np.ndarray, yn: np.ndarray):
        k1, k2, p1, p2, k3 = self.dist
        r2 = xn * xn + yn * yn
        radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
        xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn)
        yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn
        return xd, yd

    def proyectar(self, P):
        """Puntos (N,3) del robot -> (uv (N,2) en píxeles, profundidad z de la cámara (N,)).
        Los puntos detrás de la cámara quedan con uv = nan."""
        Pc = self.a_camara(P)
        z = Pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            xn, yn = Pc[:, 0] / z, Pc[:, 1] / z
        xd, yd = self.distorsionar(xn, yn)
        uv = np.column_stack((self.fx * xd + self.cx, self.fy * yd + self.cy))
        uv[z <= 0.01] = np.nan
        return uv, z

    def sin_distorsion(self, uv) -> tuple[np.ndarray, np.ndarray]:
        """Píxeles (N,2) -> coordenadas normalizadas (xn, yn) sin la distorsión del lente (iterativo)."""
        uv = np.atleast_2d(np.asarray(uv, float))
        xd, yd = (uv[:, 0] - self.cx) / self.fx, (uv[:, 1] - self.cy) / self.fy
        if not np.any(self.dist):
            return xd, yd
        xn, yn = xd.copy(), yd.copy()
        for _ in range(12):                      # punto fijo: x = xd - (distorsión(x) - x)
            xe, ye = self.distorsionar(xn, yn)
            xn, yn = xn - (xe - xd), yn - (ye - yd)
        return xn, yn

    def rayos(self, uv) -> np.ndarray:
        """Píxeles (N,2) -> direcciones unitarias (N,3) en el robot. El rayo sale de `pos`."""
        xn, yn = self.sin_distorsion(uv)
        d = np.column_stack((xn, yn, np.ones_like(xn))) @ self.R.T
        return d / np.linalg.norm(d, axis=1, keepdims=True)

    def radio_px(self, P, radio_m: float) -> np.ndarray:
        """Radio aparente (px) de una esfera de radio `radio_m` centrada en P (N,3)."""
        d = np.linalg.norm(self.a_camara(P), axis=1)
        return self.f * radio_m / np.sqrt(np.maximum(d * d - radio_m * radio_m, 1e-9))

    def dentro(self, uv, margen: float = 0.0) -> np.ndarray:
        uv = np.atleast_2d(uv)
        return ((uv[:, 0] >= margen) & (uv[:, 0] < self.ancho - margen) &
                (uv[:, 1] >= margen) & (uv[:, 1] < self.alto - margen))


def cortar_plano(origen, dirs, punto, normal) -> np.ndarray:
    """Intersección de los rayos (origen + t·dir) con el plano (punto, normal). (N,3); nan si son paralelos."""
    dirs = np.atleast_2d(dirs)
    den = dirs @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((np.asarray(punto) - np.asarray(origen)) @ normal) / den
    t = np.where((np.abs(den) < 1e-9) | (t <= 0), np.nan, t)
    return np.asarray(origen) + t[:, None] * dirs


def distancia_por_tamano(r_px: np.ndarray, f: float, radio_m: float) -> np.ndarray:
    """Distancia del lente al CENTRO de una esfera de radio_m que se ve con radio r_px."""
    s = np.asarray(r_px, float) / f
    return radio_m * np.sqrt(1.0 + 1.0 / np.maximum(s * s, 1e-12))
