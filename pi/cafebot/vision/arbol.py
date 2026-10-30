"""Forma del ÁRBOL: dónde está cada soporte respecto del PIE (lo que ve el LiDAR), y cómo medir el pie con el LiDAR.

Dos modelos:
  "competencia"   Apéndice B del reglamento (cafebot/mundo/cancha.py): soportes con la cara de arriba a 390 / 540 /
                  690 mm, 5 / 6 / 5 por fila separados 120 mm, pie de 30 cm de ancho al ras del tablero.
  "prueba" ...    cualquier otro: pi/arboles/<nombre>.json con los mismos campos (lo que no esté, de competencia).
                  El árbol de prueba del taller (29 oct) es todo de madera, con un pie angosto (una tabla) delante
                  del tablero sobre una base con una batería: hay que medir sus números [MEDIR].

Marco del árbol (el que usa el LiDAR): origen en el CENTRO DE LA CARA DEL PIE a la altura del suelo; `ang` es el
ángulo (en el robot) de la normal de esa cara, hacia adentro del árbol (alejándose del robot). Un árbol justo
delante y de frente: pose_pie = (x de la cara, y del centro, 0).

Centro de cada grano (en el robot):  [x_cara, y_pie] − grano_delante·u + (y_soporte + copa_y)·t,
                                       z = z_cara_fila + centro_sobre_cara
con u = (cos ang, sin ang) y t = (−sin ang, cos ang) (hacia la izquierda vista desde el robot).
Columnas numeradas de IZQUIERDA a derecha vistas desde el robot (col 0 = y mayor): "baja-1" es la de la izquierda.
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import asdict, dataclass, field

import numpy as np

from cafebot.mundo import cancha

RAIZ_PI = pathlib.Path(__file__).resolve().parents[2]
FILAS = ("baja", "media", "alta")


@dataclass
class ModeloArbol:
    nombre: str = "competencia"
    z_cara: dict = field(default_factory=lambda: dict(cancha.ALTURA_SOPORTE))     # cara de arriba de cada fila (m)
    columnas_y: dict = field(default_factory=lambda: {f: list(v) for f, v in cancha._OFFSETS.items()})
    copa_y_m: float = cancha.COPA_DESPLAZADA_M     # centro de los soportes respecto del centro del pie (+ izq.)
    grano_delante_m: float = cancha.GRANO_DELANTE_M  # centro del grano DELANTE de la cara del pie
    pie_ancho_m: float = cancha.ARBOL_ANCHO_PIE      # ancho del pie a la altura del LiDAR
    tablero_detras_m: float = cancha.GRANO_DELANTE_M  # cara del tablero DETRÁS del centro de los granos (dibujo)
    tablero_ancho_m: float = 0.80
    radio_grano_m: float = 0.021                   # pelotas de 42 mm
    centro_sobre_cara_m: float = cancha.CENTRO_SOBRE_SOPORTE   # esfera de 42 mm en un hueco de 25 mm
    soporte_lado_m: float = 0.07                   # soporte cuadrado (dibujo / oclusión de la parte baja del grano)
    soporte_espesor_m: float = 0.008
    ruta: str = ""

    # ------------------------------------------------------------------ archivo
    @staticmethod
    def cargar(nombre: str = "competencia", carpeta=None) -> "ModeloArbol":
        m = ModeloArbol()
        if nombre in ("", "competencia"):
            return m
        ruta = pathlib.Path(carpeta or RAIZ_PI / "arboles") / f"{nombre}.json"
        try:
            datos = json.loads(ruta.read_text())
        except OSError:
            raise ValueError(f"no existe el modelo de árbol {ruta} (crearlo con: python3 -m tools.cosecha --arbol "
                             f"{nombre} --crear-arbol)")
        for k, v in datos.items():
            if not hasattr(m, k):
                raise ValueError(f"{ruta}: '{k}' no es un campo del modelo de árbol")
            setattr(m, k, v)
        m.nombre, m.ruta = nombre, str(ruta)
        return m

    def guardar(self, nombre: str | None = None, carpeta=None) -> pathlib.Path:
        nombre = nombre or self.nombre
        ruta = pathlib.Path(carpeta or RAIZ_PI / "arboles") / f"{nombre}.json"
        ruta.parent.mkdir(parents=True, exist_ok=True)
        datos = {k: v for k, v in asdict(self).items() if k not in ("nombre", "ruta")}
        ruta.write_text(json.dumps(datos, indent=2, ensure_ascii=False) + "\n")
        self.nombre, self.ruta = nombre, str(ruta)
        return ruta

    # ------------------------------------------------------------------ geometría
    def soportes(self) -> list:
        """[(clave=(fila, col), y en el árbol, z del centro del grano)]"""
        out = []
        for fila in FILAS:
            for col, y in enumerate(self.columnas_y[fila]):
                out.append(((fila, col), float(y) + self.copa_y_m, float(self.z_cara[fila]) + self.centro_sobre_cara_m))
        return out

    @staticmethod
    def nombre_de(clave) -> str:
        return f"{clave[0]}-{clave[1] + 1}"

    @staticmethod
    def clave_de(nombre: str) -> tuple:
        fila, n = nombre.split("-")
        return fila, int(n) - 1

    @staticmethod
    def ejes(ang: float):
        u = np.array([math.cos(ang), math.sin(ang)])
        return u, np.array([-u[1], u[0]])

    def en_robot(self, pose_pie) -> dict:
        """{clave: (x, y, z)} centro de cada grano en el robot, con pose_pie = (x_cara, y_centro, ang)."""
        x, y, ang = pose_pie[:3]
        u, t = self.ejes(ang)
        base = np.array([x, y]) - self.grano_delante_m * u
        return {c: np.array([*(base + yy * t), z]) for c, yy, z in self.soportes()}

    def a_arbol(self, p_robot, pose_pie) -> np.ndarray:
        """Punto del robot -> (adelante del pie hacia el robot, izquierda, z) en el marco del árbol."""
        x, y, ang = pose_pie[:3]
        u, t = self.ejes(ang)
        d = np.asarray(p_robot[:2], float) - np.array([x, y])
        return np.array([-(d @ u), d @ t, float(p_robot[2])])

    def desde_arbol(self, q, pose_pie) -> np.ndarray:
        x, y, ang = pose_pie[:3]
        u, t = self.ejes(ang)
        p = np.array([x, y]) - q[0] * u + q[1] * t
        return np.array([p[0], p[1], q[2]])

    def esquinas_soporte(self, p_grano, ang: float) -> np.ndarray:
        """8 esquinas (en el robot) de la placa del soporte de un grano: cuadrada, centrada bajo el grano y con su
        cara de arriba `centro_sobre_cara_m` más abajo que el centro del grano (es lo que tapa la parte de abajo)."""
        u, t = self.ejes(ang)
        z_arriba = float(p_grano[2]) - self.centro_sobre_cara_m
        a = self.soporte_lado_m / 2
        out = []
        for i in (-1, 1):
            for j in (-1, 1):
                xy = np.asarray(p_grano[:2], float) + i * a * u + j * a * t
                for z in (z_arriba, z_arriba - self.soporte_espesor_m):
                    out.append([xy[0], xy[1], z])
        return np.array(out)

    def plano_granos(self, pose_pie):
        """(punto, normal) del plano vertical que pasa por el centro de los granos."""
        x, y, ang = pose_pie[:3]
        u, _ = self.ejes(ang)
        p = np.array([x, y]) - self.grano_delante_m * u
        return np.array([p[0], p[1], 0.0]), np.array([u[0], u[1], 0.0])


# ---------------------------------------------------------------------------- el pie con el LiDAR
@dataclass
class MedidaPie:
    x_cara: float          # m, en el robot
    y_centro: float
    ang: float             # rad (normal de la cara, alejándose del robot)
    ancho: float           # lo que se ve de la cara (m)
    n: int
    sigma: tuple           # incertidumbre (x, y, ang)
    centro_medido: bool    # False: el ancho no calza con el modelo y el centro es el esperado

    @property
    def pose(self):
        return (self.x_cara, self.y_centro, self.ang)


def medir_pie(pts_xy: np.ndarray, modelo: ModeloArbol, esperado=None, x_max: float = 1.6,
              x_min: float = 0.05) -> MedidaPie | None:
    """La cara del pie del árbol en un barrido del LiDAR (puntos (N,2) en el robot).
    esperado = (x, y) donde debería estar el centro de la cara (de la navegación), o None: el objeto más cercano
    delante del robot (±0,5 m de lado), más allá de x_min (lo que el LiDAR ve pegado a él suele ser el piso, un
    cable o una pieza del robot: no el árbol). Se ignoran los grupos de menos de 3 cm de ancho."""
    pts = np.asarray(pts_xy, float)
    if len(pts) == 0:
        return None
    x, y = pts[:, 0], pts[:, 1]
    if esperado is not None:
        m = ((x > max(0.05, esperado[0] - 0.30)) & (x < esperado[0] + 0.30)
             & (np.abs(y - esperado[1]) < max(0.30, modelo.pie_ancho_m)))
    else:
        m = (x > x_min) & (x < x_max) & (np.abs(y) < 0.5)
    if m.sum() < 5:
        return None
    x, y = x[m], y[m]
    # Grupos de puntos contiguos (huecos de más de 4 cm separan objetos)
    orden = np.argsort(np.arctan2(y, x))
    x, y = x[orden], y[orden]
    cortes = np.where(np.hypot(np.diff(x), np.diff(y)) > 0.04)[0] + 1
    grupos = [g for g in np.split(np.arange(len(x)), cortes)
              if len(g) >= 4 and np.hypot(np.ptp(x[g]), np.ptp(y[g])) >= 0.03]
    if not grupos:
        return None
    if esperado is not None:
        g = min(grupos, key=lambda g: abs(np.median(y[g]) - esperado[1]) + abs(np.median(x[g]) - esperado[0]))
    else:
        g = min(grupos, key=lambda g: np.median(x[g]))
    gx, gy = x[g], y[g]
    xf = np.percentile(gx, 15)
    cara = gx < xf + 0.025
    xc, yc = gx[cara], gy[cara]
    if len(xc) < 4:
        return None
    ancho = float(np.ptp(yc))
    ok = abs(ancho - modelo.pie_ancho_m) < 0.4 * modelo.pie_ancho_m + 0.02
    if ok or esperado is None:
        y_centro = 0.5 * (yc.min() + yc.max())
    else:
        y_centro = float(esperado[1])
    ang = -math.atan(np.polyfit(yc, xc, 1)[0]) if ancho > 0.08 and len(yc) > 5 else 0.0
    sigma_ang = math.radians(2.0) if ancho > 0.20 else math.radians(4.0) if ancho > 0.08 else math.radians(8.0)
    return MedidaPie(float(np.median(xc)), float(y_centro), float(ang), ancho, int(len(xc)),
                     (0.008, 0.01 if ok else 0.05, sigma_ang), bool(ok))
