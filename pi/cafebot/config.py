"""Configuración del robot en UN solo lugar (lado Raspberry).

Sistema del robot: origen en el centro de la base, x hacia ADELANTE, y hacia la IZQUIERDA,
ángulos antihorarios. Medidas en metros. Lo marcado [MEDIR] hay que confirmarlo en el robot.

Para cambiar algo sin tocar el código: crear pi/robot_local.json con las claves a cambiar, p. ej.
    {"geometria": {"lidar_x": 0.17, "extras": [[0.12, 0.0, 0.05, 0.05]]},
     "puertos": {"mega": "/dev/ttyACM0", "lidar": "/dev/ttyUSB0"}}
"""
from __future__ import annotations

import json
import math
import pathlib
from dataclasses import asdict, dataclass, field

from cafebot.drivers.ld06 import ConfigLD06


@dataclass
class GeometriaRobot:
    largo: float = 0.30            # base, a lo largo de x
    ancho: float = 0.23            # base, a lo largo de y
    margen: float = 0.03           # holgura del filtro de huella (cables, ruedas, vibración)
    lidar_x: float = 0.16          # LD06 en el frente de la base, a ~8 cm del suelo   [MEDIR]
    lidar_y: float = 0.0
    lidar_yaw_deg: float = 0.0     # hacia dónde mira el 0° del LD06 (0 = adelante)    [MEDIR]
    lidar_fov_deg: float = 95.0    # vista útil ±fov respecto del frente del LD06
    # Rectángulos extra que el LiDAR ve del propio robot (cx, cy, semiancho_x, semiancho_y):
    # p. ej. el brazo plegado o una rueda que asoma. Se dibujan en azul en la interfaz.
    extras: list = field(default_factory=list)

    @property
    def radio(self) -> float:
        """Radio del círculo que envuelve la base: lo usa el planificador."""
        return math.hypot(self.largo / 2, self.ancho / 2)

    def huella(self) -> tuple[tuple[float, float], tuple[float, float]]:
        hx, hy = self.largo / 2 + self.margen, self.ancho / 2 + self.margen
        return (-hx, hx), (-hy, hy)

    def config_ld06(self) -> ConfigLD06:
        hx, hy = self.huella()
        return ConfigLD06(x_m=self.lidar_x, y_m=self.lidar_y, yaw_deg=self.lidar_yaw_deg,
                          fov_deg=self.lidar_fov_deg, huella_x_m=hx, huella_y_m=hy,
                          extras=[tuple(e) for e in self.extras])


@dataclass
class Puertos:
    mega: str = "/dev/cafebot-mega"     # sin reglas udev: /dev/ttyACM0
    lidar: str = "/dev/cafebot-lidar"   # sin reglas udev: /dev/ttyUSB0
    u2d2: str = "/dev/cafebot-u2d2"
    imu_bus: int = 1
    imu_dir: int = 0x68


@dataclass
class ConfigNavegacion:
    mapa_lado_m: float = 10.0      # el mapa del SLAM (la cancha mide 3,6 x 3,0)
    mapa_res_m: float = 0.04
    plan_res_m: float = 0.08
    margen_obstaculos_m: float = 0.05  # distancia mínima libre entre la base y un obstáculo (albercas: no tocar)
    seguridad_m: float = 0.10      # además, rutas que se alejan de los obstáculos cuando pueden
    vel_mm_s: float = 150.0        # velocidad de los tramos rectos
    tramo_max_m: float = 0.50      # largo máx. de cada tramo: después se corrige con el SLAM
    tol_llegada_m: float = 0.04
    tol_rumbo_deg: float = 3.0     # bajo esto no se gira antes de avanzar
    distancia_aprox_m: float = 0.25  # holgura entre el frente del robot y el árbol al aproximarse


@dataclass
class Config:
    geometria: GeometriaRobot = field(default_factory=GeometriaRobot)
    puertos: Puertos = field(default_factory=Puertos)
    navegacion: ConfigNavegacion = field(default_factory=ConfigNavegacion)

    def a_dict(self) -> dict:
        return asdict(self)


def cargar(ruta: str | None = None) -> Config:
    """Config por defecto + lo que haya en pi/robot_local.json (si existe)."""
    cfg = Config()
    p = pathlib.Path(ruta) if ruta else pathlib.Path(__file__).resolve().parents[1] / "robot_local.json"
    if p.exists():
        datos = json.loads(p.read_text())
        for seccion, valores in datos.items():
            obj = getattr(cfg, seccion, None)
            if obj is None:
                raise KeyError(f"{p}: sección desconocida '{seccion}'")
            for k, v in valores.items():
                if not hasattr(obj, k):
                    raise KeyError(f"{p}: '{seccion}.{k}' no existe")
                setattr(obj, k, v)
    return cfg
