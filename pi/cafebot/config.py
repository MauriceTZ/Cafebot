"""Configuración del robot en UN solo lugar (lado Raspberry).

Sistema del robot: origen en el centro de la base, x hacia ADELANTE, y hacia la IZQUIERDA,
ángulos antihorarios. Medidas en metros. Lo marcado [MEDIR] hay que confirmarlo en el robot.

Para cambiar algo sin tocar el código: crear pi/robot_local.json con las claves a cambiar, p. ej.
    {"geometria": {"lidar_x": 0.17, "extras": [[0.12, 0.0, 0.05, 0.05]]},
     "puertos": {"mega": "/dev/ttyUSB1", "lidar": "/dev/ttyUSB0"}}
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
    lidar_x: float = 0.16          # LD06 al frente, por FUERA de la base, centrado      [MEDIR]
    lidar_z: float = 0.05          # 29 oct: a ~5 cm del suelo, confirmado (solo informativo)
    lidar_y: float = 0.0
    lidar_yaw_deg: float = 0.0     # hacia dónde mira el 0° del LD06 (0 = adelante)    [MEDIR]
    lidar_fov_deg: float = 95.0    # vista útil ±fov respecto del frente del LD06
    # Rectángulos extra que el LiDAR ve del propio robot (cx, cy, semiancho_x, semiancho_y):
    # p. ej. el brazo plegado o una rueda que asoma. Se dibujan en azul en la interfaz.
    extras: list = field(default_factory=list)
    # El LiDAR ve su propia base: se borra un CUADRADO de este lado DETRÁS del LiDAR, centrado en él (el robot
    # no puede medir más de 450 mm por lado según el reglamento). 0 = solo la huella de la base (+ margen).
    lidar_caja_m: float = 0.45

    @property
    def radio(self) -> float:
        """Radio del círculo que envuelve la base: lo usa el planificador."""
        return math.hypot(self.largo / 2, self.ancho / 2)

    def huella(self) -> tuple[tuple[float, float], tuple[float, float]]:
        """Rectángulo (sistema del robot) donde todo lo que ve el LiDAR es el propio robot: la base + margen y,
        si lidar_caja_m > 0, el cuadrado de 45 cm detrás del LiDAR (lo que el robot puede llegar a medir)."""
        hx, hy = self.largo / 2 + self.margen, self.ancho / 2 + self.margen
        x0, x1, y0, y1 = -hx, hx, -hy, hy
        if self.lidar_caja_m > 0:
            # adelante, solo hasta 1 cm delante del LiDAR (no el margen de la base: el LiDAR va por fuera de ella)
            x0, x1 = min(x0, self.lidar_x - self.lidar_caja_m), max(self.largo / 2, self.lidar_x + 0.01)
            y0, y1 = min(y0, self.lidar_y - self.lidar_caja_m / 2), max(y1, self.lidar_y + self.lidar_caja_m / 2)
        return (x0, x1), (y0, y1)

    def config_ld06(self) -> ConfigLD06:
        hx, hy = self.huella()
        return ConfigLD06(x_m=self.lidar_x, y_m=self.lidar_y, yaw_deg=self.lidar_yaw_deg,
                          fov_deg=self.lidar_fov_deg, huella_x_m=hx, huella_y_m=hy,
                          extras=[tuple(e) for e in self.extras])


@dataclass
class Puertos:
    mega: str = "/dev/cafebot-mega"     # sin reglas udev: /dev/ttyUSB1 (nuestro Mega es un clon con CH340)
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
class ConfigIR:
    """Sensores de línea: 4 × QTR-1A en A8..A11 del Mega. Orden fijo DI, DD, TI, TD (= bits 0..3).
    Calibración y prueba: `python3 -m tools.ir` (docs/ir.md)."""
    nombres: tuple = ("DI", "DD", "TI", "TD")
    # Punto del piso que mira cada sensor, en el sistema del robot (x adelante, y izquierda), en metros.
    # 29 oct: van en las 4 esquinas, a 45°, junto a cada rueda -> se toma el centro de cada rueda
    # (firmware: CINEMATICA_L = 223 mm entre ejes, CINEMATICA_W = 217 mm entre ruedas). Si el punto que mira
    # el sensor queda corrido de la rueda, medirlo y ponerlo en robot_local.json.            [MEDIR]
    posiciones: tuple = ((0.1115, 0.1085), (0.1115, -0.1085), (-0.1115, 0.1085), (-0.1115, -0.1085))
    # ---- UMBRALES ------------------------------------------------------------------------------
    # Cuentas del ADC del Mega (0..1023). NEGRO si la lectura pasa umbral + histéresis/2.
    # None = usar los guardados en la EEPROM del Mega. Con valores, tools.navegacion / tools.mision se
    # los envían al Mega al conectar (así quedan en el repo). `tools.ir` los guarda aquí al calibrar.
    umbrales: tuple | None = None
    histeresis: int | None = None
    # ---- Uso en la navegación ----
    ancho_linea_m: float = 0.019
    asociar_m: float = 0.05        # distancia máx. de un cruce a una línea conocida de la cancha
    corregir_pose: bool = False    # corregir el EKF con los cruces asociados (activar tras validarlo)
    sigma_m: float = 0.008         # incertidumbre de la posición de un cruce


@dataclass
class ConfigMision:
    nivel: int = 2                 # 1 = granos dentro del robot | 2 = + entregar | 3 = + sobremaduros y más árboles
    duracion_s: float = 480.0      # ronda de 8 min
    margen_final_s: float = 12.0   # detenerse antes del final por si acaso
    max_arboles: int = 1           # nivel 3 usa hasta 3
    recoger: tuple = ("MADURO",)   # nivel 3 agrega "SOBREMADURO"
    confianza_min: float = 0.70    # bajo esto la visión cuenta como NO_SE (no tocar)
    # Brazo: eje de giro de la base en el sistema del robot; z = altura de la BASE del brazo sobre
    # el SUELO (el hombro está 6,7 cm más arriba). Se comprueba con `tools.brazo leer`.        [MEDIR]
    brazo_x: float = 0.10
    brazo_y: float = 0.0
    brazo_z: float = 0.36
    # Árbol: distancias entre el FRENTE del robot y la cara del tablero
    distancia_foto_m: float = 0.55     # para la foto de los 16 soportes (la cámara se centra en el árbol). Con la
                                       # cámara al frente, a 25 cm del suelo e inclinada 25–32°: 16/16 desde 55 cm
                                       # (a 50 cm la fila alta casi no asoma sobre sus soportes; simulado, 29 oct).
                                       # Más lejos el robot quedaría encima de una alberca detrás.  [AJUSTAR]
    desplazamiento_foto_m: float = 0.10  # si algún soporte queda fuera de la foto: cangrejo y otra foto
    holgura_agarre_m: float = 0.06     # para recoger: con la base del brazo a 36 cm, a 8 cm la fila alta ya no
                                       # se alcanza (docs/brazo.md). Nada del robot sobre 38 cm en el frente. [AJUSTAR]
    estaciones_y: tuple = (0.24, 0.12, 0.0, -0.12, -0.24)   # posiciones laterales del brazo frente al árbol
    orden_filas: tuple = ("alta", "media", "baja")
    camara_arbol_deg: float = 25.0     # X 2: inclinación de la cámara para la foto del árbol   [AJUSTAR]
    camara_cajas_deg: float = 0.0      # X 2: para ver las cajas                                 [AJUSTAR]
    # Color de las cajas: la cámara mira adelante, así que antes de entregar se para frente a una caja
    x_vista_cajas: float = 1.25        # centro del robot, mirando hacia el beneficiadero (θ = 180°)
    # Entrega (de reversa: las compuertas están atrás)
    x_aproximacion_entrega: float = 0.95   # centro del robot antes de retroceder (cancha, m)
    x_entrega: float = 0.77               # centro del robot al abrir: atrás queda ~2 cm de la caja   [AJUSTAR]
    salida_x: float = -0.21               # dónde caen los granos, en el sistema del robot           [MEDIR]
                                          # (la rampa debe sobresalir ~5 cm por detrás: con la salida justo en
                                          #  el borde trasero, 1 cm de error de pose deja los granos fuera, −7 c/u)
    compuerta_abierta_deg: tuple = (90.0, 90.0)   # X 0 (maduros) y X 1 (sobremaduros)             [MEDIR]
    compuerta_cerrada_deg: tuple = (0.0, 0.0)     #                                                [MEDIR]
    t_vaciado_s: float = 2.5
    freno_ir: bool = False         # IS 1 durante la misión (cuando los IR estén integrados)
    pin_boton: int = 17            # GPIO BCM del botón de inicio                                  [CONFIRMAR]


@dataclass
class ConfigBrazo:
    """Brazo real (cafebot/arm/brazo.py). La posición de su base está en ConfigMision (brazo_x/y/z).
    Ids, ceros, sentidos, límites y posturas de cada brazo: su PERFIL, pi/brazos/<perfil>.json (tools.brazo)."""
    perfil: str = "principal"      # qué brazo usa la misión (pi/brazos/principal.json)
    yaw0_deg: float = 0.0          # corrección extra de la base; con `tools.brazo cero` (base mirando adelante) = 0
    yaw_signo: int = 1             # ídem: el sentido de la base lo fija `tools.brazo verificar` en el perfil
    garra_largo_m: float = 0.032   # eje de la muñeca -> centro de los dedos cerrados                    [MEDIR]
    garra_abierta_deg: float = 0.0
    garra_cerrada_deg: float = 65.0
    garra_vacia_deg: float = 58.0  # si al cerrar la garra pasa de este ángulo, no tomó nada               [MEDIR]
    pitch_preferido_deg: float = 90.0  # garra horizontal; si no alcanza, prueba en [pitch_min, pitch_max]
    pitch_min_deg: float = 55.0        # 90 = horizontal, menos = garra apuntando hacia arriba
    pitch_max_deg: float = 100.0
    aproximacion_m: float = 0.06   # el agarre parte 6 cm antes del grano y entra en línea recta
    levantar_m: float = 0.03       # sube el grano para sacarlo del hueco del soporte (25 mm)
    retirar_m: float = 0.08        # y retrocede antes de ir al depósito
    r_min_m: float = 0.05          # la secuencia nunca acerca la garra a menos de esto del eje de la base (más cerca
                                   # pasaría sobre el robot o el propio brazo): acorta la aproximación y el retiro.
                                   # Con brazo_x = 0,10 el frente del robot queda a 5 cm del eje         [AJUSTAR]
    t_mov_s: float = 1.0           # duración de un movimiento grande
    t_fino_s: float = 0.4          # duración de cada paso de la aproximación
    error_max_deg: float = 6.0     # si una articulación queda más lejos de su meta: algo la bloquea, se aborta
    carga_max_pct: float = 80.0    # ídem con la carga estimada
    postura_espera: str = "descanso"    # entre un grano y otro: plegado DETRÁS del frente del robot. "trabajo"
                                        # deja la garra a la altura de la fila media y 1 cm delante: chocaría
    descanso_sin_torque: bool = False   # True SOLO si el brazo plegado queda APOYADO (si no, se cae)
    archivo_posturas: str = "brazo_posturas.json"   # ANTERIOR (en pi/): sus posturas pasan al perfil la 1ª vez


@dataclass
class ConfigCamara:
    """Pi Camera Module 3 (estándar: 66° × 41°). Modelo completo en cafebot/vision/camara_modelo.py.
    29 oct: FIJA al frente del brazo, en el eje de su base, a 25 cm del suelo (pegada con silicona, casi
    horizontal: a 55 cm del árbol solo se ve la fila baja). Recomendado: inclinarla ~28° hacia arriba.
    La pose se calibra con el árbol (python3 -m tools.cosecha, tecla C) y los intrínsecos con un tablero de
    ajedrez (python3 -m tools.camara intrinseca); los dos quedan en robot_local.json."""
    x: float = 0.15                 # posición del LENTE en el sistema del robot (m)                 [MEDIR]
    y: float = 0.0                  # en el eje de la base del brazo                                 [MEDIR]
    z: float = 0.25                 # 29 oct: 25 cm del suelo
    inclinacion_arbol_deg: float = 28.0    # hacia ARRIBA (pitch). Hoy ~0°: re-montarla a ~28°     [AJUSTAR: tecla C]
    inclinacion_cajas_deg: float = -25.0   # hacia ABAJO (con los servos de la cámara, cuando estén) [AJUSTAR]
    yaw_deg: float = 0.0            # + = mirando hacia la izquierda                                 [AJUSTAR: tecla C]
    roll_deg: float = 0.0           #                                                                [AJUSTAR: tecla C]
    ancho_px: int = 1536
    alto_px: int = 864
    fx_px: float = 1183.0           # 768 / tan(33°): Camera Module 3 estándar a 1536×864     [AJUSTAR: intrinseca]
    fy_px: float = 0.0              # 0 = igual a fx
    cx_px: float = 0.0              # 0 = centro de la imagen
    cy_px: float = 0.0
    dist: list = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0])   # k1 k2 p1 p2 k3 (OpenCV)
    foco_m: float = 0.6             # foco FIJO (el autofoco cambia la distancia focal): LensPosition = 1/foco_m
    arbol: str = "competencia"      # forma del árbol: "competencia" (Apéndice B) o pi/arboles/<nombre>.json
    sat_min: int = 130              # pelota = saturación HSV > sat_min (0..255). 29 oct, foto del taller:
    val_min: int = 60               #   madera 85–110, pelotas roja/verde 150–190
    fotos: int = 3                  # se clasifica con varias fotos: un soporte que cambia entre fotos = NO_SE
    guardar_fotos: bool = True      # en pi/capturas/, con lo clasificado dibujado (para revisar después)
    calibrada: str = ""             # cuándo y cómo se calibró la pose (tools.cosecha C / N). Vacío = sin calibrar:
                                    # el ajuste de cada foto deja moverse inclinación/yaw/roll ±3–4° (manda el LiDAR)


@dataclass
class Config:
    geometria: GeometriaRobot = field(default_factory=GeometriaRobot)
    puertos: Puertos = field(default_factory=Puertos)
    navegacion: ConfigNavegacion = field(default_factory=ConfigNavegacion)
    ir: ConfigIR = field(default_factory=ConfigIR)
    mision: ConfigMision = field(default_factory=ConfigMision)
    brazo: ConfigBrazo = field(default_factory=ConfigBrazo)
    camara: ConfigCamara = field(default_factory=ConfigCamara)

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
                setattr(obj, k, tuple(v) if isinstance(v, list) else v)
    return cfg


def guardar_local(seccion: str, valores: dict, ruta: str | None = None) -> pathlib.Path:
    """Agrega o cambia claves en pi/robot_local.json (lo usan las herramientas de calibración)."""
    p = pathlib.Path(ruta) if ruta else pathlib.Path(__file__).resolve().parents[1] / "robot_local.json"
    datos = json.loads(p.read_text()) if p.exists() else {}
    datos.setdefault(seccion, {}).update(valores)
    p.write_text(json.dumps(datos, indent=2, ensure_ascii=False) + "\n")
    return p
