"""Driver del LiDAR LDROBOT LD06 (también sirve para LD19).

Mejoras respecto a legacy/codigo_brazo/ld06.py:
  - Verifica el CRC8 de cada paquete (descarta los corruptos en vez de dibujar basura).
  - Resincroniza si se pierden bytes (busca 0x54 0x2C).
  - Agrupa los paquetes en vueltas completas de 360°.
  - Corre en un hilo: quien lo usa nunca se bloquea esperando al puerto.
  - Convierte al sistema del robot (x adelante, y izquierda, ángulo antihorario).

Uso:
    lidar = LD06("/dev/cafebot-lidar")
    lidar.iniciar()
    scan = lidar.ultimo_scan()      # None hasta la primera vuelta
    x, y = scan.xy()                 # metros, sistema del robot
    lidar.detener()
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass, field

import numpy as np

CABECERA = 0x54
VER_LEN = 0x2C          # 12 puntos por paquete
LARGO_PAQUETE = 47
PUNTOS_POR_PAQUETE = 12
BAUDIOS = 230400

CRC_TABLA = [
    0x00, 0x4D, 0x9A, 0xD7, 0x79, 0x34, 0xE3, 0xAE, 0xF2, 0xBF, 0x68, 0x25, 0x8B, 0xC6, 0x11, 0x5C,
    0xA9, 0xE4, 0x33, 0x7E, 0xD0, 0x9D, 0x4A, 0x07, 0x5B, 0x16, 0xC1, 0x8C, 0x22, 0x6F, 0xB8, 0xF5,
    0x1F, 0x52, 0x85, 0xC8, 0x66, 0x2B, 0xFC, 0xB1, 0xED, 0xA0, 0x77, 0x3A, 0x94, 0xD9, 0x0E, 0x43,
    0xB6, 0xFB, 0x2C, 0x61, 0xCF, 0x82, 0x55, 0x18, 0x44, 0x09, 0xDE, 0x93, 0x3D, 0x70, 0xA7, 0xEA,
    0x3E, 0x73, 0xA4, 0xE9, 0x47, 0x0A, 0xDD, 0x90, 0xCC, 0x81, 0x56, 0x1B, 0xB5, 0xF8, 0x2F, 0x62,
    0x97, 0xDA, 0x0D, 0x40, 0xEE, 0xA3, 0x74, 0x39, 0x65, 0x28, 0xFF, 0xB2, 0x1C, 0x51, 0x86, 0xCB,
    0x21, 0x6C, 0xBB, 0xF6, 0x58, 0x15, 0xC2, 0x8F, 0xD3, 0x9E, 0x49, 0x04, 0xAA, 0xE7, 0x30, 0x7D,
    0x88, 0xC5, 0x12, 0x5F, 0xF1, 0xBC, 0x6B, 0x26, 0x7A, 0x37, 0xE0, 0xAD, 0x03, 0x4E, 0x99, 0xD4,
    0x7C, 0x31, 0xE6, 0xAB, 0x05, 0x48, 0x9F, 0xD2, 0x8E, 0xC3, 0x14, 0x59, 0xF7, 0xBA, 0x6D, 0x20,
    0xD5, 0x98, 0x4F, 0x02, 0xAC, 0xE1, 0x36, 0x7B, 0x27, 0x6A, 0xBD, 0xF0, 0x5E, 0x13, 0xC4, 0x89,
    0x63, 0x2E, 0xF9, 0xB4, 0x1A, 0x57, 0x80, 0xCD, 0x91, 0xDC, 0x0B, 0x46, 0xE8, 0xA5, 0x72, 0x3F,
    0xCA, 0x87, 0x50, 0x1D, 0xB3, 0xFE, 0x29, 0x64, 0x38, 0x75, 0xA2, 0xEF, 0x41, 0x0C, 0xDB, 0x96,
    0x42, 0x0F, 0xD8, 0x95, 0x3B, 0x76, 0xA1, 0xEC, 0xB0, 0xFD, 0x2A, 0x67, 0xC9, 0x84, 0x53, 0x1E,
    0xEB, 0xA6, 0x71, 0x3C, 0x92, 0xDF, 0x08, 0x45, 0x19, 0x54, 0x83, 0xCE, 0x60, 0x2D, 0xFA, 0xB7,
    0x5D, 0x10, 0xC7, 0x8A, 0x24, 0x69, 0xBE, 0xF3, 0xAF, 0xE2, 0x35, 0x78, 0xD6, 0x9B, 0x4C, 0x01,
    0xF4, 0xB9, 0x6E, 0x23, 0x8D, 0xC0, 0x17, 0x5A, 0x06, 0x4B, 0x9C, 0xD1, 0x7F, 0x32, 0xE5, 0xA8,
]


def crc8(datos: bytes) -> int:
    crc = 0
    for b in datos:
        crc = CRC_TABLA[(crc ^ b) & 0xFF]
    return crc


@dataclass
class Paquete:
    velocidad_dps: float            # grados/s del rotor
    angulos_deg: np.ndarray         # 12 ángulos en grados del LiDAR (horario, 0 = frente del sensor)
    distancias_mm: np.ndarray
    intensidades: np.ndarray
    marca_ms: int


def decodificar_paquete(p: bytes) -> Paquete | None:
    """Decodifica 47 bytes. Devuelve None si el CRC o la cabecera no calzan."""
    if len(p) != LARGO_PAQUETE or p[0] != CABECERA or p[1] != VER_LEN:
        return None
    if crc8(p[:-1]) != p[-1]:
        return None
    velocidad, ang_ini = struct.unpack_from("<HH", p, 2)
    puntos = np.frombuffer(p, dtype=np.uint8, count=PUNTOS_POR_PAQUETE * 3, offset=6).reshape(-1, 3)
    distancias = puntos[:, 0].astype(np.uint16) | (puntos[:, 1].astype(np.uint16) << 8)
    intensidades = puntos[:, 2].copy()
    ang_fin, marca = struct.unpack_from("<HH", p, 42)
    ini, fin = ang_ini / 100.0, ang_fin / 100.0
    if fin < ini:
        fin += 360.0
    angulos = np.linspace(ini, fin, PUNTOS_POR_PAQUETE) % 360.0
    return Paquete(velocidad, angulos, distancias.astype(np.float32), intensidades, marca)


@dataclass
class Scan:
    """Una vuelta completa, ya en el sistema del robot."""
    t: float                        # time.monotonic() al cerrar la vuelta
    angulos: np.ndarray             # rad, antihorario, 0 = adelante del robot
    distancias: np.ndarray          # metros
    intensidades: np.ndarray
    rpm: float = 0.0
    puntos_propios: int = 0         # puntos descartados por caer sobre el propio robot

    def xy(self) -> tuple[np.ndarray, np.ndarray]:
        return self.distancias * np.cos(self.angulos), self.distancias * np.sin(self.angulos)


@dataclass
class ConfigLD06:
    # Montaje del LiDAR en el robot. Plan actual: en el frente de la base (30 x 23 cm),
    # a ~8 cm del suelo, con ~180° de vista despejada hacia adelante.
    x_m: float = 0.16               # posición del LiDAR respecto al centro del robot  [MEDIR]
    y_m: float = 0.0
    yaw_deg: float = 0.0            # hacia dónde mira el "0°" del LiDAR (0 = adelante)
    horario: bool = True            # el LD06 entrega ángulos en sentido horario
    fov_deg: float = 95.0           # se usan solo los puntos a ±fov del frente del SENSOR
                                    # (detrás está el propio robot)
    # Filtros
    dist_min_m: float = 0.03       # el LD06 mide desde 2 cm; frente al árbol para agarrar queda a ~5 cm
    dist_max_m: float = 6.0
    intensidad_min: int = 10
    # Huella del robot (rectángulo centrado en el robot) para borrar el propio chasis/brazo
    huella_x_m: tuple[float, float] = (-0.17, 0.17)
    huella_y_m: tuple[float, float] = (-0.14, 0.14)
    sectores_ciegos_deg: list[tuple[float, float]] = field(default_factory=list)
    # Rectángulos extra a borrar (cx, cy, semiancho_x, semiancho_y) en el sistema del robot:
    # brazo, ruedas que asoman, mástil, etc.
    extras: list[tuple[float, float, float, float]] = field(default_factory=list)


def mascara_huella(x: np.ndarray, y: np.ndarray, cfg: ConfigLD06) -> np.ndarray:
    """True para los puntos que caen sobre el propio robot (chasis + extras)."""
    m = ((x > cfg.huella_x_m[0]) & (x < cfg.huella_x_m[1]) &
         (y > cfg.huella_y_m[0]) & (y < cfg.huella_y_m[1]))
    for cx, cy, hx, hy in cfg.extras:
        m |= (np.abs(x - cx) <= hx) & (np.abs(y - cy) <= hy)
    return m


def paquetes_a_scan(paquetes: list[Paquete], cfg: ConfigLD06) -> Scan:
    ang = np.concatenate([p.angulos_deg for p in paquetes])
    dist = np.concatenate([p.distancias_mm for p in paquetes]) / 1000.0
    inten = np.concatenate([p.intensidades for p in paquetes])
    rpm = float(np.mean([p.velocidad_dps for p in paquetes])) / 6.0 if paquetes else 0.0

    ok = (dist >= cfg.dist_min_m) & (dist <= cfg.dist_max_m) & (inten >= cfg.intensidad_min)
    for a0, a1 in cfg.sectores_ciegos_deg:
        ok &= ~((ang >= a0) & (ang <= a1))
    ang, dist, inten = ang[ok], dist[ok], inten[ok]

    # Campo de visión útil (en el sistema del sensor, antihorario, -180..180)
    a_ccw = (-ang if cfg.horario else ang)
    a_ccw = (a_ccw + 180.0) % 360.0 - 180.0
    vista = np.abs(a_ccw) <= cfg.fov_deg
    ang, dist, inten, a_ccw = ang[vista], dist[vista], inten[vista], a_ccw[vista]

    # Sensor -> robot
    a_sensor = np.radians(a_ccw) + math.radians(cfg.yaw_deg)
    x = cfg.x_m + dist * np.cos(a_sensor)
    y = cfg.y_m + dist * np.sin(a_sensor)
    propios = mascara_huella(x, y, cfg)
    x, y, inten = x[~propios], y[~propios], inten[~propios]
    ang_r = np.arctan2(y, x)
    orden = np.argsort(ang_r)
    return Scan(time.monotonic(), ang_r[orden], np.hypot(x, y)[orden], inten[orden], rpm,
                puntos_propios=int(propios.sum()))


class Ensamblador:
    """Recibe bytes crudos y entrega vueltas completas. Separado del hilo para poder probarlo."""

    def __init__(self, cfg: ConfigLD06):
        self.cfg = cfg
        self.buf = bytearray()
        self.vuelta: list[Paquete] = []
        self.ultimo_angulo = -1.0
        self.paquetes_ok = 0
        self.paquetes_malos = 0

    def alimentar(self, datos: bytes) -> list[Scan]:
        self.buf.extend(datos)
        scans: list[Scan] = []
        while True:
            i = self.buf.find(bytes((CABECERA, VER_LEN)))
            if i < 0:
                del self.buf[:-1]  # puede quedar un 0x54 suelto al final
                break
            if i > 0:
                del self.buf[:i]
            if len(self.buf) < LARGO_PAQUETE:
                break
            paq = decodificar_paquete(bytes(self.buf[:LARGO_PAQUETE]))
            if paq is None:
                self.paquetes_malos += 1
                del self.buf[:1]  # resincronizar desde el byte siguiente
                continue
            del self.buf[:LARGO_PAQUETE]
            self.paquetes_ok += 1
            a0 = float(paq.angulos_deg[0])
            if self.vuelta and a0 < self.ultimo_angulo - 180.0:  # el ángulo dio la vuelta
                scans.append(paquetes_a_scan(self.vuelta, self.cfg))
                self.vuelta = []
            self.vuelta.append(paq)
            self.ultimo_angulo = a0
        return scans


class LD06:
    def __init__(self, puerto: str, cfg: ConfigLD06 | None = None):
        self.puerto = puerto
        self.cfg = cfg or ConfigLD06()
        self._ens = Ensamblador(self.cfg)
        self._scan: Scan | None = None
        self._n_scans = 0
        self._cv = threading.Condition()
        self._hilo: threading.Thread | None = None
        self._corriendo = False
        self._serial = None

    def iniciar(self) -> None:
        import serial  # pyserial

        self._serial = serial.Serial(self.puerto, BAUDIOS, timeout=0.05)
        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle, name="ld06", daemon=True)
        self._hilo.start()

    def _bucle(self) -> None:
        while self._corriendo:
            datos = self._serial.read(512)
            if not datos:
                continue
            for scan in self._ens.alimentar(datos):
                with self._cv:
                    self._scan = scan
                    self._n_scans += 1
                    self._cv.notify_all()

    def ultimo_scan(self) -> Scan | None:
        with self._cv:
            return self._scan

    def esperar_scan(self, timeout: float = 1.0) -> Scan | None:
        """Bloquea hasta que llegue una vuelta NUEVA (útil tras detener el robot)."""
        with self._cv:
            n = self._n_scans
            self._cv.wait_for(lambda: self._n_scans > n, timeout)
            return self._scan if self._n_scans > n else None

    def estadisticas(self) -> dict:
        return {"ok": self._ens.paquetes_ok, "crc_malos": self._ens.paquetes_malos, "scans": self._n_scans}

    def detener(self) -> None:
        self._corriendo = False
        if self._hilo:
            self._hilo.join(timeout=1.0)
        if self._serial:
            self._serial.close()
