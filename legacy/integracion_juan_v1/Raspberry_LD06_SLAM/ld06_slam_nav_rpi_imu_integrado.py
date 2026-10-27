import serial
import threading
import time
import heapq
import json
import argparse
import math
import re
import glob
import atexit
from collections import deque
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment
from scipy.ndimage import distance_transform_edt
from sklearn.cluster import DBSCAN
import pygame as pg
import pygame.gfxdraw as gfxdraw

# Módulos de la cosecha (misma carpeta). La visión (OpenCV/cámara) se importa solo con --vision.
from brazo_interfaz import BrazoNoIntegrado, BrazoSimulado
from sensores_ir import SensoresIRBorde
from mision_cosecha import MisionCosecha, ConfigMision, Estado as EstadoMision

# =====================================================================
# 1. TABLA CRC-8 Y LECTOR MULTIHILO DE REVOLUCIÓN COMPLETA (LD06)
# =====================================================================
crc_table = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33,
    0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b,
    0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43, 0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44,
    0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b,
    0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86,
    0xcb, 0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5,
    0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab, 0x05,
    0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20, 0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b,
    0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b,
    0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c,
    0xdb, 0x96, 0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e, 0xeb,
    0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01, 0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17,
    0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
]

START_CHARACTER = 0x54
DATA_LENGTH_MASK = 0x1F
DEFAULT_BAUD = 230400           # baudios de fábrica del LD06 (antes faltaba esta constante)

# ---- Geometría del robot: el LD06 va en el CENTRO del chasis ----
# Marco local del LiDAR (el mismo de polar_to_local_cartesian): x = sin(ang)*d, y = cos(ang)*d,
# es decir, ángulo 0 = +Y = frente del robot. El LARGO (30 cm) va a lo largo de Y y el ANCHO
# (23 cm) a lo largo de X. Si el LD06 está montado girado 90 grados, intercambia largo y ancho
# con --robot-largo / --robot-ancho (el rectángulo azul dibujado en el mapa debe envolver al robot).
ROBOT_LENGTH_M = 0.30
ROBOT_WIDTH_M = 0.23
SELF_MARGIN_M = 0.03            # holgura alrededor del chasis (cables, tornillos, ruedas, vibración)


class RobotFootprint:
    """Huella rectangular del propio robot en el marco del LiDAR (centrada en el sensor).
    Los puntos que caen dentro son el chasis viendo el chasis: se descartan ANTES del SLAM, del
    mapa y del tracker, porque (1) quedan siempre en el mismo lugar respecto del sensor y
    arrastran al ICP hacia 'no me muevo', (2) se acumulan en el mapa como un obstáculo estático
    pegado al robot y (3) el tracker los clusteriza como un objeto."""

    def __init__(self, length=ROBOT_LENGTH_M, width=ROBOT_WIDTH_M, margin=SELF_MARGIN_M, extra=()):
        self.length, self.width, self.margin = length, width, margin
        self.hx = width / 2.0 + margin
        self.hy = length / 2.0 + margin
        # Rectángulos extra (cx, cy, semiancho_x, semialto_y) en el marco local: mástil, brazo, etc.
        self.extra = [tuple(map(float, r)) for r in extra]

    @property
    def circumradius(self):
        """Radio del círculo que envuelve al robot (sin margen): para el planificador."""
        return math.hypot(self.length / 2.0, self.width / 2.0)

    def contains(self, x, y):
        if abs(x) <= self.hx and abs(y) <= self.hy:
            return True
        for cx, cy, hx, hy in self.extra:
            if abs(x - cx) <= hx and abs(y - cy) <= hy:
                return True
        return False

    def corners(self):
        """Esquinas del rectángulo (con margen) en el marco local, para dibujarlo."""
        return np.array([[-self.hx, -self.hy], [self.hx, -self.hy],
                         [self.hx, self.hy], [-self.hx, self.hy]])


class ScanAssembler:
    """Convierte paquetes LD06 validos (47 bytes) en revoluciones completas de 360 grados."""

    def __init__(self, footprint=None):
        # Array (N, 3): [angle_rad, dist_m, confidence]
        self.latest_scan = None
        self.scan_id = 0
        self.latest_time = 0.0      # time.monotonic() al completarse la revolución
        self.lock = threading.Lock()
        self._current_rev = []
        self._prev_angle = 0.0
        self.footprint = footprint      # RobotFootprint o None (sin filtro)
        self.self_filtered = 0          # puntos propios descartados en total
        self.self_last_rev = 0          # ... en la última revolución completa
        self._self_cur = 0

    def feed(self, packet: bytes):
        data_length = packet[1] & DATA_LENGTH_MASK
        if data_length <= 1:
            return

        start_angle = int.from_bytes(packet[4:6], byteorder="little")
        end_angle = int.from_bytes(packet[42:44], byteorder="little")

        if start_angle <= end_angle:
            step_angle = (end_angle - start_angle) / (data_length - 1)
        else:
            step_angle = (end_angle + 36000 - start_angle) / (data_length - 1)

        for i in range(data_length):
            offset = 6 + i * 3
            distance_mm = int.from_bytes(packet[offset:offset+2], byteorder="little")
            confidence = packet[offset+2]
            angle_cdeg = (start_angle + step_angle * i) % 36000
            angle_deg = angle_cdeg / 100.0

            # Detectar fin de revolucion de 360 grados
            if angle_deg < self._prev_angle and (self._prev_angle - angle_deg) > 180.0:
                if len(self._current_rev) > 120:
                    with self.lock:
                        self.latest_scan = np.array(self._current_rev, dtype=np.float64)
                        self.scan_id += 1
                        self.latest_time = time.monotonic()
                self.self_last_rev = self._self_cur
                self._self_cur = 0
                self._current_rev = []
            self._prev_angle = angle_deg

            if 150 <= distance_mm <= 12000 and confidence >= 100:
                ang = math.radians(angle_deg)
                dist_m = distance_mm / 1000.0
                # Autodetección: descartar los impactos que caen sobre el propio chasis.
                if self.footprint is not None and self.footprint.contains(
                        math.sin(ang) * dist_m, math.cos(ang) * dist_m):
                    self.self_filtered += 1
                    self._self_cur += 1
                    continue
                self._current_rev.append((ang, dist_m, confidence))

    def get_latest_scan(self):
        with self.lock:
            return self.scan_id, (self.latest_scan.copy() if self.latest_scan is not None else None)

    def get_latest_scan_t(self):
        """(id, scan, t): t = instante (time.monotonic) en que se completó la revolución."""
        with self.lock:
            return (self.scan_id, (self.latest_scan.copy() if self.latest_scan is not None else None),
                    self.latest_time)


class LD06Scanner:
    """Lee el LD06 directamente por USB-serie (adaptador del LD06), valida CRC-8
    y entrega revoluciones completas de 360 grados."""

    def __init__(self, port='/dev/ttyUSB0', baudrate=DEFAULT_BAUD, footprint=None):
        self.port = port
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=0.1)
        self.asm = ScanAssembler(footprint)
        self.packets = 0
        self.bad_crc = 0
        self.last_rx = 0.0
        self.error = ""
        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    @staticmethod
    def verify_crc(packet: bytes) -> bool:
        crc = 0
        for b in packet[:46]:
            crc = crc_table[(crc ^ b) & 0xFF]
        return crc == packet[46]

    def _reader_loop(self):
        buf = bytearray()

        while self.running:
            try:
                chunk = self.ser.read(512)
            except (serial.SerialException, OSError) as e:   # cable desconectado, etc.
                self.error = str(e)
                break
            if not chunk:
                continue
            buf.extend(chunk)

            while len(buf) >= 47:
                idx = buf.find(bytes([START_CHARACTER, 0x2C]))
                if idx == -1:
                    buf = buf[-1:]
                    break
                if idx > 0:
                    del buf[:idx]
                if len(buf) < 47:
                    break

                packet = bytes(buf[:47])
                if not self.verify_crc(packet):
                    self.bad_crc += 1
                    del buf[:1]
                    continue

                del buf[:47]
                self.packets += 1
                self.last_rx = time.time()
                self.asm.feed(packet)

    def get_latest_scan(self):
        return self.asm.get_latest_scan()

    def get_latest_scan_t(self):
        return self.asm.get_latest_scan_t()

    def status_text(self):
        if self.error:
            return f"LIDAR     : ERROR de puerto {self.port}: {self.error[:70]}"
        if self.last_rx == 0.0:
            return f"LIDAR     : esperando datos en {self.port} (revisa cable/adaptador USB y que el LD06 gire) ..."
        age = time.time() - self.last_rx
        if age > 2.0:
            return f"LIDAR     : SIN DATOS hace {age:.0f}s en {self.port}"
        propio = (f" | chasis filtrado {self.asm.self_last_rev} pts/vuelta"
                  if self.asm.footprint is not None else " | filtro de chasis OFF")
        return f"LIDAR     : OK {self.port} | paquetes {self.packets} | CRC malos {self.bad_crc}{propio}"

    def close(self):
        self.running = False
        self.thread.join(timeout=0.5)
        self.ser.close()


# =====================================================================
# 1b. IMU MPU9250 POR I2C (RASPBERRY): GIROSCOPIO Z -> PISTA DE ROTACIÓN PARA EL ICP
# =====================================================================
MPU_ADDR_DEFAULT = 0x68           # AD0 a GND = 0x68 | AD0 a 3.3 V = 0x69
MPU_WHO_AM_I_NAMES = {0x71: "MPU9250", 0x73: "MPU9255", 0x70: "MPU6500", 0x68: "MPU6050"}
MPU_GYRO_LSB_PER_DPS = 65.5       # rango +-500 grados/s
IMU_RATE_HZ = 200
IMU_CALIB_S = 2.0


class MPU9250Gyro:
    """Lee el giroscopio Z del MPU9250 por I2C en un hilo y lo integra a un ángulo (yaw).
    Solo usa el giroscopio: el acelerómetro no sirve para posición y el magnetómetro se
    distorsiona con motores y estructura metálica. El SLAM del LiDAR es la referencia absoluta."""

    def __init__(self, bus=1, addr=MPU_ADDR_DEFAULT, rate_hz=IMU_RATE_HZ):
        try:
            from smbus2 import SMBus
        except ImportError:
            raise RuntimeError("falta la libreria smbus2:  ~/larc-env/bin/pip install smbus2")
        self.addr = addr
        self.period = 1.0 / rate_hz
        self.bus = SMBus(bus)
        try:
            who = self.bus.read_byte_data(addr, 0x75)
            if who not in MPU_WHO_AM_I_NAMES:
                raise RuntimeError(f"WHO_AM_I=0x{who:02X} en 0x{addr:02X}: no parece un MPU")
            self.model = MPU_WHO_AM_I_NAMES[who]
            self.bus.write_byte_data(addr, 0x6B, 0x80)      # reset
            time.sleep(0.1)
            self.bus.write_byte_data(addr, 0x6B, 0x01)      # despierta, reloj PLL
            time.sleep(0.05)
            self.bus.write_byte_data(addr, 0x19, 4)         # 1 kHz / (1+4) = 200 Hz
            self.bus.write_byte_data(addr, 0x1A, 0x03)      # filtro pasa-bajos ~41 Hz
            self.bus.write_byte_data(addr, 0x1B, 0x08)      # giroscopio +-500 deg/s
        except Exception:
            self.bus.close()
            raise

        self.lock = threading.Lock()
        self.hist = deque(maxlen=4000)      # (t, yaw): ~20 s de historial a 200 Hz
        self.yaw = 0.0                      # rad, integral acumulada (sin envolver)
        self.bias = 0.0                     # rad/s
        self.bias0 = 0.0
        self.calib_std_dps = 0.0
        self.calib_ok = True
        self.read_errors = 0
        self.error = ""
        self.last_read = 0.0
        self.running = True
        self._calib = []
        self._calib_until = None
        self._calib_try = 0
        self._calibrated = False
        self.start_calibration(IMU_CALIB_S)
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    # ---- calibración del sesgo (robot QUIETO) ----
    def start_calibration(self, seconds=IMU_CALIB_S):
        self._calib = []
        self._calib_try = 0
        self._calib_seconds = seconds
        self._calib_until = time.monotonic() + seconds

    def _finish_calibration(self):
        arr = np.array(self._calib) if self._calib else np.zeros(1)
        std_dps = math.degrees(float(arr.std()))
        if std_dps > 1.0 and self._calib_try < 3:          # el robot se movio: repetir
            self._calib_try += 1
            self._calib = []
            self._calib_until = time.monotonic() + self._calib_seconds
            return
        self.bias = float(arr.mean())
        self.bias0 = self.bias
        self.calib_std_dps = std_dps
        self.calib_ok = std_dps <= 1.0
        self._calibrated = True
        self._calib_until = None

    @property
    def ready(self):
        return self._calibrated and self._calib_until is None

    @property
    def ok(self):
        return time.monotonic() - self.last_read < 0.5

    def wait_ready(self, timeout=10.0):
        t0 = time.monotonic()
        while not self.ready and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return self.ready

    # ---- hilo de lectura ----
    def _loop(self):
        t_prev, rate_prev = None, None
        errs = 0
        next_t = time.monotonic()
        while self.running:
            try:
                d = self.bus.read_i2c_block_data(self.addr, 0x47, 2)     # GYRO_ZOUT_H/L
                errs = 0
            except OSError as e:
                errs += 1
                self.read_errors += 1
                self.error = f"I2C: {e}"
                time.sleep(0.05)
                continue
            t = time.monotonic()
            raw = (d[0] << 8) | d[1]
            if raw >= 0x8000:
                raw -= 0x10000
            rate = math.radians(raw / MPU_GYRO_LSB_PER_DPS)               # rad/s crudos
            self.last_read = t
            self.error = ""

            if self._calib_until is not None:       # calibrando: yaw congelado
                self._calib.append(rate)
                if t >= self._calib_until:
                    self._finish_calibration()
            elif rate_prev is not None:
                self.yaw += 0.5 * ((rate_prev - self.bias) + (rate - self.bias)) * (t - t_prev)
            with self.lock:
                self.hist.append((t, self.yaw))
            rate_prev, t_prev = rate, t

            next_t += self.period
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    def yaw_at(self, t):
        """Yaw (rad, crudo y sin envolver) interpolado en el instante t. None si no hay datos."""
        with self.lock:
            if not self.hist:
                return None
            t_last, y_last = self.hist[-1]
            if t >= t_last:
                return y_last if t - t_last < 0.3 else None
            nxt = (t_last, y_last)
            for tt, yy in reversed(self.hist):
                if tt <= t:
                    if nxt[0] == tt:
                        return yy
                    a = (t - tt) / (nxt[0] - tt)
                    return yy + a * (nxt[1] - yy)
                nxt = (tt, yy)
            return None

    def nudge_bias(self, err_rate, gain=0.02, max_dev=math.radians(1.5)):
        """Corrige despacio el sesgo con el error (rad/s) frente al giro medido por el ICP."""
        self.bias = min(max(self.bias + gain * err_rate, self.bias0 - max_dev), self.bias0 + max_dev)

    def close(self):
        self.running = False
        self.thread.join(timeout=0.5)
        try:
            self.bus.close()
        except Exception:
            pass


class ImuFusion:
    """Convierte el giroscopio en una pista de rotación (dtheta) para el ICP.
    Mientras el signo no está verificado, el SLAM prueba también la hipótesis opuesta (giroscopio
    montado al revés) y vota según cuál coincide con el giro que mide el ICP; al cabo de unos
    giros fija el signo correcto solo. Ya verificado, corrige el sesgo en marcha."""

    VOTES_MIN = 5          # votos decisivos para fijar el signo
    VOTES_GIVE_UP = 14     # si ni así hay mayoría clara: giroscopio incoherente, se desactiva

    def __init__(self, gyro, sign=1):
        self.gyro = gyro
        self.sign = 1 if sign >= 0 else -1
        self.enabled = True
        self.verified = False
        self.verdict = "pendiente (gira el robot)"
        self.reset()

    def reset(self):
        self.last_yaw = None
        self.last_t = None
        self.raw_dth = None
        self.dt = 0.0
        self.votes_main = 0
        self.votes_alt = 0

    def pre_step(self, scan_t):
        """(dtheta, dtheta_alternativo): rotación esperada desde el barrido anterior en el convenio
        del SLAM. El alternativo (signo opuesto) solo se da mientras el signo no está verificado."""
        self.raw_dth = None
        if not self.enabled or not self.gyro.ready or not self.gyro.ok:
            self.last_yaw = None
            return None, None
        yaw = self.gyro.yaw_at(scan_t)
        if yaw is None:
            self.last_yaw = None
            return None, None
        prev_yaw, prev_t = self.last_yaw, self.last_t
        self.last_yaw, self.last_t = yaw, scan_t
        if prev_yaw is None:
            return None, None
        dt = scan_t - prev_t
        if dt <= 0.0 or dt > 0.5:
            return None, None
        self.dt = dt
        self.raw_dth = yaw - prev_yaw
        pred = self.sign * self.raw_dth
        alt = -pred if (not self.verified and abs(pred) > math.radians(1.0)) else None
        return pred, alt

    def post_step(self, slam):
        """Contrasta con el giro que midió el ICP (solo con buen alineamiento)."""
        icp = slam.step_dtheta
        if self.raw_dth is None or icp is None or slam.quality < 0.6 or slam.scan_count < 10:
            return
        pred = self.sign * self.raw_dth

        if not self.verified:
            if abs(pred) < math.radians(1.0):
                return                                           # sin giro no hay información
            tol = math.radians(1.0)
            e_main = abs((icp - pred + math.pi) % (2 * math.pi) - math.pi)
            e_alt = abs((icp + pred + math.pi) % (2 * math.pi) - math.pi)
            if e_alt + tol < e_main:
                self.votes_alt += 1
            elif e_main + tol < e_alt:
                self.votes_main += 1
            n = self.votes_main + self.votes_alt
            if n >= self.VOTES_MIN:
                if self.votes_alt / n >= 0.8:
                    self.sign = -self.sign                       # montado al reves: se corrige solo
                    self.verified = True
                    self.verdict = "signo INVERTIDO -> corregido automaticamente"
                elif self.votes_main / n >= 0.8:
                    self.verified = True
                    self.verdict = "verificado"
                elif n >= self.VOTES_GIVE_UP:
                    self.enabled = False
                    self.verdict = "INCOHERENTE: giroscopio desactivado (revisa montaje/cableado)"
            return

        # Verificado: el ICP es la referencia absoluta de rumbo -> corrige el sesgo despacio
        if abs(icp) < math.radians(20) and self.dt > 0:
            self.gyro.nudge_bias((self.raw_dth - self.sign * icp) / self.dt)

    def status_text(self):
        g = self.gyro
        if not g.ready:
            return "GYRO      : calibrando sesgo... NO MOVER el robot"
        if g.error and not g.ok:
            return f"GYRO      : ERROR {g.error[:80]}"
        warn = "" if g.calib_ok else " | CALIBRACION CON MOVIMIENTO (pulsa B con el robot quieto)"
        return (f"GYRO      : {'ON' if self.enabled else 'OFF'} {g.model} 0x{g.addr:02X} | "
                f"yaw={math.degrees(self.sign * g.yaw):+.1f} deg | sesgo={math.degrees(g.bias):+.3f} deg/s | "
                f"signo {'+' if self.sign > 0 else '-'}: {self.verdict}{warn}")


# =====================================================================
# 2. SLAM 2D (SCAN-TO-MAP TRIMMED ICP + LOG-ODDS GRID + MAPA ESTÁTICO)
# =====================================================================
class LidarSLAM2D:
    def __init__(self, map_size_m=20.0, resolution_m=0.04):
        self.res = resolution_m
        self.grid_dim = int(map_size_m / resolution_m)
        self.center_cell = self.grid_dim // 2

        self.log_odds = np.zeros(
            (self.grid_dim, self.grid_dim), dtype=np.float32)
        # Contador de impactos por celda: más muestras = celda más confiable
        self.hits = np.zeros((self.grid_dim, self.grid_dim), dtype=np.uint16)
        # Suma de posiciones de impactos por celda -> posición media sub-celda
        # (permite ajustar formas con más precisión que el tamaño de celda)
        self.sum_x = np.zeros((self.grid_dim, self.grid_dim), dtype=np.float64)
        self.sum_y = np.zeros((self.grid_dim, self.grid_dim), dtype=np.float64)
        self.pt_cnt = np.zeros((self.grid_dim, self.grid_dim), dtype=np.float32)

        # [x_m, y_m, theta_rad]
        self.pose = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.pose_delta = np.zeros(3, dtype=np.float64)
        self.step_dtheta = None      # rotación (rad) que aplicó el último ICP
        self.icp_rejected = 0        # veces que se descartó un ICP malo usando el giroscopio
        self.fallback_quality = 0.20 # con giroscopio, bajo esta calidad se descarta el ICP
        self.trajectory = deque(maxlen=600)
        self.initialized = False
        self.map_locked = False
        self.hold_integration = False   # True mientras el brazo trabaja: el mapa no integra barridos
        # Huella del robot (RobotFootprint). Si se asigna, las celdas bajo el chasis cuentan como libres:
        # el robot está ahí, no puede haber obstáculo, y sin esto queda un hueco "desconocido" alrededor
        # del robot (los rayos de espacio libre empiezan al 15 % del alcance) que desconecta el origen
        # del resto del mapa para el planificador.
        self.footprint = None
        self._fp_local = None

        # Parámetros del mapa estático
        self.static_logodds_thr = 1.2
        self.min_static_hits = 3

        # Calidad / muestras
        self.scan_count = 0          # barridos integrados al mapa
        self.quality = 1.0           # fracción de puntos bien alineados (ICP)
        self.min_quality = 0.30      # bajo esto, el barrido NO se integra al mapa
        self.warmup_scans = 10
        self._ref_cache = None       # (puntos, kdtree) cuando el mapa está congelado

    # ---------- utilidades geométricas ----------
    def polar_to_local_cartesian(self, scan):
        angles = scan[:, 0]
        dists = scan[:, 1]
        x = np.sin(angles) * dists
        y = np.cos(angles) * dists
        return np.column_stack((x, y))

    @staticmethod
    def transform_points(pts_local, pose):
        x_r, y_r, theta = pose
        c, s = np.cos(theta), np.sin(theta)
        R = np.array([[c, -s], [s, c]])
        return pts_local @ R.T + np.array([x_r, y_r])

    def world_to_grid(self, pts_world):
        gx = np.floor(pts_world[:, 0] / self.res +
                      self.center_cell).astype(np.int32)
        gy = np.floor(pts_world[:, 1] / self.res +
                      self.center_cell).astype(np.int32)
        valid = (gx >= 0) & (gx < self.grid_dim) & (
            gy >= 0) & (gy < self.grid_dim)
        return gx, gy, valid

    @staticmethod
    def _dynamic_mask(pts, zones):
        """True para los puntos que caen dentro de zonas de objetos en movimiento."""
        mask = np.zeros(len(pts), dtype=bool)
        for center, radius in zones:
            mask |= np.linalg.norm(pts - center, axis=1) < radius
        return mask

    # ---------- mapa ----------
    def get_static_map_points(self):
        """Puntos de referencia para ICP (celdas ocupadas)."""
        occ_y, occ_x = np.where(self.log_odds > 0.8)
        if len(occ_x) == 0:
            return np.empty((0, 2))
        wx = (occ_x - self.center_cell + 0.5) * self.res
        wy = (occ_y - self.center_cell + 0.5) * self.res
        return np.column_stack((wx, wy))

    def static_mask(self):
        """Máscara booleana del MAPA ESTÁTICO: ocupación confirmada con suficientes muestras."""
        return (self.log_odds > self.static_logodds_thr) & (self.hits >= self.min_static_hits)

    def near_static(self, p, mask, r_cells=2):
        """True si el punto p cae sobre (o a ~8 cm de) una celda del mapa estático."""
        gx = int(np.floor(p[0] / self.res + self.center_cell))
        gy = int(np.floor(p[1] / self.res + self.center_cell))
        if gx < 0 or gy < 0 or gx >= self.grid_dim or gy >= self.grid_dim:
            return False
        y0, y1 = max(gy - r_cells, 0), min(gy + r_cells + 1, self.grid_dim)
        x0, x1 = max(gx - r_cells, 0), min(gx + r_cells + 1, self.grid_dim)
        return bool(mask[y0:y1, x0:x1].any())

    def toggle_lock(self):
        self.map_locked = not self.map_locked
        self._ref_cache = None

    def save(self, path="mapa_estatico.npz"):
        np.savez_compressed(path, log_odds=self.log_odds, hits=self.hits,
                            sum_x=self.sum_x, sum_y=self.sum_y, pt_cnt=self.pt_cnt,
                            res=self.res, grid_dim=self.grid_dim)

    def load(self, path="mapa_estatico.npz"):
        """Carga un mapa guardado. El robot debe arrancar en el mismo origen/orientación."""
        try:
            data = np.load(path)
            if int(data["grid_dim"]) != self.grid_dim or abs(float(data["res"]) - self.res) > 1e-9:
                return False
            self.log_odds = data["log_odds"].astype(np.float32)
            self.hits = data["hits"].astype(np.uint16)
            if "sum_x" in data.files:   # mapas guardados antes no traen estos campos
                self.sum_x = data["sum_x"].astype(np.float64)
                self.sum_y = data["sum_y"].astype(np.float64)
                self.pt_cnt = data["pt_cnt"].astype(np.float32)
            else:
                self.sum_x = np.zeros_like(self.sum_x)
                self.sum_y = np.zeros_like(self.sum_y)
                self.pt_cnt = np.zeros_like(self.pt_cnt)
        except (OSError, KeyError, ValueError):
            return False
        self.pose = np.zeros(3)
        self.pose_delta = np.zeros(3)
        self.trajectory.clear()
        self.initialized = True
        self.map_locked = True
        self._ref_cache = None
        return True

    # ---------- scan matching ----------
    def _trimmed_icp(self, pts_local, ref_points, init_pose, tree=None, max_iters=18):
        """Devuelve (pose, calidad). Calidad = fracción de puntos a < 10 cm del mapa."""
        if len(ref_points) < 30 or len(pts_local) < 30:
            return init_pose, 1.0

        if tree is None:
            tree = cKDTree(ref_points)
        pose = init_pose.copy()

        for _ in range(max_iters):
            pts_world = self.transform_points(pts_local, pose)
            dists, indices = tree.query(pts_world, k=1)

            valid_mask = dists < 0.35
            if np.sum(valid_mask) < 20:
                break

            valid_dists = dists[valid_mask]
            cutoff = np.percentile(valid_dists, 80)
            inlier_mask = valid_mask & (dists <= cutoff)

            if np.sum(inlier_mask) < 15:
                break

            src = pts_world[inlier_mask]
            dst = ref_points[indices[inlier_mask]]

            mu_src = src.mean(axis=0)
            mu_dst = dst.mean(axis=0)
            H = (src - mu_src).T @ (dst - mu_dst)
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            if np.linalg.det(R) < 0:
                Vt[1, :] *= -1
                R = Vt.T @ U.T

            d_theta = np.arctan2(R[1, 0], R[0, 0])
            d_trans = mu_dst - (R @ mu_src)

            c, s = np.cos(d_theta), np.sin(d_theta)
            R_step = np.array([[c, -s], [s, c]])
            new_xy = R_step @ pose[:2] + d_trans
            pose[0], pose[1] = new_xy[0], new_xy[1]
            pose[2] = (pose[2] + d_theta + np.pi) % (2 * np.pi) - np.pi

            if np.linalg.norm(d_trans) < 1e-4 and abs(d_theta) < 1e-4:
                break

        final_d, _ = tree.query(self.transform_points(pts_local, pose), k=1)
        quality = float(np.mean(final_d < 0.10))
        return pose, quality

    def _update_occupancy_grid(self, pts_world):
        if self.map_locked or self.hold_integration or len(pts_world) == 0:
            return

        rx, ry = self.pose[0], self.pose[1]
        ratios = np.linspace(0.15, 0.92, 12)[:, None]
        ray_x = rx + ratios * (pts_world[:, 0] - rx)
        ray_y = ry + ratios * (pts_world[:, 1] - ry)
        free_pts = np.column_stack((ray_x.ravel(), ray_y.ravel()))

        fgx, fgy, fvalid = self.world_to_grid(free_pts)
        self.log_odds[fgy[fvalid], fgx[fvalid]] -= 0.12

        if self.footprint is not None:
            if self._fp_local is None:
                st = self.res * 0.5
                gx_, gy_ = np.meshgrid(np.arange(-self.footprint.hx, self.footprint.hx + st, st),
                                       np.arange(-self.footprint.hy, self.footprint.hy + st, st))
                self._fp_local = np.column_stack((gx_.ravel(), gy_.ravel()))
            bx, by, bvalid = self.world_to_grid(self.transform_points(self._fp_local, self.pose))
            celdas = np.unique(np.column_stack((by[bvalid], bx[bvalid])), axis=0)
            self.log_odds[celdas[:, 0], celdas[:, 1]] -= 0.12

        hgx, hgy, hvalid = self.world_to_grid(pts_world)
        self.log_odds[hgy[hvalid], hgx[hvalid]] += 0.65

        cur = self.hits[hgy[hvalid], hgx[hvalid]]
        self.hits[hgy[hvalid], hgx[hvalid]] = np.minimum(cur, 60000) + 1
        idx = (hgy[hvalid], hgx[hvalid])
        np.add.at(self.sum_x, idx, pts_world[hvalid, 0])
        np.add.at(self.sum_y, idx, pts_world[hvalid, 1])
        np.add.at(self.pt_cnt, idx, 1.0)

        np.clip(self.log_odds, -4.0, 4.5, out=self.log_odds)

    def step(self, scan, dynamic_zones=(), gyro_dtheta=None, gyro_alt_dtheta=None, gyro_trusted=False):
        """dynamic_zones: lista de (centro_xy, radio) de objetos en movimiento a excluir del mapa.
        gyro_dtheta: rotación (rad, convenio del SLAM) medida por el giroscopio desde el barrido anterior.
        gyro_alt_dtheta: hipótesis opuesta (giroscopio montado al revés); solo mientras no está verificado.
        gyro_trusted: el signo ya está verificado (permite descartar ICP inservibles)."""
        pts_local = self.polar_to_local_cartesian(scan)

        if not self.initialized:
            pts_world = self.transform_points(pts_local, self.pose)
            self._update_occupancy_grid(pts_world)
            gx, gy, valid = self.world_to_grid(pts_world)
            self.log_odds[gy[valid], gx[valid]] = 2.0
            self.hits[gy[valid], gx[valid]] = self.min_static_hits
            self.initialized = True
            self.scan_count = 1
            self.step_dtheta = None
            self.trajectory.append(self.pose[:2].copy())
            return pts_world, np.empty((0, 2))

        pred_pose = self.pose + self.pose_delta * 0.5
        if gyro_dtheta is not None:
            # La rotación la da el giroscopio (mucho mejor que extrapolar la anterior);
            # la traslación sigue extrapolándose y la refina el ICP.
            pred_pose[2] = (self.pose[2] + gyro_dtheta + np.pi) % (2 * np.pi) - np.pi

        # Referencia ICP (con caché cuando el mapa está congelado)
        if self.map_locked and self._ref_cache is not None:
            ref_map_pts, tree = self._ref_cache
        else:
            ref_map_pts = self.get_static_map_points()
            tree = cKDTree(ref_map_pts) if len(ref_map_pts) >= 30 else None
            if self.map_locked and tree is not None:
                self._ref_cache = (ref_map_pts, tree)

        # Excluir objetos dinámicos del scan matching
        if dynamic_zones:
            pw_pred = self.transform_points(pts_local, pred_pose)
            pts_icp = pts_local[~self._dynamic_mask(pw_pred, dynamic_zones)]
        else:
            pts_icp = pts_local

        new_pose, quality = self._trimmed_icp(
            pts_icp, ref_map_pts, pred_pose, tree=tree)
        if gyro_alt_dtheta is not None and quality < 0.6:
            # Signo sin verificar y alineamiento malo: probar la hipótesis opuesta y quedarse con la mejor
            alt_pose = pred_pose.copy()
            alt_pose[2] = (self.pose[2] + gyro_alt_dtheta + np.pi) % (2 * np.pi) - np.pi
            pose_b, q_b = self._trimmed_icp(pts_icp, ref_map_pts, alt_pose, tree=tree)
            if q_b > quality + 0.10:
                new_pose, quality = pose_b, q_b
        if gyro_trusted and gyro_dtheta is not None and quality < self.fallback_quality:
            new_pose = pred_pose.copy()      # ICP inservible: mejor la predicción con giroscopio
            self.icp_rejected += 1
        self.quality = quality
        self.pose_delta = new_pose - self.pose
        self.pose_delta[2] = (self.pose_delta[2] + np.pi) % (2 * np.pi) - np.pi
        self.step_dtheta = float(self.pose_delta[2])
        self.pose = new_pose

        if len(self.trajectory) == 0 or np.linalg.norm(self.pose[:2] - self.trajectory[-1]) > 0.03:
            self.trajectory.append(self.pose[:2].copy())

        pts_world = self.transform_points(pts_local, self.pose)
        gx, gy, valid = self.world_to_grid(pts_world)

        is_candidate = np.ones(len(pts_world), dtype=bool)
        is_candidate[valid] = self.log_odds[gy[valid], gx[valid]] < 2.2
        candidate_pts = pts_world[is_candidate]

        # Integración al mapa: solo puntos estáticos y solo si el alineamiento es bueno.
        # Cada barrido válido suma una muestra más a las celdas -> menor error acumulado.
        if quality >= self.min_quality or self.scan_count < self.warmup_scans:
            if dynamic_zones:
                static_pts = pts_world[~self._dynamic_mask(
                    pts_world, dynamic_zones)]
            else:
                static_pts = pts_world
            if not self.map_locked:
                self._update_occupancy_grid(static_pts)
                self.scan_count += 1

        return pts_world, candidate_pts


# =====================================================================
# 3. PLANIFICADOR POR GRAFOS (A* SOBRE EL MAPA ESTÁTICO)
# =====================================================================
class GraphPlanner:
    """
    Grafo: cada celda libre (a resolución de planificación) es un nodo con 8 vecinos.
    Peso de arista = distancia * (1 + penalización por cercanía a obstáculos).
    A* encuentra el camino de menor peso total; luego se suaviza con line-of-sight.
    """

    def __init__(self, slam, plan_res=0.08, robot_radius=0.18, safety=0.30, snap_max=0.8):
        self.slam = slam
        self.f = max(1, int(round(plan_res / slam.res)))
        self.res = self.f * slam.res
        self.robot_radius = robot_radius
        self.safety = safety
        self.snap_max = snap_max

        self.allowed = None
        self.cost = None
        self._snap_idx = None
        self._snap_dist = None

        self.path = None        # ruta suavizada (waypoints)
        self.path_raw = None    # ruta completa del grafo
        self.raw_cum = None
        self.length = 0.0
        self.expanded = 0
        self.n_nodes = 0
        self.msg = "Sin ruta"

    def clear(self):
        self.path = None
        self.path_raw = None
        self.raw_cum = None
        self.length = 0.0
        self.expanded = 0
        self.msg = "Sin ruta"

    # ---------- conversión de coordenadas ----------
    def _world_to_cell(self, p):
        s = self.slam
        fx = int(np.floor(p[0] / s.res + s.center_cell))
        fy = int(np.floor(p[1] / s.res + s.center_cell))
        py, px = fy // self.f, fx // self.f
        if fx < 0 or fy < 0 or py >= self.allowed.shape[0] or px >= self.allowed.shape[1]:
            return None
        return py, px

    def _cell_to_world(self, py, px):
        s = self.slam
        x = (px * self.f + self.f / 2.0 - s.center_cell) * s.res
        y = (py * self.f + self.f / 2.0 - s.center_cell) * s.res
        return np.array([x, y])

    # ---------- construcción del grafo ----------
    def build_costmap(self):
        s, f = self.slam, self.f
        n = (s.grid_dim // f) * f
        m = n // f
        lo = s.log_odds[:n, :n]
        static = s.static_mask()[:n, :n]

        occ = static.reshape(m, f, m, f).any(axis=(1, 3))
        free_ratio = (lo < -0.25).reshape(m, f, m, f).mean(axis=(1, 3))
        base_free = (free_ratio >= 0.5) & ~occ

        if occ.any():
            dist = distance_transform_edt(~occ) * self.res
        else:
            dist = np.full(occ.shape, 99.0)

        self.allowed = base_free & (dist >= self.robot_radius)
        pen = np.clip((self.robot_radius + self.safety - dist) / self.safety, 0.0, 1.0)
        self.cost = 1.0 + 6.0 * pen
        self.n_nodes = int(self.allowed.sum())

        if self.allowed.any():
            d, idx = distance_transform_edt(~self.allowed, return_indices=True)
            self._snap_dist = d * self.res
            self._snap_idx = idx

    def _snap(self, cell):
        if cell is None:
            return None
        if self.allowed[cell]:
            return cell
        if self._snap_idx is None or self._snap_dist[cell] > self.snap_max:
            return None
        return int(self._snap_idx[0][cell]), int(self._snap_idx[1][cell])

    # ---------- A* ----------
    def _astar(self, start, goal):
        allowed = self.allowed.tolist()
        cost = self.cost.tolist()
        H, W = self.allowed.shape
        res = self.res
        r2 = np.sqrt(2.0)
        moves = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                 (-1, -1, r2), (-1, 1, r2), (1, -1, r2), (1, 1, r2)]

        def heur(y, x):
            dy, dx = abs(goal[0] - y), abs(goal[1] - x)
            return res * (max(dy, dx) + (r2 - 1.0) * min(dy, dx))

        g = {start: 0.0}
        parent = {}
        closed = set()
        heap = [(heur(*start), 0.0, start)]
        expanded = 0

        while heap:
            _, gc, node = heapq.heappop(heap)
            if node in closed:
                continue
            closed.add(node)
            expanded += 1
            if node == goal:
                cells = [node]
                while node in parent:
                    node = parent[node]
                    cells.append(node)
                return cells[::-1], gc, expanded
            y, x = node
            cy = cost[y][x]
            for dy, dx, step in moves:
                ny, nx = y + dy, x + dx
                if ny < 0 or nx < 0 or ny >= H or nx >= W or not allowed[ny][nx]:
                    continue
                if dy != 0 and dx != 0 and not (allowed[y + dy][x] and allowed[y][x + dx]):
                    continue  # no cortar esquinas
                ng = gc + step * res * 0.5 * (cy + cost[ny][nx])
                if ng < g.get((ny, nx), np.inf):
                    g[(ny, nx)] = ng
                    parent[(ny, nx)] = node
                    heapq.heappush(heap, (ng + heur(ny, nx), ng, (ny, nx)))
        return None, np.inf, expanded

    def _line_free(self, a, b):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 2
        ys = np.rint(np.linspace(a[0], b[0], n)).astype(int)
        xs = np.rint(np.linspace(a[1], b[1], n)).astype(int)
        return bool(self.allowed[ys, xs].all())

    def _shortcut(self, cells):
        out = [cells[0]]
        i = 0
        while i < len(cells) - 1:
            j = len(cells) - 1
            while j > i + 1 and not self._line_free(cells[i], cells[j]):
                j -= 1
            out.append(cells[j])
            i = j
        return out

    # ---------- API ----------
    def plan(self, start_xy, goal_xy):
        self.clear()
        self.build_costmap()
        if not self.allowed.any():
            self.msg = "Mapa sin zonas libres suficientes"
            return False

        s = self._snap(self._world_to_cell(start_xy))
        g = self._snap(self._world_to_cell(goal_xy))
        if s is None:
            self.msg = "Origen A fuera de zona libre"
            return False
        if g is None:
            self.msg = "Destino B inalcanzable / zona desconocida"
            return False

        cells, total_cost, expanded = self._astar(s, g)
        self.expanded = expanded
        if cells is None:
            self.msg = "No existe ruta entre A y B"
            return False

        self.path_raw = np.array([self._cell_to_world(*c) for c in cells])
        self.path = np.array([self._cell_to_world(*c)
                             for c in self._shortcut(cells)])
        seg = np.linalg.norm(np.diff(self.path_raw, axis=0), axis=1)
        self.raw_cum = np.concatenate(([0.0], np.cumsum(seg)))
        self.length = float(np.linalg.norm(
            np.diff(self.path, axis=0), axis=1).sum())
        self.msg = f"OK (peso {total_cost:.2f})"
        return True

    def approach_point(self, center, radius, from_xy, standoff=0.25):
        """Punto libre a 'standoff' del borde de un objeto, del lado más cercano a from_xy."""
        self.build_costmap()
        d = radius + self.robot_radius + standoff
        best, best_dist = None, np.inf
        for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
            p = np.asarray(center) + d * np.array([np.cos(ang), np.sin(ang)])
            cell = self._world_to_cell(p)
            if cell is None or not self.allowed[cell]:
                continue
            dist = float(np.linalg.norm(p - np.asarray(from_xy)))
            if dist < best_dist:
                best, best_dist = p, dist
        return best

    def remaining(self, robot_xy):
        """Distancia restante a lo largo de la ruta desde la posición actual."""
        if self.path_raw is None or len(self.path_raw) == 0:
            return None
        d = np.linalg.norm(self.path_raw - robot_xy, axis=1)
        i = int(np.argmin(d))
        return float(self.raw_cum[-1] - self.raw_cum[i] + d[i])


# =====================================================================
# 3b. LANDMARKS: CLASIFICACIÓN POR FORMA Y REGISTRO CON NOMBRES ESTABLES
# =====================================================================
TREE_R_MIN = 0.03        # radio mínimo de un tronco (m)   <-- AJUSTAR A TUS ÁRBOLES
TREE_R_MAX = 0.25        # radio máximo de un tronco (m)   <-- AJUSTAR A TUS ÁRBOLES
WALL_MIN_LEN = 1.0       # clusters más largos que esto se consideran pared/borde
CIRCLE_RMS_MIN = 0.010   # tolerancia mínima del ajuste de círculo (m); subir si tus árboles salen "OBJ"
CIRCLE_RMS_FRAC = 0.05   # tolerancia relativa al radio
MIN_ARC_DEG = 90.0       # arco mínimo visible para aceptar un cilindro
SHOW_STATIC_TRACKS = False   # dibujar también los tracks estáticos del tracker
CLASS_PREFIX = {"ARBOL": "ARBOL", "OBSTACULO": "OBJ"}


def fit_circle(pts):
    """Ajuste algebraico de círculo (Kasa). Devuelve (centro, radio, rms, arco_deg) o None."""
    if len(pts) < 4:
        return None
    mu = pts.mean(axis=0)
    q = pts - mu
    A = np.column_stack((2 * q[:, 0], 2 * q[:, 1], np.ones(len(q))))
    b = (q ** 2).sum(axis=1)
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy, c = sol
    r2 = c + cx * cx + cy * cy
    if r2 <= 0:
        return None
    r = float(np.sqrt(r2))
    center = np.array([cx, cy]) + mu
    d = np.linalg.norm(pts - center, axis=1)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    ang = np.sort(np.arctan2(pts[:, 1] - center[1], pts[:, 0] - center[0]))
    gaps = np.diff(np.concatenate((ang, [ang[0] + 2 * np.pi])))
    arc_deg = float(np.degrees(2 * np.pi - gaps.max()))
    return center, r, rms, arc_deg


def classify_cluster(pts):
    """pts: (N,2) celdas estáticas de un cluster. Devuelve rasgos + clase (PARED/ARBOL/OBSTACULO)."""
    center = pts.mean(axis=0)
    q = pts - center
    _, _, Vt = np.linalg.svd(q, full_matrices=False)
    proj = q @ Vt.T
    length = float(np.ptp(proj[:, 0]))
    width = float(np.ptp(proj[:, 1])) if proj.shape[1] > 1 else 0.0
    info = {"center": center, "radius": max(length, width) / 2.0, "length": length,
            "width": width, "n": len(pts), "rms": None, "arc": None, "cls": "OBSTACULO"}

    if length >= WALL_MIN_LEN:
        info["cls"] = "PARED"
        return info

    fit = fit_circle(pts)
    if fit is not None:
        c, r, rms, arc = fit
        info["rms"], info["arc"] = rms, arc
        tol = max(CIRCLE_RMS_MIN, CIRCLE_RMS_FRAC * r)
        if (TREE_R_MIN <= r <= TREE_R_MAX and rms <= tol and arc >= MIN_ARC_DEG
                and np.linalg.norm(c - center) <= 2 * TREE_R_MAX):
            info.update(cls="ARBOL", center=c, radius=r)
    return info


def detect_static_clusters(slam, eps=0.09, min_samples=3):
    """Agrupa las celdas del mapa estático y clasifica cada grupo por su forma."""
    ys, xs = np.nonzero(slam.static_mask())
    if len(xs) < min_samples:
        return []
    # Posición media de los impactos de cada celda (sub-celda); si no hay, centro de celda
    cnt = slam.pt_cnt[ys, xs]
    has = cnt > 0
    safe = np.maximum(cnt, 1.0)
    px = np.where(has, slam.sum_x[ys, xs] / safe, (xs - slam.center_cell + 0.5) * slam.res)
    py = np.where(has, slam.sum_y[ys, xs] / safe, (ys - slam.center_cell + 0.5) * slam.res)
    pts = np.column_stack((px, py))
    labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(pts)
    return [classify_cluster(pts[labels == lbl]) for lbl in set(labels) if lbl != -1]


class Landmark:
    def __init__(self, center, radius, cls):
        self.pos = np.array(center, dtype=np.float64)
        self.radius = float(radius)
        self.cls = None          # clase confirmada (None = aún sin confirmar)
        self.name = None
        self.cand_cls = cls      # clase observada recientemente
        self.streak = 1          # observaciones consecutivas con esa clase
        self.obs = 1
        self.missed = 0

    @property
    def confirmed(self):
        return self.name is not None


class LandmarkRegistry:
    """Mantiene landmarks con nombre estable (ARBOL_1, OBJ_2...) asociados por POSICIÓN en el mapa."""

    def __init__(self, assoc_dist=0.35, confirm_n=3, max_missed=30):
        self.items = []
        self.counters = {}
        self.assoc_dist = assoc_dist
        self.confirm_n = confirm_n
        self.max_missed = max_missed

    def _assign_name(self, lm, cls):
        n = self.counters.get(cls, 0) + 1
        self.counters[cls] = n
        lm.cls = cls
        lm.name = f"{CLASS_PREFIX[cls]}_{n}"

    def _observe(self, lm, c):
        lm.missed = 0
        lm.obs += 1
        a = 0.3
        lm.pos = (1 - a) * lm.pos + a * np.asarray(c["center"])
        lm.radius = (1 - a) * lm.radius + a * c["radius"]
        if c["cls"] == lm.cand_cls:
            lm.streak += 1
        else:
            lm.cand_cls, lm.streak = c["cls"], 1
        if lm.streak >= self.confirm_n and lm.cand_cls != lm.cls:
            self._assign_name(lm, lm.cand_cls)

    def update(self, clusters, allow_expire=True):
        clusters = [c for c in clusters if c["cls"] != "PARED"]
        matched_l, matched_c = set(), set()
        if self.items and clusters:
            lm_pos = np.array([l.pos for l in self.items])
            cl_pos = np.array([c["center"] for c in clusters])
            cost = np.linalg.norm(lm_pos[:, None, :] - cl_pos[None, :, :], axis=2)
            rows, cols = linear_sum_assignment(cost)
            for r, c in zip(rows, cols):
                if cost[r, c] <= self.assoc_dist:
                    self._observe(self.items[r], clusters[c])
                    matched_l.add(r)
                    matched_c.add(c)
        for i, lm in enumerate(self.items):
            if i not in matched_l:
                lm.missed += 1
        for j, c in enumerate(clusters):
            if j not in matched_c:
                self.items.append(Landmark(c["center"], c["radius"], c["cls"]))
        if allow_expire:
            self.items = [l for l in self.items
                          if l.missed <= (self.max_missed if l.confirmed else 3)]

    def confirmed_items(self):
        conf = [l for l in self.items if l.confirmed]
        return sorted(conf, key=lambda l: (l.cls, int(l.name.rsplit("_", 1)[1])))

    def get(self, name):
        for l in self.items:
            if l.name is not None and l.name == name:
                return l
        return None

    def save(self, path="landmarks.json"):
        data = {"counters": self.counters,
                "items": [{"name": l.name, "cls": l.cls,
                           "pos": [float(l.pos[0]), float(l.pos[1])],
                           "radius": float(l.radius)}
                          for l in self.items if l.confirmed]}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

    def load(self, path="landmarks.json"):
        try:
            with open(path) as f:
                data = json.load(f)
            items = []
            for d in data.get("items", []):
                lm = Landmark(d["pos"], d["radius"], d["cls"])
                lm.cls, lm.name, lm.streak = d["cls"], d["name"], self.confirm_n
                items.append(lm)
            self.counters = {k: int(v) for k, v in data.get("counters", {}).items()}
        except (OSError, ValueError, KeyError, TypeError):
            return False
        self.items = items
        return True


# =====================================================================
# 4. FILTRO DE KALMAN Y TRACKER MULTI-OBJETO (DBSCAN + HÚNGARO)
# =====================================================================
class KalmanTrack:
    _next_id = 1

    def __init__(self, centroid, radius, dt=0.1):
        self.id = KalmanTrack._next_id
        KalmanTrack._next_id += 1
        self.dt = dt
        self.x = np.array([[centroid[0]], [centroid[1]], [
                          0.0], [0.0]], dtype=np.float64)
        self.P = np.diag([0.05, 0.05, 0.8, 0.8])
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1,  0],
            [0, 0, 0,  1]
        ], dtype=np.float64)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        self.Q = np.diag([0.01, 0.01, 0.15, 0.15])
        self.R = np.eye(2) * 0.015

        self.radius = radius
        self.hits = 1
        self.missed = 0
        self.history = deque([centroid.copy()], maxlen=35)

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2].ravel()

    def update(self, centroid, radius):
        z = np.reshape(centroid, (2, 1))
        y = z - (self.H @ self.x)
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ y)
        self.P = (np.eye(4) - K @ self.H) @ self.P

        self.radius = 0.7 * self.radius + 0.3 * radius
        self.hits += 1
        self.missed = 0
        self.history.append(self.x[:2].ravel().copy())


class MultiObjectTracker:
    def __init__(self, eps=0.16, min_samples=3, max_match_dist=0.55, max_missed=6):
        self.dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        self.tracks = []
        self.max_match_dist = max_match_dist
        self.max_missed = max_missed

    def update(self, all_world_pts, dynamic_candidates):
        pts_to_cluster = all_world_pts if len(
            all_world_pts) > 0 else dynamic_candidates
        detections = []

        if len(pts_to_cluster) >= self.dbscan.min_samples:
            labels = self.dbscan.fit_predict(pts_to_cluster)
            for lbl in set(labels):
                if lbl == -1:
                    continue
                cluster = pts_to_cluster[labels == lbl]
                span = np.linalg.norm(cluster.max(
                    axis=0) - cluster.min(axis=0))
                if 0.03 <= span <= 0.85:
                    center = cluster.mean(axis=0)
                    radius = max(span / 2.0, 0.08)
                    detections.append((center, radius))

        preds = np.array([t.predict() for t in self.tracks]
                         ) if self.tracks else np.empty((0, 2))
        det_centers = np.array([d[0] for d in detections]
                               ) if detections else np.empty((0, 2))

        if len(self.tracks) > 0 and len(detections) > 0:
            cost_matrix = np.linalg.norm(
                preds[:, None, :] - det_centers[None, :, :], axis=2)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            matched_tracks = set()
            matched_dets = set()

            for r, c in zip(row_ind, col_ind):
                if cost_matrix[r, c] <= self.max_match_dist:
                    self.tracks[r].update(detections[c][0], detections[c][1])
                    matched_tracks.add(r)
                    matched_dets.add(c)

            for i, t in enumerate(self.tracks):
                if i not in matched_tracks:
                    t.missed += 1

            for j, d in enumerate(detections):
                if j not in matched_dets:
                    self.tracks.append(KalmanTrack(d[0], d[1]))
        elif len(detections) > 0:
            for d in detections:
                self.tracks.append(KalmanTrack(d[0], d[1]))
        else:
            for t in self.tracks:
                t.missed += 1

        self.tracks = [t for t in self.tracks if t.missed <= self.max_missed]
        return [t for t in self.tracks if t.hits >= 3]



# =====================================================================
# 4c. ENLACE DE ENCODERS + ODOMETRIA EKF EN RASPBERRY + CONTROL DE RUTA
# =====================================================================
def wrap_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class MegaSerialLink:
    """Lee contadores acumulados de encoders; no usa la pose POS calculada por el Mega."""
    ENC_RE = re.compile(r"ENC\s+M1:\s*(-?\d+)\s+M2:\s*(-?\d+)\s+M3:\s*(-?\d+)\s+M4:\s*(-?\d+)\s+S:\s*(\d+)", re.I)

    def __init__(self, port, baud=115200):
        self.ser = serial.Serial(port, baudrate=baud, timeout=0.05, write_timeout=0.2)
        time.sleep(2.0)
        self.lock = threading.Lock()
        self.latest_enc = None  # (m1,m2,m3,m4, servo_posture)
        self.enc_seq = 0
        self.messages = deque(maxlen=200)
        self.running = True
        self.error = ""
        self.last_poll = 0.0
        self.thread = threading.Thread(target=self._reader, daemon=True)
        self.thread.start()

    def _reader(self):
        while self.running:
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue
                m = self.ENC_RE.search(line)
                with self.lock:
                    self.messages.append(line)
                    if m:
                        self.latest_enc = tuple(int(m.group(i)) for i in range(1, 6))
                        self.enc_seq += 1
            except (serial.SerialException, OSError) as e:
                self.error = str(e)
                break

    def send(self, command):
        try:
            # El firmware usa readStringUntil('\n'); debe enviarse LF real.
            self.ser.write((command.strip() + "\n").encode("ascii"))
            return True
        except (serial.SerialException, OSError) as e:
            self.error = str(e)
            return False

    def poll(self, hz=25.0):
        now = time.monotonic()
        if now - self.last_poll >= 1.0 / max(hz, 1.0):
            self.last_poll = now
            self.send("E")

    def get_state(self):
        with self.lock:
            msgs = list(self.messages)
            self.messages.clear()
            return self.latest_enc, self.enc_seq, msgs

    def close(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join(timeout=0.4)
        try:
            self.ser.close()
        except Exception:
            pass


class OdometryEKF:
    """EKF planar. Predice pose exclusivamente desde ticks de encoder; SLAM corrige la pose."""
    # Deben coincidir con config.h del Mega.
    PULSES_PER_REV = 800.0
    WHEEL_DIAMETER_M = 0.053
    WHEELBASE_M = 0.217
    TURN_DIAGONAL_M = math.hypot(0.217, 0.223)

    def __init__(self):
        self.x = np.zeros(3, dtype=float)
        self.P = np.diag([0.08**2, 0.08**2, math.radians(8.0)**2])
        self.initialized = False
        self.prev_ticks = None
        self.prev_posture = 0
        self.last_encoder_time = None

    def initialize(self, slam_pose, enc_state):
        self.x = np.asarray(slam_pose, dtype=float).copy()
        self.P = np.diag([0.04**2, 0.04**2, math.radians(4.0)**2])
        self.prev_ticks = tuple(enc_state[:4])
        self.prev_posture = int(enc_state[4])
        self.last_encoder_time = time.monotonic()
        self.initialized = True

    def predict_encoders(self, enc_state):
        if not self.initialized:
            return
        ticks = tuple(enc_state[:4])
        posture = int(enc_state[4])
        old = self.prev_ticks
        # Diferencias con manejo de wrap int32 de los contadores acumulados.
        ds_ticks = [((ticks[i] - old[i] + 2**31) % 2**32) - 2**31 for i in range(4)]
        self.prev_ticks = ticks
        self.prev_posture = posture
        # Todo en METROS (WHEEL_DIAMETER_M, WHEELBASE_M y TURN_DIAGONAL_M están en m).
        # Antes se dividía ds entre 1000 como si fuera mm: la traslación predicha salía 1000 veces
        # más chica (el SLAM la tapaba) y el ruido de proceso Q quedaba subestimado.
        m_per_tick = math.pi * self.WHEEL_DIAMETER_M / self.PULSES_PER_REV
        s1, s2, s3, s4 = [v * m_per_tick for v in ds_ticks]
        s13 = 0.5 * (s1 + s3)
        s24 = 0.5 * (s2 + s4)
        dtheta = (s13 - s24) / (self.TURN_DIAGONAL_M if posture else self.WHEELBASE_M)
        if posture:
            ds = 0.0
        else:
            ds = 0.5 * (s13 + s24)
        theta_mid = self.x[2] + 0.5 * dtheta
        # En el marco SLAM theta=0 apunta hacia +Y; positivo rota desde +Y hacia -X.
        self.x[0] -= ds * math.sin(theta_mid)
        self.x[1] += ds * math.cos(theta_mid)
        self.x[2] = wrap_pi(self.x[2] + dtheta)
        dist = abs(ds)
        Q = np.diag([(0.004 + 0.06 * dist)**2,
                     (0.004 + 0.06 * dist)**2,
                     math.radians(0.6 + 0.08 * abs(math.degrees(dtheta)))**2])
        self.P = self.P + Q

    def correct_slam(self, slam_pose, quality):
        if not self.initialized or quality < 0.30:
            return
        z = np.asarray(slam_pose, dtype=float)
        q = max(0.30, min(float(quality), 1.0))
        R = np.diag([(0.025 / q)**2, (0.025 / q)**2, math.radians(2.0 / q)**2])
        innovation = z - self.x
        innovation[2] = wrap_pi(innovation[2])
        S = self.P + R
        K = self.P @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.x[2] = wrap_pi(self.x[2])
        I = np.eye(3)
        self.P = (I - K) @ self.P @ (I - K).T + K @ R @ K.T

    @property
    def pose(self):
        return self.x.copy()


class RouteExecutor:
    """Seguimiento cerrado de ruta en Raspberry; Mega solo recibe consignas V y regula motores."""
    def __init__(self, link):
        self.link = link
        self.points = []
        self.index = 0
        self.active = False
        self.status = "inactivo"
        self.final_heading = None
        self.last_cmd = 0.0
        self.last_pose = None
        self.max_v = 80.0       # mm/s conservador para pruebas
        self.max_w = 0.35       # rad/s
        self.lookahead_tol = 0.10

    def start(self, points_map_xy, final_heading_slam, planner=None, bridge=None):
        if not self.link:
            self.status = "Enlace Mega no disponible"
            return False
        pts = np.asarray(points_map_xy, dtype=float)
        if len(pts) == 0:
            self.status = "Ruta vacía"
            return False
        if len(pts) > 48:
            idx = np.linspace(0, len(pts)-1, 48).round().astype(int)
            pts = pts[idx]
        self.points = [np.asarray(v, dtype=float) for v in pts]
        self.index = 0
        self.final_heading = float(final_heading_slam)
        self.active = True
        self.status = "seguimiento de ruta iniciado"
        self.link.send("C")
        self.link.send("V 0 0")
        return True

    def stop(self):
        self.active = False
        self.link.send("S")
        self.status = "parada solicitada"

    def update(self, pose):
        if not self.active or pose is None:
            return
        now = time.monotonic()
        if now - self.last_cmd < 0.08:
            return
        self.last_cmd = now
        xy = np.asarray(pose[:2], dtype=float)
        theta = float(pose[2])
        while self.index < len(self.points) and np.linalg.norm(self.points[self.index] - xy) < self.lookahead_tol:
            self.index += 1
        if self.index >= len(self.points):
            desired = self.final_heading
            err = wrap_pi(desired - theta)
            if abs(err) > 0.06:
                wz = float(np.clip(1.4 * err, -self.max_w, self.max_w))
                self.link.send(f"V 0 {wz:.4f}")
                self.status = "alineando orientación final"
            else:
                self.link.send("S")
                self.active = False
                self.status = "ruta completada"
            return
        target = self.points[self.index]
        delta = target - xy
        desired = math.atan2(-delta[0], delta[1])  # theta=0 apunta +Y
        err = wrap_pi(desired - theta)
        if abs(err) > 0.20:
            wz = float(np.clip(1.4 * err, -self.max_w, self.max_w))
            self.link.send(f"V 0 {wz:.4f}")
            self.status = f"girando hacia punto {self.index+1}/{len(self.points)}"
        else:
            # Modo recto; el Mega ejecuta su cinemática inversa y los PID de cada rueda.
            v = min(self.max_v, max(35.0, np.linalg.norm(delta) * 220.0))
            self.link.send(f"V {v:.1f} 0.0000")
            self.status = f"avanzando punto {self.index+1}/{len(self.points)}"

    def process_messages(self, messages):
        for line in messages:
            if "ERROR:" in line:
                self.active = False
                self.status = "error reportado por Mega: " + line

def parse_args():
    ap = argparse.ArgumentParser(description="LD06 por USB: SLAM 2D + mapa estático + landmarks + ruta A->B (Raspberry Pi)")
    ap.add_argument("--port", default="auto",
                    help="puerto serie del LD06 (ej: /dev/ttyUSB0). 'auto' = detectar")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help="baudios del LD06 (230400 de fábrica)")
    ap.add_argument("--no-imu", action="store_true", help="no usar el MPU9250")
    ap.add_argument("--imu-bus", type=int, default=1, help="bus I2C de la Raspberry (/dev/i2c-N)")
    ap.add_argument("--imu-addr", type=lambda v: int(v, 0), default=MPU_ADDR_DEFAULT,
                    help="direccion I2C del MPU: 0x68 (AD0 a GND) o 0x69 (AD0 a 3.3 V)")
    ap.add_argument("--imu-sign", type=int, choices=[1, -1], default=1,
                    help="signo inicial del giro (se verifica y corrige solo al girar el robot)")
    ap.add_argument("--mega-port", default=None,
                    help="puerto serie del Arduino Mega (ej. /dev/ttyACM0); si se omite, solo mapea/planifica")
    ap.add_argument("--mega-baud", type=int, default=115200,
                    help="baudios del Arduino Mega")
    ap.add_argument("--mega-poll-hz", type=float, default=25.0,
                    help="frecuencia de consulta de odometría del Mega")
    # --- geometría del robot / filtro de autodetección ---
    ap.add_argument("--robot-largo", type=float, default=ROBOT_LENGTH_M,
                    help="largo del robot en m (eje de avance, +Y del LiDAR). Por defecto 0.30")
    ap.add_argument("--robot-ancho", type=float, default=ROBOT_WIDTH_M,
                    help="ancho del robot en m (eje X del LiDAR). Por defecto 0.23")
    ap.add_argument("--self-margin", type=float, default=SELF_MARGIN_M,
                    help="holgura (m) alrededor del chasis para descartar puntos propios")
    ap.add_argument("--self-extra", action="append", default=[], metavar="CX,CY,HX,HY",
                    help="rectángulo extra a ignorar en el marco del LiDAR (mástil, brazo...). Repetible")
    ap.add_argument("--no-self-filter", action="store_true",
                    help="desactivar el filtro de autodetección (solo para comparar)")
    # --- visión / cosecha ---
    ap.add_argument("--vision", action="store_true",
                    help="habilitar cámara + clasificador de granos y la misión de cosecha ([V], [K])")
    ap.add_argument("--vision-imagen", default=None,
                    help="usar una foto fija en vez de la cámara (pruebas sin hardware)")
    ap.add_argument("--vision-hz", type=float, default=5.0, help="cuadros/s de la visión mientras está activa")
    ap.add_argument("--vision-n", type=int, default=5,
                    help="cuadros seguidos con la misma categoría para dar un grano por estable")
    ap.add_argument("--brazo-simulado", action="store_true",
                    help="brazo falso para probar todo el flujo sin hardware (no mueve nada)")
    ap.add_argument("--brazo-dynamixel", action="store_true",
                    help="habilitar brazo físico Dynamixel (solo tras calibrar y probar sin fruto)")
    ap.add_argument("--brazo-port", default="/dev/ttyUSB1",
                    help="puerto del brazo Dynamixel; debe ser distinto del LD06 y del Mega")
    ap.add_argument("--brazo-baud", type=int, default=1000000,
                    help="baudrate Dynamixel XL430 (por defecto 1000000)")
    ap.add_argument("--brazo-sin-mujoco", action="store_true",
                    help="desactivar la ventana MuJoCo del brazo físico")
    ap.add_argument("--brazo-max-dist", type=float, default=0.40,
                    help="distancia máxima cámara-fruto (m) permitida para iniciar agarre")
    ap.add_argument("--brazo-radio-mm", type=float, default=140.0,
                    help="radio de alcance del brazo calibrado en mm")
    ap.add_argument("--brazo-altura-mm", type=float, default=200.0,
                    help="altura objetivo inicial del fruto en mm; calibrar en el árbol real")
    ap.add_argument("--ir-borde", action="store_true",
                    help="reservado para habilitar sensores IR; desactivado hasta confirmar pines/cableado")
    ap.add_argument("--standoff", type=float, default=0.25,
                    help="holgura (m) entre el borde del robot y el árbol al aproximarse; ajustar para que "
                         "la cámara quede a ~0.32 m de los granos (distancia de enfoque calibrada)")
    ap.add_argument("--ronda-s", type=float, default=None,
                    help="tiempo de ronda en s (p. ej. 480): la misión termina al agotarse")
    return ap.parse_args()


def imu_error_hint(e):
    """Mensaje claro para los fallos tipicos del I2C en la Raspberry."""
    code = getattr(e, "errno", None)
    if isinstance(e, RuntimeError):
        return str(e)
    if code == 2:
        return "I2C desactivado: sudo raspi-config nonint do_i2c 0   y reinicia la Pi"
    if code == 13:
        return "sin permiso para I2C: sudo usermod -aG i2c $USER   y reinicia la sesion"
    if code in (5, 121, 110):
        return "el MPU no responde en esa direccion: revisa SDA=GPIO2(pin 3), SCL=GPIO3(pin 5), 3.3V, GND y AD0"
    return str(e)

def find_serial_port():
    """Devuelve el único puerto /dev/ttyUSB* o /dev/ttyACM* conectado; si hay 0 o varios, explica qué hacer."""
    ports = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
    if len(ports) == 1:
        return ports[0]
    if not ports:
        raise SystemExit("No se encontró ningún puerto USB-serie (/dev/ttyUSB* ni /dev/ttyACM*).\n"
                         "Revisa que el adaptador USB del LD06 esté conectado a la Raspberry (con cable de DATOS).")
    raise SystemExit("Hay varios puertos USB-serie: " + ", ".join(ports) +
                     "\nElige uno con:  --port /dev/ttyUSB0")



# =====================================================================
# 5. FUNCIONES DE CÁMARA Y RENDER DEL MAPA
# =====================================================================
def world_to_screen(pt_m, width, height, scale, pan_x, pan_y):
    """Convierte coordenadas métricas globales (x, y) a píxeles en pantalla aplicando pan y zoom."""
    sx = int(width / 2 + pan_x + pt_m[0] * scale)
    sy = int(height / 2 + pan_y - pt_m[1] * scale)  # Eje +Y hacia arriba
    return sx, sy


def screen_to_world(sx, sy, width, height, scale, pan_x, pan_y):
    """Convierte coordenadas de píxeles a coordenadas métricas globales (x, y)."""
    wx = (sx - width / 2 - pan_x) / scale
    wy = (height / 2 + pan_y - sy) / scale
    return wx, wy


BG_COLOR = (12, 14, 20)
FREE_COLOR = (20, 28, 40)
TENTATIVE_COLOR = (55, 62, 85)
STATIC_COLOR = (70, 150, 220)


def draw_map_layer(window, slam, width, height, scale, pan_x, pan_y):
    """Dibuja la grilla (libre / tentativo / ESTÁTICO) solo en la región visible, vectorizado."""
    x_min, y_max = screen_to_world(0, 0, width, height, scale, pan_x, pan_y)
    x_max, y_min = screen_to_world(
        width, height, width, height, scale, pan_x, pan_y)
    res, c, dim = slam.res, slam.center_cell, slam.grid_dim

    gx0 = max(int(np.floor(x_min / res + c)), 0)
    gx1 = min(int(np.ceil(x_max / res + c)), dim)
    gy0 = max(int(np.floor(y_min / res + c)), 0)
    gy1 = min(int(np.ceil(y_max / res + c)), dim)
    if gx1 <= gx0 or gy1 <= gy0:
        return

    lo = slam.log_odds[gy0:gy1, gx0:gx1]
    hits = slam.hits[gy0:gy1, gx0:gx1]
    rgb = np.empty((gy1 - gy0, gx1 - gx0, 3), dtype=np.uint8)
    rgb[:] = BG_COLOR
    rgb[lo < -0.25] = FREE_COLOR
    rgb[lo > 0.3] = TENTATIVE_COLOR
    rgb[(lo > slam.static_logodds_thr) & (hits >= slam.min_static_hits)] = STATIC_COLOR

    arr = np.ascontiguousarray(rgb[::-1].transpose(1, 0, 2))
    surf = pg.surfarray.make_surface(arr)
    wpx = max(1, int(round((gx1 - gx0) * res * scale)))
    hpx = max(1, int(round((gy1 - gy0) * res * scale)))
    surf = pg.transform.scale(surf, (wpx, hpx))
    sx, sy = world_to_screen(((gx0 - c) * res, (gy1 - c) * res),
                             width, height, scale, pan_x, pan_y)
    window.blit(surf, (sx, sy))


def draw_vision_panel(window, font, vision, mission, width, height):
    """Miniatura de la cámara (esquina inferior derecha) + lista de granos vistos."""
    import cv2
    snap = vision.snapshot()
    pw, ph = 384, 216
    x0, y0 = width - pw - 15, height - ph - 130
    if snap.imagen is not None:
        small = np.ascontiguousarray(cv2.resize(snap.imagen, (pw, ph))[:, :, ::-1])   # BGR -> RGB
        window.blit(pg.image.frombuffer(small.tobytes(), (pw, ph), "RGB"), (x0, y0))
    pg.draw.rect(window, (90, 95, 110), (x0 - 1, y0 - 1, pw + 2, ph + 2), 1)
    window.blit(font.render("CAMARA / GRANOS", True, (255, 220, 120)), (x0, y0 - 20))
    for i, g in enumerate(snap.granos[:6]):
        col = (120, 255, 140) if g.recogible else ((255, 200, 80) if g.estable else (170, 170, 185))
        window.blit(font.render(g.resumen()[:52], True, col), (x0, y0 + ph + 6 + i * 16))


def draw_marker(window, font, pos, label, color, width, height, scale, pan_x, pan_y):
    sx, sy = world_to_screen(pos, width, height, scale, pan_x, pan_y)
    pg.draw.circle(window, color, (sx, sy), 10, 3)
    pg.draw.circle(window, color, (sx, sy), 3)
    window.blit(font.render(label, True, color), (sx + 13, sy - 22))


# =====================================================================
# 6. BUCLE PRINCIPAL
# =====================================================================
def main():
    args = parse_args()
    mega_link = None
    if args.mega_port:
        try:
            mega_link = MegaSerialLink(args.mega_port, args.mega_baud)
            print(f"Arduino Mega conectado en {args.mega_port}; control de movimiento habilitado.")
        except Exception as e:
            print(f"No se pudo abrir Arduino Mega ({args.mega_port}): {e}")
            print("Se ejecutará solo SLAM/planificación, sin movimiento autónomo.")
    route_executor = RouteExecutor(mega_link) if mega_link else None
    odom_ekf = OdometryEKF()
    last_mega_seq = -1
    last_enc_state = None
    fused_pose = None
    last_ekf_slam_scan = -1
    pg.init()
    width, height = 1200, 1200
    scale = 100.0       # 100 píxeles = 1 metro
    pan_x = 0.0
    pan_y = 0.0
    dragging = False
    drag_travel = 0.0

    window = pg.display.set_mode((width, height))
    pg.display.set_caption("LD06 SLAM + IMU + EKF encoders + navegación A→B / árbol")
    font = pg.font.SysFont("monospace", 14, bold=True)
    clock = pg.time.Clock()

    # Huella del robot (LiDAR en el centro): sus puntos se descartan antes del SLAM.
    try:
        extra_rects = [[float(v) for v in s.split(",")] for s in args.self_extra]
        assert all(len(r) == 4 for r in extra_rects)
    except (ValueError, AssertionError):
        pg.quit()
        raise SystemExit("--self-extra debe tener el formato CX,CY,HX,HY (metros), p. ej. 0,0.10,0.05,0.04")
    footprint = RobotFootprint(args.robot_largo, args.robot_ancho, args.self_margin, extra_rects)
    robot_radius_plan = footprint.circumradius + 0.01      # radio que envuelve al robot (planificador)

    port = find_serial_port() if args.port == "auto" else args.port
    try:
        scanner = LD06Scanner(port=port, baudrate=args.baud,
                              footprint=None if args.no_self_filter else footprint)
    except (serial.SerialException, OSError) as e:
        pg.quit()
        raise SystemExit(f"No se pudo abrir {port}: {e}\n"
                         "Si dice 'Permission denied': sudo usermod -aG dialout $USER  y reinicia sesión.")
    fusion, imu_msg = None, "desactivado (--no-imu)"
    if not args.no_imu:
        try:
            gyro = MPU9250Gyro(bus=args.imu_bus, addr=args.imu_addr)
            print(f"Giroscopio {gyro.model} detectado. Calibrando sesgo: NO muevas el robot ({IMU_CALIB_S:.0f} s)...")
            gyro.wait_ready(15.0)
            fusion = ImuFusion(gyro, sign=args.imu_sign)
        except Exception as e:
            imu_msg = "no disponible: " + imu_error_hint(e)
            print("IMU desactivada ->", imu_msg)

    slam = LidarSLAM2D(map_size_m=20.0, resolution_m=0.04)
    slam.footprint = None if args.no_self_filter else footprint
    tracker = MultiObjectTracker()
    planner = GraphPlanner(slam, robot_radius=robot_radius_plan)

    # Visión (cámara + clasificador de color calibrado): hilo aparte, inactiva hasta que se pida.
    vision = None
    if args.vision or args.vision_imagen:
        try:
            from percepcion_granos import VisionWorker
            vision = VisionWorker(hz=args.vision_hz, imagen=args.vision_imagen, n_estable=args.vision_n)
            vision.start()
            print(f"Visión habilitada ({'foto fija ' + args.vision_imagen if args.vision_imagen else 'Pi Camera'}); "
                  "[V] enciende/apaga la cámara, [K] inicia la cosecha en el árbol seleccionado.")
        except Exception as e:
            vision = None
            print("Visión DESACTIVADA:", e)

    point_A = None     # origen de la ruta (None = posición actual del robot)
    point_B = None      # destino
    follow = False      # replanificar continuamente desde el robot
    last_replan = 0.0
    status_msg = ""

    last_scan_id = -1
    pts_world = np.empty((0, 2))
    confirmed_tracks = []
    dynamic_ids = set()
    registry = LandmarkRegistry()
    selected_name = None
    last_lm_time = 0.0

    def current_nav_pose():
        return fused_pose if fused_pose is not None else slam.pose.copy()

    def replan():
        if point_B is None:
            return
        start_pose = current_nav_pose()
        start = point_A if (point_A is not None and not follow) else start_pose[:2]
        planner.plan(np.asarray(start), np.asarray(point_B))

    def approach_tree(lm):
        """Planifica y ARRANCA la ruta de aproximación a un ARBOL (lo usan [G] y la misión [K])."""
        nonlocal point_A, point_B
        ap_pt = planner.approach_point(lm.pos, lm.radius, current_nav_pose()[:2], standoff=args.standoff)
        if ap_pt is None:
            return False, f"Sin acceso libre cerca de {lm.name}"
        point_B = ap_pt
        point_A = None
        replan()
        if planner.path is None:
            return False, f"Sin ruta a {lm.name}: {planner.msg}"
        if route_executor is None:
            return False, "Ruta calculada; conecta el Mega con --mega-port para ejecutarla"
        if not odom_ekf.initialized:
            return False, "Esperando encoders y pose SLAM para inicializar odometría"
        v = lm.pos - ap_pt
        final_theta = math.atan2(v[1], v[0]) - math.pi / 2.0      # quedar mirando al árbol
        if route_executor.start(planner.path[1:] if len(planner.path) > 1 else planner.path,
                                final_theta, planner, None):
            return True, f"aproximación a {lm.name} iniciada"
        return False, route_executor.status

    try:
        sensores_ir = SensoresIRBorde(enabled=args.ir_borde)
    except Exception as e:
        print("Sensores IR DESACTIVADOS:", e)
        sensores_ir = SensoresIRBorde(enabled=False)
    if args.ir_borde and not sensores_ir.enabled:
        print("AVISO: --ir-borde pedido pero IR_ENABLED=False en sensores_ir.py; los IR siguen deshabilitados.")
    brazo = None
    if args.brazo_simulado:
        brazo = BrazoSimulado()
    elif args.brazo_dynamixel:
        # Se compara con el puerto REAL del LD06 (con --port auto, args.port vale 'auto').
        if args.brazo_port in (port, args.mega_port):
            print(f"Brazo DESACTIVADO: el puerto Dynamixel ({args.brazo_port}) debe ser distinto al del "
                  f"LD06 ({port}) y al del Arduino Mega ({args.mega_port}).")
        else:
            try:
                from brazo_dynamixel import BrazoDynamixel
                brazo = BrazoDynamixel(
                    puerto=args.brazo_port, baud=args.brazo_baud,
                    mujoco=not args.brazo_sin_mujoco, max_dist_m=args.brazo_max_dist,
                    radio_obj_mm=args.brazo_radio_mm, altura_obj_mm=args.brazo_altura_mm)
                # Pase lo que pase (excepción, Ctrl+C), el brazo se estaciona antes de soltar el torque.
                atexit.register(brazo.cerrar)
            except Exception as e:
                brazo = None
                print("Brazo Dynamixel DESACTIVADO:", e)
        if brazo is None:
            print("Se continúa SIN brazo (BrazoNoIntegrado): la misión verá y clasificará, pero no recogerá.")
        elif vision is None:
            print("AVISO: el brazo está activo pero sin --vision no hay misión de cosecha que lo use.")
    if brazo is None:
        brazo = BrazoNoIntegrado()
    cfg_mision = ConfigMision()
    if args.ronda_s:
        cfg_mision.deadline_ronda = time.monotonic() + args.ronda_s
    mission = (MisionCosecha(vision, brazo, route_executor, approach_tree, cfg_mision)
               if vision is not None else None)

    running = True
    while running:
        for event in pg.event.get():
            if event.type == pg.QUIT:
                running = False

            # --- PAN (arrastre) y CLIC (fijar A / B) ---
            elif event.type == pg.MOUSEBUTTONDOWN:
                if event.button in (1, 2, 3):
                    dragging = True
                    drag_travel = 0.0

            elif event.type == pg.MOUSEBUTTONUP:
                if event.button in (1, 2, 3):
                    if dragging and event.button == 1 and drag_travel < 6:
                        wx, wy = screen_to_world(
                            event.pos[0], event.pos[1], width, height, scale, pan_x, pan_y)
                        if pg.key.get_mods() & pg.KMOD_SHIFT:
                            point_A = np.array([wx, wy])
                        else:
                            point_B = np.array([wx, wy])
                        replan()
                    dragging = False

            elif event.type == pg.MOUSEMOTION:
                if dragging:
                    dx, dy = event.rel
                    pan_x += dx
                    pan_y += dy
                    drag_travel += abs(dx) + abs(dy)

            # --- ZOOM CON RUEDA CENTRADO EN EL CURSOR ---
            elif event.type == pg.MOUSEWHEEL:
                mx, my = pg.mouse.get_pos()
                wx, wy = screen_to_world(
                    mx, my, width, height, scale, pan_x, pan_y)
                zoom_factor = 1.15 if event.y > 0 else (1.0 / 1.15)
                new_scale = np.clip(scale * zoom_factor, 15.0, 700.0)
                pan_x = mx - width / 2 - wx * new_scale
                pan_y = my - height / 2 + wy * new_scale
                scale = new_scale

            # --- TECLADO ---
            elif event.type == pg.KEYDOWN:
                if event.key == pg.K_r:
                    if route_executor and route_executor.active:
                        route_executor.stop()
                    if mission:
                        mission.abortar("reinicio del SLAM")
                    slam = LidarSLAM2D(map_size_m=20.0, resolution_m=0.04)
                    slam.footprint = None if args.no_self_filter else footprint
                    tracker = MultiObjectTracker()
                    planner = GraphPlanner(slam, robot_radius=robot_radius_plan)
                    registry = LandmarkRegistry()
                    selected_name = None
                    point_A = point_B = None
                    follow = False
                    odom_ekf.initialized = False
                    odom_ekf.prev_ticks = None
                    fused_pose = None
                    last_mega_seq = -1
                    last_enc_state = None
                    if mega_link:
                        mega_link.send("S")
                        mega_link.send("Z")
                    pan_x, pan_y = 0.0, 0.0
                    if fusion is not None:
                        fusion.reset()
                    status_msg = "SLAM reiniciado; esperando encoders para reinicializar EKF"
                elif event.key == pg.K_m:
                    slam.toggle_lock()
                    status_msg = "Mapa estatico CONGELADO (solo localizacion)" if slam.map_locked \
                        else "Mapeo reanudado"
                    replan()
                elif event.key == pg.K_c:
                    pan_x = -slam.pose[0] * scale
                    pan_y = slam.pose[1] * scale
                elif event.key == pg.K_o:
                    pan_x, pan_y = 0.0, 0.0
                elif event.key == pg.K_a:
                    point_A = slam.pose[:2].copy()
                    replan()
                elif event.key == pg.K_p:
                    replan()
                elif event.key == pg.K_f:
                    follow = not follow
                    last_replan = 0.0
                elif event.key == pg.K_x:
                    point_A = point_B = None
                    follow = False
                    planner.clear()
                elif event.key == pg.K_s:
                    slam.save()
                    registry.save()
                    status_msg = "Mapa y landmarks guardados (mapa_estatico.npz, landmarks.json)"
                elif event.key == pg.K_l:
                    if slam.load():
                        planner.clear()
                        got_lm = registry.load()
                        status_msg = ("Mapa y landmarks cargados" if got_lm
                                      else "Mapa cargado (sin landmarks.json)") + \
                            " - iniciar en el mismo origen/orientacion"
                    else:
                        status_msg = "No se pudo cargar mapa_estatico.npz"
                elif event.key == pg.K_TAB:
                    names = [l.name for l in registry.confirmed_items()]
                    if names:
                        if selected_name in names:
                            selected_name = names[(names.index(selected_name) + 1) % len(names)]
                        else:
                            selected_name = names[0]
                    else:
                        status_msg = "Aun no hay landmarks confirmados"
                elif event.key == pg.K_g:
                    lm = registry.get(selected_name)
                    if lm is None or lm.cls != "ARBOL":
                        status_msg = "Selecciona un landmark ARBOL confirmado con [TAB]"
                    else:
                        _ok, status_msg = approach_tree(lm)
                elif event.key == pg.K_k:
                    # Cosecha: aproximarse al ARBOL seleccionado, mirar los granos y recogerlos.
                    if mission is None:
                        status_msg = "Misión no disponible: ejecuta con --vision"
                    elif mission.activa:
                        mission.abortar("detenida con [K]")
                        if route_executor and route_executor.active:
                            route_executor.stop()
                        status_msg = "Misión de cosecha detenida"
                    else:
                        _ok, status_msg = mission.iniciar(registry.get(selected_name))
                elif event.key == pg.K_v:
                    if vision is None:
                        status_msg = "Visión no habilitada: ejecuta con --vision"
                    else:
                        vision.manual = not vision.active
                        vision.set_active(vision.manual)
                        status_msg = "Cámara ENCENDIDA (manual)" if vision.manual else "Cámara apagada"
                elif event.key in (pg.K_RETURN, pg.K_KP_ENTER):
                    if planner.path is None or point_B is None:
                        status_msg = "Selecciona B y calcula una ruta primero"
                    elif route_executor is None:
                        status_msg = "Ruta calculada; conecta Mega con --mega-port para ejecutarla"
                    elif not odom_ekf.initialized:
                        status_msg = "Esperando encoders y pose SLAM para inicializar odometría"
                    else:
                        pth = planner.path
                        if len(pth) > 1:
                            d = pth[-1] - pth[-2]
                            final_theta = math.atan2(d[1], d[0]) - math.pi/2.0
                        else:
                            final_theta = current_nav_pose()[2]
                        if route_executor.start(pth[1:] if len(pth) > 1 else pth,
                                                final_theta, planner, None):
                            status_msg = "Ejecución A→B iniciada en Mega"
                elif event.key == pg.K_ESCAPE:
                    if route_executor:
                        route_executor.stop()
                    if mission:
                        mission.abortar("ESC")
                    status_msg = "Parada solicitada al Mega"
                elif event.key == pg.K_b:
                    if fusion is not None:
                        fusion.gyro.start_calibration(IMU_CALIB_S)
                        fusion.reset()
                        status_msg = "Recalibrando giroscopio: NO muevas el robot"
                    else:
                        status_msg = "Giroscopio no disponible"
                elif event.key in (pg.K_PLUS, pg.K_EQUALS, pg.K_KP_PLUS):
                    scale = min(scale * 1.2, 700.0)
                elif event.key in (pg.K_MINUS, pg.K_KP_MINUS):
                    scale = max(scale / 1.2, 15.0)

        # --- Enlace Mega: Raspberry calcula odometría desde contadores acumulados ---
        if mega_link is not None:
            mega_link.poll(args.mega_poll_hz)
            enc_state, enc_seq, mega_messages = mega_link.get_state()
            if enc_state is not None and enc_seq != last_mega_seq:
                last_mega_seq = enc_seq
                last_enc_state = enc_state
                if not odom_ekf.initialized and slam.initialized:
                    odom_ekf.initialize(slam.pose, enc_state)
                    fused_pose = odom_ekf.pose
                    status_msg = "EKF inicializado: odometría por encoders calculada en Raspberry"
                elif odom_ekf.initialized:
                    odom_ekf.predict_encoders(enc_state)
                    fused_pose = odom_ekf.pose
            if route_executor:
                route_executor.process_messages(mega_messages)
                route_executor.update(current_nav_pose())
                if route_executor.active:
                    status_msg = route_executor.status
                elif route_executor.status not in ("inactivo", "ruta completada"):
                    status_msg = route_executor.status

        # --- Misión de cosecha (máquina de estados no bloqueante) ---
        if mission is not None:
            mission.update()
            # Mientras el brazo recoge puede entrar al plano del LiDAR: no se integra al mapa para no
            # dejar celdas "estáticas" fantasma (ni landmarks falsos) donde pasó el brazo.
            slam.hold_integration = mission.estado is EstadoMision.RECOGIENDO
            if mission.activa and mission.bitacora:
                status_msg = mission.bitacora[-1]

        # --- SLAM + tracking con cada barrido nuevo (10 Hz) ---
        scan_id, scan_data, scan_t = scanner.get_latest_scan_t()
        if scan_data is not None and scan_id != last_scan_id:
            last_scan_id = scan_id
            smask = slam.static_mask()

            # Dinámico = se mueve Y no está sobre el mapa estático. Los fragmentos de
            # pared que "se deslizan" al barrer el láser tienen velocidad falsa, pero
            # están sobre celdas estáticas, así que no se tratan como móviles.
            def _is_dynamic(t):
                return (t.hits >= 5 and np.linalg.norm(t.x[2:]) > 0.15
                        and not slam.near_static(t.x[:2].ravel(), smask))

            zones = [(t.x[:2].ravel().copy(), t.radius + 0.2)
                     for t in confirmed_tracks if _is_dynamic(t)]
            gyro_dth, gyro_alt = fusion.pre_step(scan_t) if fusion is not None else (None, None)
            pts_world, candidate_pts = slam.step(
                scan_data, zones, gyro_dtheta=gyro_dth, gyro_alt_dtheta=gyro_alt,
                gyro_trusted=bool(fusion is not None and fusion.verified))
            if fusion is not None:
                fusion.post_step(slam)
            if odom_ekf.initialized:
                odom_ekf.correct_slam(slam.pose, slam.quality)
                fused_pose = odom_ekf.pose
            else:
                fused_pose = slam.pose.copy()
            confirmed_tracks = tracker.update(pts_world, candidate_pts)
            dynamic_ids = {t.id for t in confirmed_tracks if _is_dynamic(t)}

        # Landmarks: clasificar por forma los clusters del mapa estático (1 Hz; 0.33 Hz con mapa congelado)
        if slam.initialized and time.time() - last_lm_time > (3.0 if slam.map_locked else 1.0):
            last_lm_time = time.time()
            registry.update(detect_static_clusters(slam),
                            allow_expire=not slam.map_locked)

        # Replanificación continua desde la posición del robot
        if follow and point_B is not None and time.time() - last_replan > 1.5:
            last_replan = time.time()
            planner.plan(current_nav_pose()[:2].copy(), np.asarray(point_B))

        # --- RENDERIZADO ---
        window.fill(BG_COLOR)

        ox, oy = world_to_screen(
            (0.0, 0.0), width, height, scale, pan_x, pan_y)

        # 1. Mapa (libre / tentativo / ESTÁTICO)
        draw_map_layer(window, slam, width, height, scale, pan_x, pan_y)

        # 2. Ejes y anillos métricos
        if 0 <= ox < width:
            pg.draw.line(window, (30, 36, 48), (ox, 0), (ox, height), 1)
        if 0 <= oy < height:
            pg.draw.line(window, (30, 36, 48), (0, oy), (width, oy), 1)
        for r_m in range(1, 15):
            r_px = int(r_m * scale)
            if -r_px < ox < width + r_px and -r_px < oy < height + r_px and r_px < 4000:
                gfxdraw.aacircle(window, ox, oy, r_px, (28, 34, 46))

        # 3. Nube de puntos actual del LiDAR
        for pt in pts_world:
            sx, sy = world_to_screen(pt, width, height, scale, pan_x, pan_y)
            if 2 <= sx < width - 2 and 2 <= sy < height - 2:
                gfxdraw.filled_circle(window, sx, sy, 2, (240, 240, 255))

        # 4. Ruta A -> B (grafo): ruta completa tenue + ruta suavizada
        if planner.path_raw is not None and len(planner.path_raw) > 1:
            raw_px = [world_to_screen(p, width, height, scale, pan_x, pan_y)
                      for p in planner.path_raw]
            pg.draw.lines(window, (110, 40, 95), False, raw_px, 1)
        if planner.path is not None and len(planner.path) > 1:
            sm_px = [world_to_screen(p, width, height, scale, pan_x, pan_y)
                     for p in planner.path]
            pg.draw.lines(window, (255, 60, 200), False, sm_px, 3)
            for p in sm_px:
                pg.draw.circle(window, (255, 160, 230), p, 4)

        # Landmarks (verde = ARBOL por ajuste de círculo, naranja = OBJ, gris = sin confirmar)
        for lm in registry.items:
            lx, ly = world_to_screen(lm.pos, width, height, scale, pan_x, pan_y)
            rpx = max(int(lm.radius * scale), 6)
            if not lm.confirmed:
                pg.draw.circle(window, (90, 95, 110), (lx, ly), rpx, 1)
                continue
            if lm.cls == "ARBOL":
                lcol = (90, 255, 120)
                pg.draw.circle(window, lcol, (lx, ly), rpx, 2)
                pg.draw.circle(window, lcol, (lx, ly), 3)
            else:
                lcol = (255, 170, 60)
                pg.draw.rect(window, lcol, (lx - rpx, ly - rpx, 2 * rpx, 2 * rpx), 2)
            if lm.name == selected_name:
                pg.draw.circle(window, (255, 255, 0), (lx, ly), rpx + 6, 2)
            window.blit(font.render(f"{lm.name} r={lm.radius:.2f}m", True, lcol),
                        (lx + rpx + 6, ly - 8))

        if point_A is not None:
            draw_marker(window, font, point_A, "A", (60, 255, 120),
                        width, height, scale, pan_x, pan_y)
        if point_B is not None:
            draw_marker(window, font, point_B, "B", (255, 70, 70),
                        width, height, scale, pan_x, pan_y)

        # 5. Trayectoria y pose del LiDAR
        if len(slam.trajectory) > 1:
            traj_px = [world_to_screen(
                p, width, height, scale, pan_x, pan_y) for p in slam.trajectory]
            pg.draw.lines(window, (0, 200, 120), False, traj_px, 2)

        nav_pose = current_nav_pose()
        rx, ry = world_to_screen(
            nav_pose[:2], width, height, scale, pan_x, pan_y)
        heading_pt = nav_pose[:2] + \
            np.array([-np.sin(nav_pose[2]), np.cos(nav_pose[2])]) * 0.35
        hx, hy = world_to_screen(
            heading_pt, width, height, scale, pan_x, pan_y)
        pg.draw.circle(window, (0, 255, 140), (rx, ry), 7, 2)
        pg.draw.line(window, (0, 255, 140), (rx, ry), (hx, hy), 3)
        # Huella del robot (zona de puntos propios descartados): debe envolver al chasis real.
        fp_world = LidarSLAM2D.transform_points(footprint.corners(), nav_pose)
        fp_px = [world_to_screen(p, width, height, scale, pan_x, pan_y) for p in fp_world]
        pg.draw.lines(window, (70, 140, 255), True, fp_px, 2)

        # 6. Objetos rastreados
        for trk in confirmed_tracks:
            pos = trk.x[:2].ravel()
            vel = trk.x[2:].ravel()
            speed = np.linalg.norm(vel)
            tx, ty = world_to_screen(pos, width, height, scale, pan_x, pan_y)
            rad_px = max(int(trk.radius * scale), 8)

            is_dyn = trk.id in dynamic_ids
            color = (255, 90, 60) if is_dyn else (80, 210, 255)
            if not is_dyn and not SHOW_STATIC_TRACKS:
                continue  # los objetos estáticos se muestran como landmarks

            if len(trk.history) > 1:
                hist_px = [world_to_screen(
                    p, width, height, scale, pan_x, pan_y) for p in trk.history]
                pg.draw.lines(window, color, False, hist_px, 2)

            pg.draw.circle(window, color, (tx, ty), rad_px, 2)

            if is_dyn:
                end_vel = pos + vel * 1.0
                vx_px, vy_px = world_to_screen(
                    end_vel, width, height, scale, pan_x, pan_y)
                pg.draw.line(window, (255, 230, 0),
                             (tx, ty), (vx_px, vy_px), 2)

            lbl = font.render(
                f"ID:{trk.id} ({pos[0]:.2f},{pos[1]:.2f})m {speed:.2f}m/s", True, color)
            window.blit(lbl, (tx + rad_px + 5, ty - 10))

        # 7. HUD
        nav_pose = current_nav_pose()
        deg_pose = np.degrees(nav_pose[2])
        static_cells = int(np.count_nonzero(slam.static_mask()))
        conf_lm = registry.confirmed_items()
        n_tree = sum(1 for l in conf_lm if l.cls == "ARBOL")
        lm_txt = (f"LANDMARKS : {len(conf_lm)} confirmados ({n_tree} arboles, "
                  f"{len(conf_lm) - n_tree} obstaculos) | Seleccionado: {selected_name or '--'}")
        if planner.path is not None:
            rem = planner.remaining(slam.pose[:2])
            arrived = point_B is not None and np.linalg.norm(
                slam.pose[:2] - point_B) < 0.25
            route_txt = (f"RUTA      : {planner.msg} | Largo {planner.length:.2f} m | "
                         f"Restante {rem:.2f} m | Nodos expandidos {planner.expanded}"
                         + (" | DESTINO ALCANZADO" if arrived else ""))
        else:
            route_txt = f"RUTA      : {planner.msg}"

        hud_lines = [
            f"POSE NAVEG.: X={nav_pose[0]:+.2f}m | Y={nav_pose[1]:+.2f}m | Theta={deg_pose:+.1f} deg | EKF={'ENCODERS+SLAM' if odom_ekf.initialized else 'esperando encoders/SLAM'}",
            f"TRACKS     : {len(confirmed_tracks)} objetos | Escala: {scale:.1f} px/m",
            f"MAPA       : {'CONGELADO (localizacion)' if slam.map_locked else 'MAPEANDO'} | "
            f"Celdas estaticas: {static_cells} | Muestras integradas: {slam.scan_count} | "
            f"Calidad ICP: {slam.quality:.2f}",
            route_txt,
            (f"MEGA      : {'conectado' if mega_link and not mega_link.error else 'sin enlace'} | "
             f"{route_executor.status if route_executor else 'solo planificación'}"),
            lm_txt,
            f"A={'robot' if point_A is None else f'({point_A[0]:.2f},{point_A[1]:.2f})'} | "
            f"B={'--' if point_B is None else f'({point_B[0]:.2f},{point_B[1]:.2f})'} | "
            f"Seguimiento: {'ON' if follow else 'OFF'}  {status_msg}",
            "[Clic] fijar B  [Shift+Clic] fijar A  [A] A=robot  [P] replanificar  [F] seguir  [X] borrar ruta",
            "[TAB] elegir landmark  [G] aproximar a árbol  [ENTER] ejecutar ruta A→B  [ESC] STOP  [B] recalibrar gyro",
            "[K] cosecha en el árbol seleccionado (aproximar+ver+recoger)  [V] cámara on/off",
            "[M] congelar/reanudar mapa  [S] guardar  [L] cargar  [C] centrar  [O] origen  [R] reset  [Arrastrar] pan  [Rueda] zoom",
        ]
        if sensores_ir.enabled or sensores_ir.pedido:
            hud_lines.append(sensores_ir.estado_texto())
        hud_lines.insert(0, scanner.status_text())
        hud_lines.insert(2, fusion.status_text() if fusion is not None else f"GYRO      : {imu_msg}")
        if mission is not None:
            hud_lines.insert(8, mission.status_text())
            hud_lines.insert(9, f"VISION    : {vision.status} | {brazo.resumen()}")
        else:
            hud_lines.insert(8, "VISION    : desactivada (ejecuta con --vision para cámara + cosecha)")
        for idx, text in enumerate(hud_lines):
            surf = font.render(text, True, (220, 230, 240))
            window.blit(surf, (15, 15 + idx * 20))

        # 8. Panel de cámara con los granos detectados (solo con la visión encendida)
        if vision is not None and vision.active:
            draw_vision_panel(window, font, vision, mission, width, height)

        pg.display.flip()
        clock.tick(60)

    if route_executor and route_executor.active:
        route_executor.stop()
    if mission is not None:
        mission.abortar("cierre del programa")
    if vision is not None:
        vision.stop()
    if mega_link is not None:
        mega_link.close()
    if fusion is not None:
        fusion.gyro.close()
    scanner.close()
    try:
        sensores_ir.cerrar()
    except Exception:
        pass
    if args.brazo_dynamixel and hasattr(brazo, "cerrar"):
        brazo.cerrar()
    pg.quit()


if __name__ == "__main__":
    main()
