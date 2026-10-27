"""Giroscopio Z del MPU9250 (o MPU6500/6050) por I2C, integrado a un ángulo de guiñada.

Portado de legacy/integracion_juan_v1. Solo se usa el GIROSCOPIO: el acelerómetro no sirve
para posición y el magnetómetro se distorsiona con motores y estructuras metálicas.
El sesgo se calibra con el robot QUIETO al arrancar (repite si detecta movimiento).
El signo (montado al revés o no) lo verifica sola la odometría (ver navegacion/odometria.py).
Requiere: pip install smbus2  (o sudo apt install python3-smbus2)
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque

import numpy as np

NOMBRES = {0x71: "MPU9250", 0x73: "MPU9255", 0x70: "MPU6500", 0x68: "MPU6050"}
LSB_POR_DPS = 65.5          # rango ±500 °/s


class Giroscopio:
    def __init__(self, bus: int = 1, direccion: int = 0x68, hz: float = 200.0, calib_s: float = 2.0):
        from smbus2 import SMBus

        self.dir = direccion
        self.periodo = 1.0 / hz
        self.bus = SMBus(bus)
        try:
            who = self.bus.read_byte_data(direccion, 0x75)
            if who not in NOMBRES:
                raise RuntimeError(f"WHO_AM_I=0x{who:02X} en 0x{direccion:02X}: no parece un MPU")
            self.modelo = NOMBRES[who]
            self.bus.write_byte_data(direccion, 0x6B, 0x80)   # reset
            time.sleep(0.1)
            self.bus.write_byte_data(direccion, 0x6B, 0x01)   # despertar, reloj PLL
            time.sleep(0.05)
            self.bus.write_byte_data(direccion, 0x19, 4)      # 200 Hz
            self.bus.write_byte_data(direccion, 0x1A, 0x03)   # pasa-bajos ~41 Hz
            self.bus.write_byte_data(direccion, 0x1B, 0x08)   # ±500 °/s
        except Exception:
            self.bus.close()
            raise
        self.yaw = 0.0
        self.sesgo = 0.0
        self.calib_std_dps = 0.0
        self.error = ""
        self.ultima_lectura = 0.0
        self._lock = threading.Lock()
        self._calib: list[float] = []
        self._calib_s = calib_s
        self._calib_hasta = time.monotonic() + calib_s
        self._intentos = 0
        self.listo = False
        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle, name="imu", daemon=True)
        self._hilo.start()

    def recalibrar(self):
        """Con el robot quieto."""
        self._calib, self._intentos, self.listo = [], 0, False
        self._calib_hasta = time.monotonic() + self._calib_s

    def _fin_calibracion(self):
        a = np.array(self._calib) if self._calib else np.zeros(1)
        std = math.degrees(float(a.std()))
        if std > 1.0 and self._intentos < 3:               # se movió: repetir
            self._intentos += 1
            self._calib = []
            self._calib_hasta = time.monotonic() + self._calib_s
            return
        self.sesgo, self.calib_std_dps = float(a.mean()), std
        self._calib_hasta = None
        self.listo = True

    def _bucle(self):
        t_ant = w_ant = None
        sig = time.monotonic()
        while self._corriendo:
            try:
                d = self.bus.read_i2c_block_data(self.dir, 0x47, 2)
            except OSError as e:
                self.error = f"I2C: {e}"
                time.sleep(0.05)
                continue
            t = time.monotonic()
            crudo = (d[0] << 8) | d[1]
            crudo -= 0x10000 if crudo >= 0x8000 else 0
            w = math.radians(crudo / LSB_POR_DPS)
            self.ultima_lectura, self.error = t, ""
            if self._calib_hasta is not None:
                self._calib.append(w)
                if t >= self._calib_hasta:
                    self._fin_calibracion()
            elif w_ant is not None:
                with self._lock:
                    self.yaw += 0.5 * ((w_ant - self.sesgo) + (w - self.sesgo)) * (t - t_ant)
            w_ant, t_ant = w, t
            sig += self.periodo
            espera = sig - time.monotonic()
            if espera > 0:
                time.sleep(espera)
            else:
                sig = time.monotonic()

    def leer_yaw(self) -> float | None:
        """Yaw integrado (rad, sin envolver) o None si no está listo / no responde."""
        if not self.listo or time.monotonic() - self.ultima_lectura > 0.5:
            return None
        with self._lock:
            return self.yaw

    def estado(self) -> str:
        if self.error and time.monotonic() - self.ultima_lectura > 0.5:
            return f"ERROR {self.error}"
        if not self.listo:
            return "calibrando sesgo: NO mover el robot"
        return (f"{self.modelo} yaw={math.degrees(self.yaw):+.1f}° sesgo={math.degrees(self.sesgo):+.3f}°/s"
                + (" (CALIBRÓ CON MOVIMIENTO: recalibrar quieto)" if self.calib_std_dps > 1.0 else ""))

    def cerrar(self):
        self._corriendo = False
        self._hilo.join(0.5)
        try:
            self.bus.close()
        except Exception:
            pass
