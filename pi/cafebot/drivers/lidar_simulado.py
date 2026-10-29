"""LD06 simulado (misma interfaz que drivers.ld06.LD06) para probar la navegación sin robot.

La pose "real" del robot se obtiene integrando los desplazamientos que reporta el Mega
(simulado o real) con un ERROR DE CALIBRACIÓN a propósito (escala de distancia y de giro):
así la odometría deriva igual que en la vida real y se puede comprobar que el SLAM la corrige.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

from cafebot.drivers.ld06 import ConfigLD06, Scan, mascara_huella
from cafebot.mundo.simulador_lidar import simular_scan
from cafebot.navegacion.geometria import componer, relativo


class LD06Simulado:
    def __init__(self, cfg: ConfigLD06, segmentos: np.ndarray, pose_inicial, pose_mega_fn,
                 escala_dist=1.03, escala_giro=1.02, hz=10.0, semilla=None):
        """pose_inicial: pose del robot en la cancha al arrancar (m, m, rad).
        pose_mega_fn: función que devuelve la pose del Mega (m, m, rad) en su propio marco.
        escala_*: cuánto MÁS cree moverse la odometría que lo real (1,03 = 3 % de más)."""
        self.cfg = cfg
        self.segmentos = segmentos
        self.real = np.asarray(pose_inicial, float)
        self.pose_mega_fn = pose_mega_fn
        self.esc_d, self.esc_g = escala_dist, escala_giro
        self.periodo = 1.0 / hz
        self.rng = np.random.default_rng(semilla)
        self._scan = None
        self._n = 0
        self._cv = threading.Condition()
        self._corriendo = False
        self._mega_prev = None

    def iniciar(self):
        self._corriendo = True
        threading.Thread(target=self._bucle, name="ld06-sim", daemon=True).start()

    def _bucle(self):
        while self._corriendo:
            pm = np.asarray(self.pose_mega_fn(), float)
            if self._mega_prev is not None:
                d = relativo(self._mega_prev, pm)
                if math.hypot(d[0], d[1]) < 0.25:
                    self.real = componer(self.real, np.array([d[0] / self.esc_d, d[1] / self.esc_d, d[2] / self.esc_g]))
            self._mega_prev = pm
            s = simular_scan(tuple(self.real), self.segmentos, self.cfg, self.rng)
            x, y = s.xy()
            propios = mascara_huella(x, y, self.cfg)
            scan = Scan(time.monotonic(), s.angulos[~propios], s.distancias[~propios], s.intensidades[~propios],
                        rpm=600.0, puntos_propios=int(propios.sum()))
            with self._cv:
                self._scan = scan
                self._n += 1
                self._cv.notify_all()
            time.sleep(self.periodo)

    def pose_real_ahora(self, pose_mega=None) -> np.ndarray:
        """Pose real AHORA (no solo en el último barrido): la del último barrido + lo que se movió el
        Mega desde entonces, con el mismo error de calibración. La usa el IR simulado."""
        pm = np.asarray(pose_mega if pose_mega is not None else self.pose_mega_fn(), float)
        if self._mega_prev is None:
            return self.real.copy()
        d = relativo(self._mega_prev, pm)
        if math.hypot(d[0], d[1]) >= 0.25:
            return self.real.copy()
        return componer(self.real, np.array([d[0] / self.esc_d, d[1] / self.esc_d, d[2] / self.esc_g]))

    def ultimo_scan(self):
        with self._cv:
            return self._scan

    def esperar_scan(self, timeout=1.0):
        with self._cv:
            n = self._n
            self._cv.wait_for(lambda: self._n > n, timeout)
            return self._scan if self._n > n else None

    def estadisticas(self):
        return {"ok": self._n, "crc_malos": 0, "scans": self._n}

    def detener(self):
        self._corriendo = False
