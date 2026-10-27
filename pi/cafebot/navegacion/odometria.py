"""Odometría fusionada en la Raspberry: EKF planar (x, y, θ).

  PREDICCIÓN  con los desplazamientos que reporta el Mega en su telemetría (T a 20 Hz).
              El Mega ya integra según la postura de las ruedas (recto / giro / cangrejo), así
              que aquí no se re-implementa la cinemática (el código antiguo asumía tracción
              diferencial con ticks crudos: incorrecto en cangrejo).
              Si hay giroscopio verificado, el giro (dθ) lo da el giroscopio.
  CORRECCIÓN  con la pose del SLAM, pesada por la calidad del ICP.

Se guardan dos trayectorias para calibrar: la del EKF (fusionada) y la de SOLO odometría.
La diferencia entre ambas muestra cuánto deriva la odometría (patinaje, diámetro de rueda…).
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

from cafebot.navegacion.geometria import componer, envolver, relativo


class OdometriaEKF:
    def __init__(self):
        self.x = np.zeros(3)
        self.P = np.diag([0.02 ** 2, 0.02 ** 2, math.radians(2) ** 2])
        self.inicializada = False
        self._mega_prev = None            # última pose del Mega (m, m, rad) en SU marco
        self._gyro_prev = None
        self.solo_odom = np.zeros(3)      # integración pura de la odometría (para comparar)
        self.tray_ekf = deque(maxlen=3000)
        self.tray_odom = deque(maxlen=3000)
        self.saltos_descartados = 0
        # giroscopio: verificación automática del signo comparando con la odometría en giros
        self.gyro_signo = 1
        self.gyro_verificado = False
        self._votos = [0, 0]

    def inicializar(self, pose, mega_pose, gyro_yaw=None):
        self.x = np.asarray(pose, float).copy()
        self.solo_odom = self.x.copy()
        self._mega_prev = np.asarray(mega_pose, float)
        self._gyro_prev = gyro_yaw
        self.P = np.diag([0.02 ** 2, 0.02 ** 2, math.radians(2) ** 2])
        self.inicializada = True

    def predecir(self, mega_pose, gyro_yaw=None):
        """mega_pose: (x_m, y_m, θ) de la telemetría del Mega. gyro_yaw: yaw integrado (rad) o None."""
        if not self.inicializada:
            return
        mega_pose = np.asarray(mega_pose, float)
        d = relativo(self._mega_prev, mega_pose)          # desplazamiento en el sistema del robot
        self._mega_prev = mega_pose
        if math.hypot(d[0], d[1]) > 0.25 or abs(d[2]) > 0.8:  # salto (reinicio del Mega, R ...)
            self.saltos_descartados += 1
            self._gyro_prev = gyro_yaw
            return
        if gyro_yaw is not None and self._gyro_prev is not None:
            dg = envolver(gyro_yaw - self._gyro_prev)
            if not self.gyro_verificado and abs(d[2]) > math.radians(2):
                self._votos[0 if math.copysign(1, dg) == math.copysign(1, d[2]) else 1] += 1
                if sum(self._votos) >= 5:
                    self.gyro_signo = 1 if self._votos[0] >= self._votos[1] else -1
                    self.gyro_verificado = max(self._votos) / sum(self._votos) >= 0.8
            if self.gyro_verificado:
                d[2] = self.gyro_signo * dg
        self._gyro_prev = gyro_yaw
        self.solo_odom = componer(self.solo_odom, d)
        self.x = componer(self.x, d)
        dist = math.hypot(d[0], d[1])
        self.P = self.P + np.diag([(0.003 + 0.04 * dist) ** 2, (0.003 + 0.04 * dist) ** 2,
                                   math.radians(0.3 + 0.05 * abs(math.degrees(d[2]))) ** 2])
        self._guardar()

    def corregir_slam(self, pose_slam, calidad: float, informacion=None):
        """informacion: matriz 3x3 del ICP (slam.localizar). Donde el ICP ve poco (p. ej.
        desplazamiento lateral frente a caras paralelas) la incertidumbre es grande y manda
        la odometría; donde ve bien, manda el SLAM."""
        if not self.inicializada or calidad < 0.30:
            return
        q = min(max(calidad, 0.30), 1.0)
        piso = np.diag([0.01 ** 2, 0.01 ** 2, math.radians(0.5) ** 2])        # nunca "perfecto"
        if informacion is not None and np.any(informacion):
            previa = np.diag([1 / 1.0 ** 2, 1 / 1.0 ** 2, 1 / math.radians(30) ** 2])  # ~sin información
            R = np.linalg.inv(informacion * q + previa) * 25 + piso   # ×25: puntos vecinos correlacionados
        else:
            R = np.diag([(0.025 / q) ** 2, (0.025 / q) ** 2, math.radians(2.0 / q) ** 2])
        inn = np.asarray(pose_slam, float) - self.x
        inn[2] = envolver(inn[2])
        K = self.P @ np.linalg.inv(self.P + R)
        self.x = self.x + K @ inn
        self.x[2] = envolver(self.x[2])
        I = np.eye(3)
        self.P = (I - K) @ self.P @ (I - K).T + K @ R @ K.T
        self._guardar()

    def _guardar(self):
        if not self.tray_ekf or np.linalg.norm(self.x[:2] - self.tray_ekf[-1]) > 0.01:
            self.tray_ekf.append(self.x[:2].copy())
            self.tray_odom.append(self.solo_odom[:2].copy())

    @property
    def pose(self) -> np.ndarray:
        return self.x.copy()

    def deriva(self) -> tuple[float, float]:
        """(m, grados) entre la odometría pura y la pose fusionada: cuánto corrigió el SLAM."""
        return (float(np.linalg.norm(self.solo_odom[:2] - self.x[:2])),
                math.degrees(envolver(self.solo_odom[2] - self.x[2])))
