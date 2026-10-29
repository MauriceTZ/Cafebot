"""Sensores QTR-1A SIMULADOS para el firmware simulado (tests/sim_serial.py).

Un hilo calcula, ~500 veces por segundo, dónde está cada sensor (pose real del robot + posición del
sensor) y qué lectura analógica daría sobre las líneas negras (rectángulos de cancha.Rect), y se la
manda a la planta del simulador con líneas "@A pin valor" (el firmware las lee con analogRead).

    ir = IRSimulado(mega, cfg.ir.posiciones, lineas, pose_fn)   # pose_fn() -> (x, y, θ) real, en el marco de las líneas
    ir.iniciar()
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

PINES_SIM = (62, 63, 64, 65)    # A8..A11 en el simulador (A0 = 54), como IR_PINES del firmware


def pose_mega_extrapolada(mega) -> np.ndarray:
    """Pose del Mega (m, m, rad) extrapolada desde la última telemetría con su velocidad: la telemetría
    llega cada 20–50 ms y en ese tiempo el robot avanza varios mm."""
    t = mega.tele
    dt = min(max(time.monotonic() - t.recibida, 0.0), 0.1) if t.recibida else 0.0
    th = t.th + t.wz * dt
    v = t.vx / 1000.0
    return np.array([t.x / 1000.0 + v * math.cos(t.th) * dt, t.y / 1000.0 + v * math.sin(t.th) * dt, th])


class IRSimulado:
    def __init__(self, mega, posiciones, lineas, pose_fn, blanco=140, negro=880, ruido=8.0,
                 radio_mancha_m=0.002, periodo_s=0.002, semilla=None):
        """lineas: lista de cancha.Rect (o de (nombre, Rect)). radio_mancha_m: zona que "ve" el sensor
        (a 3 mm del piso, unos pocos mm): da el paso gradual blanco→negro en el borde de la línea."""
        self.mega = mega
        self.sensores = [np.array(p, float) for p in posiciones]
        self.lineas = [l[1] if isinstance(l, tuple) else l for l in lineas]
        self.pose_fn = pose_fn
        self.blanco, self.negro, self.ruido = blanco, negro, ruido
        self.r = radio_mancha_m
        self.periodo = periodo_s
        self.rng = np.random.default_rng(semilla)
        self._ultimo = [None] * len(self.sensores)
        self._t_envio = [0.0] * len(self.sensores)
        self._corriendo = False
        self.valores = [blanco] * len(self.sensores)

    def cobertura(self, p) -> float:
        """0 = todo blanco, 1 = todo negro (distancia firmada al borde de la línea más cercana)."""
        mejor = 0.0
        for r in self.lineas:
            dx = abs(p[0] - r.cx) - r.ancho_x / 2
            dy = abs(p[1] - r.cy) - r.ancho_y / 2
            fuera = math.hypot(max(dx, 0.0), max(dy, 0.0)) if (dx > 0 or dy > 0) else max(dx, dy)
            mejor = max(mejor, min(1.0, max(0.0, 0.5 - fuera / (2 * self.r))))
        return mejor

    def lecturas(self) -> list[int]:
        x, y, th = self.pose_fn()
        c, s = math.cos(th), math.sin(th)
        out = []
        for sp in self.sensores:
            p = (x + c * sp[0] - s * sp[1], y + s * sp[0] + c * sp[1])
            v = self.blanco + (self.negro - self.blanco) * self.cobertura(p) + self.rng.normal(0, self.ruido)
            out.append(int(min(1023, max(0, v))))
        return out

    def _bucle(self):
        while self._corriendo:
            try:
                ahora = time.monotonic()
                for k, v in enumerate(self.lecturas()):
                    self.valores[k] = v
                    u = self._ultimo[k]
                    if u is None or abs(v - u) > 3 * self.ruido or ahora - self._t_envio[k] > 0.05:
                        self.mega.enviar_sim(f"@A {PINES_SIM[k]} {v}")
                        self._ultimo[k], self._t_envio[k] = v, ahora
            except Exception:
                pass
            time.sleep(self.periodo)

    def iniciar(self):
        self._corriendo = True
        threading.Thread(target=self._bucle, name="ir-sim", daemon=True).start()
        return self

    def detener(self):
        self._corriendo = False
