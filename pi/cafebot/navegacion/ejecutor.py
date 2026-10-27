"""Ejecuta una ruta (lista de puntos del mapa) con las primitivas del Mega.

Para una base de ruedas orientables "parar–orientar–avanzar" es lo natural:
    por cada tramo:  W (girar hacia el punto)  →  A (avanzar, máx. tramo_max_m)
    y antes de cada orden se vuelve a leer la pose FUSIONADA (odometría + SLAM),
    así los errores de un tramo se corrigen en el siguiente.
Corre en un hilo; `detener()` lo corta en cualquier momento (y para el Mega).
"""
from __future__ import annotations

import math
import threading
from typing import Callable

import numpy as np

from cafebot.navegacion.geometria import envolver


class EjecutorRuta:
    def __init__(self, mega, pose_fn: Callable[[], np.ndarray], vel_mm_s=150.0, tramo_max_m=0.5,
                 tol_llegada_m=0.04, tol_rumbo_deg=3.0):
        self.mega = mega
        self.pose_fn = pose_fn
        self.vel = vel_mm_s
        self.tramo_max = tramo_max_m
        self.tol = tol_llegada_m
        self.tol_rumbo = math.radians(tol_rumbo_deg)
        self.estado = "inactivo"
        self.activo = False
        self._cancelar = threading.Event()
        self._hilo = None
        self.puntos: list[np.ndarray] = []
        self.indice = 0

    def iniciar(self, puntos, rumbo_final: float | None = None) -> bool:
        if self.mega is None:
            self.estado = "sin Mega conectado (solo planificación)"
            return False
        if self.activo:
            self.detener()
            self._hilo.join(2.0)
        self.puntos = [np.asarray(p, float) for p in puntos]
        if not self.puntos:
            self.estado = "ruta vacía"
            return False
        self.indice = 0
        self._cancelar.clear()
        self.activo = True
        self._hilo = threading.Thread(target=self._correr, args=(rumbo_final,), daemon=True)
        self._hilo.start()
        return True

    def detener(self):
        self._cancelar.set()
        if self.mega is not None:
            try:
                self.mega.parar()
            except Exception:
                pass
        self.estado = "detenido"

    # ------------------------------------------------------------------ hilo
    def _girar_hacia(self, rumbo) -> bool:
        err = envolver(rumbo - self.pose_fn()[2])
        if abs(err) <= self.tol_rumbo:
            return True
        r = self.mega.girar(math.degrees(err))
        return r.ok

    def _correr(self, rumbo_final):
        try:
            for self.indice, p in enumerate(self.puntos):
                for _ in range(20):                         # tramos hasta llegar a este punto
                    if self._cancelar.is_set():
                        return
                    pose = self.pose_fn()
                    d = p - pose[:2]
                    dist = float(np.hypot(*d))
                    if dist <= self.tol:
                        break
                    self.estado = f"punto {self.indice + 1}/{len(self.puntos)}: faltan {dist:.2f} m"
                    if not self._girar_hacia(math.atan2(d[1], d[0])):
                        self.estado = "giro abortado"
                        return
                    if self._cancelar.is_set():
                        return
                    # Re-leer la pose después del giro y avanzar como máximo un tramo
                    pose = self.pose_fn()
                    dist = float(np.hypot(*(p - pose[:2])))
                    r = self.mega.avanzar(1000 * min(dist, self.tramo_max), self.vel)
                    if not r.ok:
                        self.estado = f"avance abortado ({r.motivo})"
                        return
            if rumbo_final is not None and not self._cancelar.is_set():
                self.estado = "alineando rumbo final"
                if not self._girar_hacia(rumbo_final):
                    self.estado = "giro final abortado"
                    return
            self.estado = "ruta completada"
        except Exception as e:  # enlace caído, timeout...
            self.estado = f"error: {e}"
        finally:
            self.activo = False
