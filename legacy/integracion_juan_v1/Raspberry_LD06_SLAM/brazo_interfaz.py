"""Interfaz del brazo robótico (Dynamixel) con el resto del sistema.

El código real del brazo todavía no está integrado. La misión de cosecha solo habla con esta
interfaz, así que cuando el brazo exista basta con escribir UNA clase que herede de BrazoBase
(p. ej. BrazoDynamixel) e implementar sus métodos; nada más del SLAM cambia.

Contrato (todo NO bloqueante: se llama desde el bucle principal a ~60 Hz):

    disponible()              -> True si hay un brazo real o simulado conectado.
    iniciar_recogida(grano)   -> arranca la secuencia (acercarse, abrir, cerrar, levantar,
                                 retroceder y soltar en el depósito de su categoría). Devuelve
                                 False si no puede empezar. `grano` es percepcion_granos.Grano:
                                 usa grano.dx_mm / dy_mm / dist_m (desvío respecto del píxel
                                 objetivo) y grano.categoria (MADURO -> brazo/depósito de
                                 maduros, SOBREMADURO -> brazo/depósito de sobremaduros).
    estado()                  -> "LIBRE" | "OCUPADO" | "OK" | "FALLO". Tras leer "OK" o "FALLO"
                                 la misión llama a reconocer() y el brazo vuelve a "LIBRE".
    cancelar()                -> aborta y deja el brazo en posición segura (ESC / parada).
    deposito_lleno(categoria) -> True si ese depósito ya no admite más granos.
    ultimo_error()            -> (opcional) motivo del último rechazo/fallo, para la bitácora.

Recordatorio del reglamento: sacar UN grano a la vez y con movimientos lentos (los jueces
deben verlo); un grano verde fuera del árbol o abandonado resta 7 puntos.
"""
from __future__ import annotations

import time


class BrazoBase:
    nombre = "base"

    def disponible(self) -> bool:
        return False

    def iniciar_recogida(self, grano) -> bool:
        return False

    def estado(self) -> str:
        return "LIBRE"

    def reconocer(self):
        """La misión ya leyó OK/FALLO."""

    def cancelar(self):
        pass

    def deposito_lleno(self, categoria: str) -> bool:
        return False

    def ultimo_error(self) -> str:
        """Motivo del último rechazo / fallo (texto corto para la bitácora y el HUD); "" si no hay."""
        return ""

    def resumen(self) -> str:
        return self.nombre


class BrazoNoIntegrado(BrazoBase):
    """Valor por defecto: no hay brazo. La misión detecta y clasifica los granos, informa qué
    recogería y termina sin mover nada."""
    nombre = "brazo NO integrado"


class BrazoSimulado(BrazoBase):
    """Brazo falso para probar TODO el flujo (aproximación -> visión -> recogida -> depósito)
    sin hardware. Tarda `duracion_s` por grano y siempre tiene éxito (salvo `fallar_cada`)."""
    nombre = "brazo SIMULADO"

    def __init__(self, duracion_s=3.0, capacidad=8, fallar_cada=0):
        self.duracion_s = duracion_s
        self.capacidad = capacidad
        self.fallar_cada = fallar_cada
        self.guardados = {"MADURO": 0, "SOBREMADURO": 0}
        self._t_fin = None
        self._res = "LIBRE"
        self._grano = None
        self._intentos = 0

    def disponible(self) -> bool:
        return True

    def iniciar_recogida(self, grano) -> bool:
        if self._res != "LIBRE" or self.deposito_lleno(grano.categoria):
            return False
        self._grano = grano
        self._intentos += 1
        self._t_fin = time.monotonic() + self.duracion_s
        self._res = "OCUPADO"
        return True

    def estado(self) -> str:
        if self._res == "OCUPADO" and time.monotonic() >= self._t_fin:
            if self.fallar_cada and self._intentos % self.fallar_cada == 0:
                self._res = "FALLO"
            else:
                self.guardados[self._grano.categoria] += 1
                self._res = "OK"
        return self._res

    def reconocer(self):
        self._res = "LIBRE"

    def cancelar(self):
        self._res = "LIBRE"
        self._t_fin = None

    def deposito_lleno(self, categoria: str) -> bool:
        return self.guardados.get(categoria, 0) >= self.capacidad

    def resumen(self) -> str:
        return f"{self.nombre} (maduros={self.guardados['MADURO']} sobremaduros={self.guardados['SOBREMADURO']})"
