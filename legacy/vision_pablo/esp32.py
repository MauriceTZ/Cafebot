"""Comunicación con la ESP32: comandos de alto nivel por serial.

Protocolo: la Pi envía una línea de texto y la ESP32 responde con una línea
`DONE [datos]` o `ERR <motivo>`. Cualquier otra línea (mensajes de depuración)
se registra y se ignora.

Comandos:
  MOVE <mm>              avanzar (negativo = retroceder)
  TURN <grados>          girar en el lugar (positivo = antihorario)
  APPROACH <mm>          avanzar hasta quedar a <mm> del frente según los ToF,
                         enderezándose hasta que ambos ToF midan igual
  LIFT ALTA|MEDIA|BAJA   elevador a la altura de una fila del árbol
  SLIDE <mm>             carro lateral a la posición ABSOLUTA <mm> (0 = centro)
  EXTEND / RETRACT       avance de la garra hacia el árbol / de vuelta
  GRIP / RELEASE         cerrar / abrir la garra
  FLAP RIPE|OVERRIPE     compuerta: a qué compartimento cae el grano
  DUMP RIPE|OVERRIPE     vaciar un compartimento (en la caja)
  STOP                   detener todo
  ODOM                   responde `DONE <x_mm> <y_mm> <rumbo_grados>`

APPROACH, LIFT y DUMP son nuevos respecto de la lista original: confirmar con
el equipo de diseño que el mecanismo los permite.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass


@dataclass
class Respuesta:
    ok: bool
    datos: str = ""


class ErrorRobot(Exception):
    pass


def duracion_estimada(comando: str) -> float:
    """Segundos que tarda cada comando. PROVISIONAL: reemplazar con tiempos
    medidos en el robot real. Lo usan el simulador y el planificador de tiempo."""
    partes = comando.split()
    nombre = partes[0]
    arg = partes[1] if len(partes) > 1 else ""
    if nombre == "MOVE":
        return 0.4 + abs(float(arg)) / 250.0       # ~250 mm/s
    if nombre == "TURN":
        return 0.4 + abs(float(arg)) / 90.0 * 1.2  # ~1.2 s cada 90°
    if nombre == "SLIDE":
        return 0.8                                 # peor caso aproximado
    return {"APPROACH": 2.0, "LIFT": 1.5, "EXTEND": 1.0, "RETRACT": 1.0,
            "GRIP": 0.6, "RELEASE": 0.5, "FLAP": 0.3, "DUMP": 2.5,
            "STOP": 0.05, "ODOM": 0.05}.get(nombre, 0.5)


class RelojReal:
    def __init__(self):
        self.inicio = time.monotonic()

    def ahora(self) -> float:
        return time.monotonic() - self.inicio

    def esperar(self, segundos: float):
        time.sleep(segundos)


class RelojSimulado:
    def __init__(self):
        self.t = 0.0

    def ahora(self) -> float:
        return self.t

    def esperar(self, segundos: float):
        self.t += segundos


class ESP32Simulada:
    """Responde DONE a todo, avanza un reloj simulado y lleva la pose exacta.
    Si se le pasa `mundo`, le avisa cada comando (para el simulador de ronda)."""

    def __init__(self, reloj: RelojSimulado, x=0.0, y=0.0, rumbo=90.0, mundo=None):
        self.reloj = reloj
        self.x, self.y, self.rumbo = x, y, rumbo  # rumbo 90 = mirando a +y
        self.mundo = mundo
        self.registro = []

    def comando(self, texto: str, timeout: float = 15.0) -> Respuesta:
        self.reloj.esperar(duracion_estimada(texto))
        self.registro.append((round(self.reloj.ahora(), 2), texto))
        partes = texto.split()
        if partes[0] == "MOVE":
            d = float(partes[1])
            self.x += d * math.cos(math.radians(self.rumbo))
            self.y += d * math.sin(math.radians(self.rumbo))
        elif partes[0] == "TURN":
            self.rumbo = (self.rumbo + float(partes[1])) % 360
        elif partes[0] == "ODOM":
            return Respuesta(True, f"{self.x:.1f} {self.y:.1f} {self.rumbo:.1f}")
        if self.mundo is not None:
            motivo = self.mundo.al_comando(texto, self)
            if motivo:
                return Respuesta(False, motivo)
        return Respuesta(True)


def buscar_puerto() -> str:
    """Primer dispositivo en /dev/serial/by-id/ (nombre estable, no cambia
    aunque se reconecte, a diferencia de /dev/ttyUSB0 o /dev/ttyACM0)."""
    import glob
    puertos = sorted(glob.glob("/dev/serial/by-id/*"))
    if not puertos:
        raise ErrorRobot("No hay ninguna ESP32 conectada por USB (/dev/serial/by-id/ vacío)")
    return puertos[0]


class ESP32Serial:
    """La ESP32 real por USB-serial. Misma interfaz que ESP32Simulada."""

    def __init__(self, puerto: str = None, baudios: int = 115200, log=print):
        import serial  # solo se necesita con la ESP32 real
        puerto = puerto or buscar_puerto()
        self.ser = serial.Serial(puerto, baudios, timeout=0.1)
        self.log = log
        time.sleep(2.0)  # la ESP32 se reinicia al abrir el puerto
        self.ser.reset_input_buffer()

    def comando(self, texto: str, timeout: float = 15.0) -> Respuesta:
        self.ser.write((texto + "\n").encode())
        limite = time.monotonic() + timeout
        while time.monotonic() < limite:
            linea = self.ser.readline().decode(errors="ignore").strip()
            if not linea:
                continue
            if linea == "DONE" or linea.startswith("DONE "):
                return Respuesta(True, linea[5:])
            if linea.startswith("ERR"):
                return Respuesta(False, linea[4:] or "sin motivo")
            self.log(f"[ESP32] {linea}")
        self.ser.write(b"STOP\n")
        return Respuesta(False, f"timeout de {timeout:.0f} s en '{texto}'")
