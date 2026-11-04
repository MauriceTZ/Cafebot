"""Encontrar los puertos USB-serial aunque falten las reglas udev (pi/system/99-cafebot.rules).

Con las reglas, los nombres son fijos (/dev/cafebot-mega, -lidar, -u2d2). Sin ellas, los ttyUSB0/1/2 se reparten
según el orden en que se conectan en cada arranque, así que se revisa el fabricante del chip USB de cada uno:
    Mega (clon)  CH340  1a86   (original 2341)
    LD06         CP2102 10c4
    U2D2         FTDI   0403
- Si el nombre pedido no existe, se usa el único puerto con el chip que corresponde (y se avisa).
- Si existe pero es de OTRO aparato (p. ej. robot_local.json dice lidar = /dev/ttyUSB0 y ahora ahí quedó el U2D2),
  también se cambia al que corresponde, si hay exactamente uno.
Si no se puede decidir, se deja el nombre pedido (dará el error, o el aviso dice qué revisar).
"""
from __future__ import annotations

import os

VID = {"mega": (0x1A86, 0x2341), "lidar": (0x10C4,), "u2d2": (0x0403,)}
NOMBRE_CHIP = {0x1A86: "CH340 (Mega)", 0x2341: "Arduino (Mega)", 0x10C4: "CP2102 (LD06)", 0x0403: "FTDI (U2D2)"}


def _puertos() -> list:
    """[(dispositivo, vid)] de los puertos USB-serial (vacío si falta pyserial)."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [(p.device, p.vid) for p in list_ports.comports()]


def candidatos(que: str) -> list:
    return sorted(d for d, vid in _puertos() if vid in VID[que])


def chip_de(puerto: str):
    """vid del chip USB detrás de `puerto` (sigue los enlaces como /dev/cafebot-lidar), o None si no se sabe."""
    real = os.path.realpath(puerto)
    for d, vid in _puertos():
        if os.path.realpath(d) == real:
            return vid
    return None


def resolver(puerto: str | None, que: str, log=print) -> str | None:
    """`puerto` si existe y es del aparato `que` ("mega", "lidar", "u2d2"); si no, el único puerto con ese chip."""
    if not puerto or not puerto.startswith("/dev/"):
        return puerto
    c = candidatos(que)
    if not os.path.exists(puerto):
        if len(c) == 1:
            log(f"[puertos] {puerto} no existe: uso {c[0]} (chip USB del {que}). Para nombres fijos instalar "
                "pi/system/99-cafebot.rules (docs/hardware.md)")
            return c[0]
        return puerto
    vid = chip_de(puerto)
    if vid is None or vid in VID[que]:
        return puerto
    otro = NOMBRE_CHIP.get(vid, f"vid {vid:04x}")
    if len(c) == 1:
        log(f"[puertos] {puerto} es el {otro}, no el {que}: uso {c[0]}. Los ttyUSB cambian de número al reconectar: "
            "sacar los ttyUSB de robot_local.json o instalar pi/system/99-cafebot.rules (docs/hardware.md)")
        return c[0]
    log(f"[puertos] OJO: {puerto} es el {otro}, no el {que}, y no encuentro otro puerto con el chip del {que}")
    return puerto
