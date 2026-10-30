"""Encontrar los puertos USB-serial aunque falten las reglas udev (pi/system/99-cafebot.rules).

Con las reglas, los nombres son fijos (/dev/cafebot-mega, -lidar, -u2d2). Sin ellas (o si el chip USB del aparato no
calza con la regla), si el nombre pedido no existe se busca el puerto por el fabricante de su chip USB:
    Mega (clon)  CH340  1a86   (original 2341)
    LD06         CP2102 10c4
    U2D2         FTDI   0403
Si hay exactamente uno de ese fabricante se usa ese (y se avisa); si no, se deja el nombre pedido (dará el error).
"""
from __future__ import annotations

import os

VID = {"mega": (0x1A86, 0x2341), "lidar": (0x10C4,), "u2d2": (0x0403,)}


def candidatos(que: str) -> list:
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return sorted(p.device for p in list_ports.comports() if p.vid in VID[que])


def resolver(puerto: str | None, que: str, log=print) -> str | None:
    """`puerto` si existe; si no, el único puerto USB con el chip de `que` ("mega", "lidar", "u2d2")."""
    if not puerto or not puerto.startswith("/dev/") or os.path.exists(puerto):
        return puerto
    c = candidatos(que)
    if len(c) == 1:
        log(f"[puertos] {puerto} no existe: uso {c[0]} (chip USB del {que}). Para nombres fijos instalar "
            "pi/system/99-cafebot.rules (docs/hardware.md)")
        return c[0]
    return puerto
