"""Puertos USB-serial que cambian de número (sin hardware).   python3 -m tests.test_puertos

  1. El nombre pedido no existe: se usa el único puerto con el chip del aparato.
  2. El nombre existe pero es de OTRO aparato (lidar = /dev/ttyUSB0 y ahí quedó el U2D2): se usa el que corresponde.
  3. El nombre existe y es del aparato: se deja (también a través de un enlace como /dev/cafebot-lidar).
"""
import os

from cafebot.drivers import puertos as P

# Así quedaron el 4 nov: el U2D2 en ttyUSB0
CONECTADOS = [("/dev/ttyUSB0", 0x0403), ("/dev/ttyUSB1", 0x1A86), ("/dev/ttyUSB2", 0x10C4)]


def main():
    P._puertos = lambda: CONECTADOS
    existen = {d for d, _ in CONECTADOS} | {"/dev/cafebot-lidar"}
    enlaces = {"/dev/cafebot-lidar": "/dev/ttyUSB2"}
    os.path.exists = lambda p: p in existen
    os.path.realpath = lambda p: enlaces.get(p, p)
    log = []
    assert P.resolver("/dev/cafebot-u2d2", "u2d2", log.append) == "/dev/ttyUSB0"
    assert P.resolver("/dev/ttyUSB0", "lidar", log.append) == "/dev/ttyUSB2"
    assert any("FTDI (U2D2), no el lidar" in t for t in log), log
    assert P.resolver("/dev/ttyUSB0", "mega", log.append) == "/dev/ttyUSB1"
    n = len(log)
    assert P.resolver("/dev/ttyUSB1", "mega", log.append) == "/dev/ttyUSB1"
    assert P.resolver("/dev/cafebot-lidar", "lidar", log.append) == "/dev/cafebot-lidar"
    assert P.resolver("/dev/ttyUSB0", "u2d2", log.append) == "/dev/ttyUSB0" and len(log) == n
    P._puertos = lambda: CONECTADOS[:2]                  # sin LiDAR conectado: no se adivina, se avisa
    assert P.resolver("/dev/ttyUSB0", "lidar", log.append) == "/dev/ttyUSB0" and "OJO" in log[-1]
    print("OK: puertos (nombre que no existe, puerto de otro aparato, puerto correcto)")


if __name__ == "__main__":
    main()
