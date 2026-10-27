"""Prueba el decodificador LD06 con paquetes sintéticos (sin hardware).
Correr desde pi/:  python3 -m tests.test_ld06
"""
import math
import random
import struct

import numpy as np

from cafebot.drivers.ld06 import CABECERA, VER_LEN, ConfigLD06, Ensamblador, crc8


def paquete(ang_ini_deg, paso_deg, dist_mm, intensidad=200, velocidad=3600):
    ang_fin = (ang_ini_deg + 11 * paso_deg) % 360
    cuerpo = struct.pack("<BBHH", CABECERA, VER_LEN, velocidad, int(ang_ini_deg * 100) % 36000)
    for d in dist_mm:
        cuerpo += struct.pack("<HB", int(d), intensidad)
    cuerpo += struct.pack("<HH", int(ang_fin * 100), 1234)
    return cuerpo + bytes([crc8(cuerpo)])


def vuelta(dist_fn, paso=0.8):
    datos = b""
    a = 0.0
    while a < 360:
        dist = [dist_fn((a + i * paso) % 360) for i in range(12)]
        datos += paquete(a, paso, dist)
        a += 12 * paso
    return datos


def main():
    # Pared a 1 m delante del sensor (ángulo LiDAR 0) y nada en el resto
    def d(a):
        return 1000 if (a < 10 or a > 350) else 0

    cfg = ConfigLD06(huella_x_m=(-0.1, 0.1), huella_y_m=(-0.1, 0.1))
    ens = Ensamblador(cfg)
    datos = vuelta(d) + vuelta(d) + vuelta(d)

    # Corromper algunos bytes y cortar en trozos aleatorios
    datos = bytearray(datos)
    for _ in range(5):
        datos[random.randrange(len(datos))] ^= 0xFF
    scans = []
    i = 0
    while i < len(datos):
        n = random.randint(1, 100)
        scans += ens.alimentar(bytes(datos[i:i + n]))
        i += n

    print("scans:", len(scans), "paquetes ok:", ens.paquetes_ok, "crc malos:", ens.paquetes_malos)
    assert len(scans) >= 2 and ens.paquetes_malos >= 1
    x, y = scans[-1].xy()
    assert len(x) > 0 and np.all(np.abs(x - 1.0) < 0.02), (x[:5], y[:5])

    # Sentido: un punto a 90° del LiDAR (horario) debe quedar a la DERECHA del robot (y < 0)
    ens2 = Ensamblador(cfg)
    s = ens2.alimentar(vuelta(lambda a: 800 if 85 < a < 95 else 0) * 2)
    x, y = s[-1].xy()
    assert np.all(y < -0.7), y
    print("OK: LD06 (CRC, resincronización, vueltas, sistema del robot)")


if __name__ == "__main__":
    random.seed(1)
    main()
