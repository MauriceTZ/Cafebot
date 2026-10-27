"""Prueba el driver de la Pi contra el firmware real simulado (tiempo real, ~25 s).
Correr desde pi/:  python3 -m tests.test_mega_sim
"""
import math
import time

from cafebot.drivers.mega import Mega
from tests.sim_serial import SerialSimulado, compilar


def main():
    compilar()
    mega = Mega("sim", log=lambda s: print("   <", s))
    mega.conectar(serial_obj=SerialSimulado())
    print("version:", mega.version)
    assert mega.telemetria(20).startswith("OK M")

    r = mega.avanzar(400)
    assert r.ok and abs(r.x - 400) < 10, r
    r = mega.girar(90)
    assert r.ok and abs(math.degrees(r.th) - 90) < 2, r
    r = mega.cangrejo(-90, 200)          # de lado hacia la derecha del robot
    assert r.ok, r
    time.sleep(0.2)
    t = mega.tele
    print(f"telemetria: x={t.x:.0f} y={t.y:.0f} th={math.degrees(t.th):.1f}")
    assert abs(t.x - 600) < 15 and abs(t.y) < 15, t   # mirando a +y, su derecha es +x

    # Watchdog: una sola orden V -> el Mega se detiene solo
    mega.velocidad(100, 0)
    ev = mega.eventos.get(timeout=2)
    while ev.tipo != "STOP":
        ev = mega.eventos.get(timeout=2)
    assert ev.args[0] == "WATCHDOG", ev

    # Fijar pose desde la Pi
    assert mega.fijar_pose(1000, 500, 0).startswith("POSE 1000 500 0")
    mega.cerrar()
    print("OK: driver Mega contra firmware simulado")


if __name__ == "__main__":
    main()
