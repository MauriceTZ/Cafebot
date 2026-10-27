"""Mover la base con el teclado (pruebas, NUNCA en una ronda oficial).

    python3 -m tools.teleop --puerto /dev/cafebot-mega
    python3 -m tools.teleop --sim          # contra el firmware simulado

Flechas: avanzar/retroceder, girar en el sitio.   A / D: cangrejo izq / der.
Espacio: parar.   + / -: velocidad.   Q: salir.
"""
import argparse
import curses
import math
import time

from cafebot.drivers.mega import Mega


def bucle(pantalla, mega: Mega):
    curses.curs_set(0)
    pantalla.nodelay(True)
    vel = 120.0
    accion = None
    t_tecla = 0.0
    while True:
        k = pantalla.getch()
        ahora = time.monotonic()
        if k != -1:
            t_tecla = ahora
            if k in (ord("q"), ord("Q")):
                break
            elif k == ord(" "):
                accion = None
                mega.parar()
            elif k in (ord("+"), ord("=")):
                vel = min(300, vel + 20)
            elif k == ord("-"):
                vel = max(40, vel - 20)
            else:
                accion = {curses.KEY_UP: "adelante", curses.KEY_DOWN: "atras", curses.KEY_LEFT: "izq",
                          curses.KEY_RIGHT: "der", ord("a"): "k_izq", ord("d"): "k_der"}.get(k, accion)
        # La terminal repite la tecla mientras se mantiene; si deja de llegar, parar
        if accion and ahora - t_tecla > 0.6:
            accion = None
        w = vel / 150.0
        if accion == "adelante":
            mega.velocidad(vel, 0)
        elif accion == "atras":
            mega.velocidad(-vel, 0)
        elif accion == "izq":
            mega.velocidad(0, w)
        elif accion == "der":
            mega.velocidad(0, -w)
        elif accion == "k_izq":
            mega.velocidad_cangrejo(90, vel)
        elif accion == "k_der":
            mega.velocidad_cangrejo(-90, vel)
        t = mega.tele
        pantalla.erase()
        pantalla.addstr(0, 0, f"Cafebot teleop | vel {vel:.0f} mm/s | accion: {accion or '-'}")
        pantalla.addstr(2, 0, f"x={t.x:7.0f} mm  y={t.y:7.0f} mm  th={math.degrees(t.th):7.1f}°")
        pantalla.addstr(3, 0, f"vx={t.vx:5.0f} mm/s  postura={t.postura}  IR={t.ir:04b}")
        pantalla.addstr(5, 0, "Flechas mover/girar · A/D cangrejo · espacio parar · +/- vel · Q salir")
        pantalla.refresh()
        time.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="/dev/cafebot-mega")
    ap.add_argument("--sim", action="store_true", help="usar el firmware simulado")
    a = ap.parse_args()
    mega = Mega(a.puerto)
    if a.sim:
        from tests.sim_serial import SerialSimulado
        mega.conectar(serial_obj=SerialSimulado())
    else:
        mega.conectar()
    mega.telemetria(10)
    try:
        curses.wrapper(bucle, mega)
    finally:
        mega.cerrar()


if __name__ == "__main__":
    main()
