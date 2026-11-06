"""Servos de DIRECCIÓN de las 4 ruedas: sentido y calibración, SIN subir el firmware (después de desarmar y rearmar).

Cada servo usa su tabla de µs para −90°, 0° y +90° guardada en la EEPROM del Mega (comandos Y / YS / YG). Ahí
manda la tabla, no SIGNO_SERVO de config.h (que solo da los valores de fábrica, YD). Si un servo quedó al revés
basta con intercambiar su −90 y su +90 (tecla x) y guardar (g).

    python3 -m tools.direccion            (mejor con las ruedas en el aire)

Convención: +90° = la PUNTA DELANTERA de la rueda hacia la IZQUIERDA del robot (visto desde arriba, el frente arriba).
Ruedas: 1 = M1 delantera derecha, 2 = M2 delantera izquierda, 3 = M3 trasera derecha, 4 = M4 trasera izquierda.

TECLAS
  i     PRUEBA: las 4 a +45°: tienen que quedar PARALELAS con la punta hacia la IZQUIERDA ( \\ \\ )
  k     las 4 a −45° ( / / )      t  las 4 RECTAS (0°)      o / l  las 4 a +90° / −90°
  1 2 3 4   elegir la rueda
  x     INVERTIR el sentido de la rueda elegida (intercambia su −90 y su +90)
  a s d     elegir el punto de la rueda elegida: −90 (punta a la DERECHA) / 0 (RECTA) / +90 (punta a la IZQUIERDA);
            la rueda va a ese punto
  ← →   mover ese punto ±10 µs        , .  ±2 µs        [ ]  ±50 µs
  g     GUARDAR en la EEPROM del Mega        y  ver la tabla        q  salir
Orden sugerido: i -> la que quede en la otra diagonal, elegirla (1-4) y x -> i otra vez -> cada rueda: s y ← →
hasta que quede recta (paralela al chasis, con una regla), d y a para ±90 (con una escuadra) -> g.
"""
from __future__ import annotations

import argparse
import re
import sys

NOMBRES = {0: "M1 delantera DERECHA", 1: "M2 delantera IZQUIERDA", 2: "M3 trasera DERECHA", 3: "M4 trasera IZQUIERDA"}
PUNTOS = {-1: "−90 (punta a la DERECHA)", 0: "0 (RECTA)", 1: "+90 (punta a la IZQUIERDA)"}
US_MIN, US_MAX = 500, 2500


def leer_tabla(r: str) -> list:
    t = [[int(a), int(b), int(c)] for _, a, b, c in re.findall(r"M([1-4])=(\d+)/(\d+)/(\d+)", r)]
    if len(t) != 4:
        raise RuntimeError(f"respuesta inesperada a Y: {r!r}")
    return t


def angulo_a_us(u, f):
    """Igual que el firmware (actuadores.h): f = ángulo / 90, lineal por tramo."""
    us = u[1] + (u[2] - u[1]) * f if f >= 0 else u[1] + (u[1] - u[0]) * f
    return int(round(min(US_MAX, max(US_MIN, us))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None)
    a = ap.parse_args()
    import termios
    import tty
    from cafebot.config import cargar
    from cafebot.drivers.mega import Mega
    from cafebot.drivers.puertos import resolver
    from tools.cabezal import leer_tecla
    cfg = cargar()
    mega = Mega(resolver(a.mega or cfg.puertos.mega, "mega"))
    print("Conectando con el Mega...")
    mega.conectar()
    mega.comando("S")
    tabla = leer_tabla(mega.comando("Y"))
    guardada = [list(f) for f in tabla]
    rueda, punto = 0, 0

    def mostrar():
        print("\r\n  tabla (µs para −90 / 0 / +90):   " + "   ".join(
            f"M{i + 1}={u[0]}/{u[1]}/{u[2]}{'' if u[2] > u[0] else ' (al revés que las de fábrica)'}"
            for i, u in enumerate(tabla)) + ("   [SIN GUARDAR]" if tabla != guardada else "") + "\r")

    def elegido():
        print(f"\r  rueda {NOMBRES[rueda]}, punto {PUNTOS[punto]}: {tabla[rueda][punto + 1]} µs        \r")

    def fijar(i, k, us):
        us = int(min(US_MAX, max(US_MIN, us)))
        tabla[i][k + 1] = us
        r = mega.comando(f"YS {i} {k} {us}")          # el Mega la guarda en la tabla y mueve el servo ahí
        if not r.startswith("Y us"):
            print(f"\r  ¡{r}!\r")

    def todas(f):
        for i in range(4):
            mega.comando(f"Y {i} {angulo_a_us(tabla[i], f)}")

    print(__doc__.split("TECLAS")[1])
    mostrar()
    viejo = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        while True:
            k = leer_tecla(0.2)
            if not k:
                continue
            if k in ("q", "Q"):
                if tabla != guardada:
                    print("\r\n  Hay cambios SIN GUARDAR (se pierden al apagar el Mega). g para guardar, q de nuevo para "
                          "salir igual.\r")
                    if leer_tecla(30) not in ("q", "Q"):
                        continue
                break
            if k in "1234":
                rueda = int(k) - 1
                elegido()
            elif k in ("i", "k", "t", "o", "l"):
                f = {"i": 0.5, "k": -0.5, "t": 0.0, "o": 1.0, "l": -1.0}[k]
                todas(f)
                print(f"\r  las 4 a {90 * f:+.0f}°" + ("  -> deben quedar PARALELAS con la punta a la IZQUIERDA ( \\ \\ )"
                                                    if k == "i" else "") + "\r")
            elif k == "x":
                u = tabla[rueda]
                bajo, alto = u[0], u[2]
                fijar(rueda, -1, alto)
                fijar(rueda, 1, bajo)
                print(f"\r  {NOMBRES[rueda]}: sentido INVERTIDO (−90 <-> +90). Probar con i y guardar con g.\r")
                mostrar()
            elif k in ("a", "s", "d"):
                punto = {"a": -1, "s": 0, "d": 1}[k]
                fijar(rueda, punto, tabla[rueda][punto + 1])
                elegido()
            elif k in ("izq", "der", ",", ".", "[", "]"):
                paso = {"izq": -10, "der": 10, ",": -2, ".": 2, "[": -50, "]": 50}[k]
                fijar(rueda, punto, tabla[rueda][punto + 1] + paso)
                elegido()
            elif k == "g":
                r = mega.comando("YG")
                guardada = [list(f) for f in tabla]
                print(f"\r  {r}\r")
                mostrar()
            elif k == "y":
                tabla = leer_tabla(mega.comando("Y"))
                mostrar()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)
        try:
            mega.comando("S")
        except Exception:
            pass
        mega.cerrar()
        print()


if __name__ == "__main__":
    main()
