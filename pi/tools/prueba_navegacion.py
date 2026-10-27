"""Pruebas de navegación desde la terminal de la Raspberry (Mega conectado por USB).

    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 recta 1000
    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 giro 360
    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 cangrejo 90 300
    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 cuadrado 500
    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 ida_vuelta 1000
    python3 -m tools.prueba_navegacion --puerto /dev/ttyACM0 consola      # escribir comandos del Mega
    (agregar --sim para probar contra el firmware simulado, sin robot)

Cada prueba muestra lo que el robot CREE que hizo (odometría). Compáralo con lo que
mides con la huincha / transportador: esa diferencia es lo que hay que calibrar.
Ctrl+C detiene el robot.
"""
import argparse
import math
import sys
import time

from cafebot.drivers.mega import Mega


def linea_resultado(nombre, r, t0):
    estado = "OK " if r.ok else f"ABORTADO ({r.motivo})"
    print(f"  {nombre:<28} {estado}  pose x={r.x:7.0f} mm  y={r.y:7.0f} mm  θ={math.degrees(r.th):7.1f}°"
          f"   [{time.monotonic() - t0:4.1f} s]")
    return r.ok


def ejecutar(mega, pasos):
    for nombre, fn in pasos:
        t0 = time.monotonic()
        if not linea_resultado(nombre, fn(), t0):
            return False
    return True


def consola(mega):
    print("Consola del Mega: escribe comandos (A 500, W 90, L 90 300, P, Y, IS 1, ...). 'salir' para terminar.")
    mega.log = lambda s: print("   <", s)
    while True:
        try:
            linea = input("> ").strip()
        except EOFError:
            break
        if linea.lower() in ("salir", "exit", "q"):
            break
        if linea:
            mega._enviar(linea)
            time.sleep(0.3)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default="/dev/ttyACM0")
    ap.add_argument("--sim", action="store_true", help="firmware simulado (sin robot)")
    ap.add_argument("--vel", type=float, default=0, help="velocidad mm/s (0 = la del firmware)")
    ap.add_argument("prueba", choices=["recta", "giro", "cangrejo", "cuadrado", "ida_vuelta", "consola"])
    ap.add_argument("valores", nargs="*", type=float)
    a = ap.parse_args()

    mega = Mega(a.puerto)
    print("Conectando (el Mega se reinicia al abrir el puerto, ~2 s)...")
    if a.sim:
        from tests.sim_serial import SerialSimulado
        mega.conectar(serial_obj=SerialSimulado())
    else:
        mega.conectar()
    print("Conectado:", mega.version)
    mega.fijar_pose(0, 0, 0)
    v = a.vel
    p = a.valores
    try:
        if a.prueba == "consola":
            consola(mega)
        elif a.prueba == "recta":
            d = p[0] if p else 1000
            print(f"Avanzar {d:.0f} mm. Marca el suelo y mide al final.")
            ejecutar(mega, [(f"A {d:.0f}", lambda: mega.avanzar(d, v))])
        elif a.prueba == "giro":
            g = p[0] if p else 360
            print(f"Girar {g:.0f}° en el sitio. Marca la orientación inicial y mide al final.")
            ejecutar(mega, [(f"W {g:.0f}", lambda: mega.girar(g, timeout=60))])
        elif a.prueba == "cangrejo":
            ang, d = (p + [90, 300])[:2] if len(p) < 2 else p[:2]
            print(f"Cangrejo {d:.0f} mm hacia {ang:.0f}° (sin girar el robot).")
            ejecutar(mega, [(f"L {ang:.0f} {d:.0f}", lambda: mega.cangrejo(ang, d, v))])
        elif a.prueba == "cuadrado":
            l = p[0] if p else 500
            print(f"Cuadrado de {l:.0f} mm (4 × avanzar + girar 90°). Debe volver al punto de partida.")
            pasos = []
            for i in range(4):
                pasos += [(f"lado {i + 1}: A {l:.0f}", lambda: mega.avanzar(l, v)),
                          (f"lado {i + 1}: W 90", lambda: mega.girar(90))]
            ejecutar(mega, pasos)
        elif a.prueba == "ida_vuelta":
            d = p[0] if p else 1000
            print(f"Ida y vuelta de {d:.0f} mm con cangrejo lateral: prueba de repetibilidad.")
            ejecutar(mega, [(f"A {d:.0f}", lambda: mega.avanzar(d, v)),
                            ("L 90 300", lambda: mega.cangrejo(90, 300, v)),
                            (f"A -{d:.0f}", lambda: mega.avanzar(-d, v)),
                            ("L -90 300", lambda: mega.cangrejo(-90, 300, v))])
        x, y, th = mega.pose()
        print(f"\nPose final según el robot: x={x:.0f} mm  y={y:.0f} mm  θ={math.degrees(th):.1f}°")
    except KeyboardInterrupt:
        print("\nCtrl+C: deteniendo el robot")
    finally:
        mega.cerrar()


if __name__ == "__main__":
    main()
