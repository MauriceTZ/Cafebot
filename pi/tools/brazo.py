"""Probar el brazo REAL (XL430 por U2D2) paso a paso, de lo más seguro a lo más completo.

    python3 -m tools.brazo salud                     # 1. ¿responden los 6 servos? firmware, °C, V, errores
    python3 -m tools.brazo leer                      # 2. SIN torque: mover a mano y ver ángulos y punto de la garra
                                                     #    (tecla G = grabar la postura actual con un nombre)
    python3 -m tools.brazo ir trabajo                # 3. ir a una postura (trabajo, descanso, deposito_maduro, ...)
    python3 -m tools.brazo garra cerrar              #    abrir | cerrar | <ángulo>
    python3 -m tools.brazo punto 0.20 0.0 0.45       # 4. llevar la garra a (x, y, z) del ROBOT, en metros
    python3 -m tools.brazo teleop                    #    mover la garra con el teclado
    python3 -m tools.brazo agarrar 0.20 0.0 0.407 --sin-deposito   # 5. secuencia completa de un grano
    python3 -m tools.brazo apagar                    # ir a descanso y soltar el torque (sujetar el brazo)

Puerto: el de config (/dev/cafebot-u2d2 con las reglas udev); si no existe, busca el U2D2 (FTDI) solo.
Sistema del robot: x adelante, y izquierda, z desde el SUELO. Base del brazo: ConfigMision.brazo_x/y/z.
"""
from __future__ import annotations

import argparse
import curses
import glob
import math
import os
import sys
import time

import numpy as np

from cafebot.config import cargar
from cafebot.mundo import cancha

NOMBRES = {9: "base", 8: "hombro", 11: "codo1", 7: "codo2", 5: "muñeca", 4: "garra"}


def buscar_u2d2(preferido: str) -> str:
    if os.path.exists(preferido):
        return preferido
    ftdi = sorted(glob.glob("/dev/serial/by-id/*FTDI*"))
    if ftdi:
        return os.path.realpath(ftdi[0])
    raise SystemExit(f"No encuentro el U2D2 ({preferido}). Conéctalo o usa --puerto /dev/ttyUSBx "
                     "(ojo: el LD06 también es ttyUSB; `ls -l /dev/serial/by-id` dice cuál es cuál).")


def confirmar(txt: str, si: bool):
    if si:
        return
    r = input(f"{txt}  ¿Seguir? [s/N] ").strip().lower()
    if r not in ("s", "si", "sí", "y"):
        raise SystemExit("Cancelado.")


def crear(a):
    from cafebot.arm.brazo import BrazoXL430
    cfg = cargar()
    puerto = a.puerto or buscar_u2d2(cfg.puertos.u2d2)
    b = BrazoXL430(cfg, puerto)
    print(f"U2D2 en {puerto}. Conectando y configurando...")
    vivos = b.conectar()
    viejo = [s for s, (_, fw) in vivos.items() if fw < 42]
    print(f"Servos: {', '.join(f'{s} {NOMBRES[s]} fw{fw}' for s, (_, fw) in sorted(vivos.items()))}")
    if viejo:
        print(f"Firmware < 42 en {viejo}: se usa perfil por velocidad (actualizar con Dynamixel Wizard si se puede).")
    return b


def linea_salud(b) -> str:
    return "  ".join(f"{NOMBRES[s.id]}:{s.temp_c}°C {s.volt_v:.1f}V carga{s.carga_pct:+.0f}%"
                     + (f" ERROR {','.join(s.errores)}" if s.error else "") for s in b.bus.salud())


# ---------------------------------------------------------------------------- comandos
def cmd_salud(a):
    b = crear(a)
    for s in b.bus.salud():
        print(f"  {s.id:2d} {NOMBRES[s.id]:7s} {s.temp_c:3d} °C  {s.volt_v:4.1f} V  carga {s.carga_pct:+5.1f} %  "
              f"{'ERROR: ' + ', '.join(s.errores) if s.error else 'ok'}")
    ok, motivo = b.bus.puede_trabajar()
    print("Puede trabajar:", "SÍ" if ok else f"NO ({motivo})")
    ang = b.bus.angulos()
    p, pitch = b.directa_robot(ang)
    print("Ángulos: " + "  ".join(f"{NOMBRES[s]} {math.degrees(v):+6.1f}°" for s, v in ang.items()))
    print(f"Garra en el robot: x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f} m, inclinación {pitch:.0f}°")
    b.cerrar()


def _txt(scr, y, x, texto):
    """addstr que no se cae si la terminal es angosta o baja."""
    alto, ancho = scr.getmaxyx()
    if y < alto - 1:
        try:
            scr.addstr(y, x, texto[:max(0, ancho - x - 1)])
        except curses.error:
            pass


def _pantalla_leer(scr, b):
    curses.curs_set(0)
    scr.nodelay(True)
    msg = "G = grabar la postura actual con un nombre | Q = salir"
    while True:
        ang = b.bus.angulos()
        p, pitch = b.directa_robot(ang)
        esf = b.esfuerzo_pct(ang)
        scr.erase()
        _txt(scr, 0, 0, "BRAZO SIN TORQUE: muévelo a mano (sujétalo: se cae si lo sueltas estirado)")
        for i, (s, v) in enumerate(ang.items()):
            _txt(scr, 2 + i, 2, f"{s:2d} {NOMBRES[s]:7s} {math.degrees(v):+8.1f}°")
        _txt(scr, 9, 0, f"Garra en el ROBOT: x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:.3f} m   inclinación {pitch:5.1f}° "
                         "(90 = horizontal)")
        d = math.degrees((ang[9] - math.radians(b.cb.yaw0_deg)) * b.cb.yaw_signo)
        _txt(scr, 10, 0, f"Dirección del brazo: {d:+.1f}° (0 = adelante, + = izquierda)   "
                          f"esfuerzo por gravedad: {', '.join(f'{x:.0f}%' for x in esf)}")
        _txt(scr, 12, 0, linea_salud(b))
        _txt(scr, 14, 0, msg)
        _txt(scr, 15, 0, "Posturas: " + ", ".join(sorted(b.posturas)))
        scr.refresh()
        k = scr.getch()
        if k in (ord("q"), ord("Q"), 27):
            return
        if k in (ord("g"), ord("G")):
            scr.nodelay(False)
            curses.echo()
            curses.curs_set(1)
            _txt(scr, 16, 0, "Nombres usados: trabajo, descanso, deposito_maduro, deposito_sobremaduro")
            _txt(scr, 17, 0, "Nombre: ")
            nombre = scr.getstr(17, 8, 40).decode().strip()
            curses.noecho()
            curses.curs_set(0)
            scr.nodelay(True)
            if nombre:
                post = b.grabar_postura(nombre)
                msg = f"Grabada '{nombre}': {post}  -> {b.ruta_posturas.name}"
        time.sleep(0.15)


def cmd_leer(a):
    b = crear(a)
    confirmar("El brazo queda SIN torque (cae si no lo sujetas).", a.si)
    b.bus.torque(False)
    try:
        curses.wrapper(_pantalla_leer, b)
    finally:
        b.cerrar()


def cmd_grabar(a):
    b = crear(a)
    post = b.grabar_postura(a.nombre)
    print(f"Grabada '{a.nombre}' en {b.ruta_posturas}: {post}")
    b.cerrar()


def cmd_ir(a):
    b = crear(a)
    b.bus.torque(True)
    ok = b.ir(a.nombre, a.t)
    print("OK" if ok else f"NO llegó bien: {b.ultimo_motivo}")
    print(linea_salud(b))
    print("El torque sigue activo. Para soltarlo: python3 -m tools.brazo apagar")
    b.cerrar()


def cmd_garra(a):
    b = crear(a)
    b.bus.torque(True, [4])
    if a.que in ("abrir", "cerrar"):
        ang = b.garra(abrir=a.que == "abrir")
    else:
        ang = b.garra(angulo_deg=float(a.que))
    print(f"Garra en {ang:.1f}°  (umbral 'no hay grano': > {b.cb.garra_vacia_deg:.0f}°)")
    b.cerrar()


def cmd_punto(a):
    b = crear(a)
    p = np.array([a.x, a.y, a.z])
    s = b.ik(p, a.pitch)
    if s is None:
        raise SystemExit("Fuera de alcance (prueba otro punto o --pitch).")
    ang, sol = s
    print("Ángulos: " + "  ".join(f"{NOMBRES[k]} {math.degrees(v):+.1f}°" for k, v in ang.items())
          + f"  | inclinación {math.degrees(sol.pitch):.0f}°  | esfuerzo máx {100 * np.max(np.abs(sol.tau)) / b.modelo.tau_max:.0f}%")
    confirmar("El brazo se moverá.", a.si)
    b.bus.torque(True)
    ok = b.ir_a_punto(p, a.pitch, a.t)
    real, pitch = b.directa_robot(b.bus.angulos())
    print(("OK" if ok else f"NO llegó bien: {b.ultimo_motivo}")
          + f" | garra en x={real[0]:.3f} y={real[1]:+.3f} z={real[2]:.3f} (pedido {a.x:.3f} {a.y:+.3f} {a.z:.3f})")
    print("Mide con una regla dónde quedó la garra: la diferencia corrige brazo_x/y/z o yaw0_deg.")
    b.cerrar()


def cmd_agarrar(a):
    b = crear(a)
    p = np.array([a.x, a.y, a.z])
    plan = b.plan_agarre(p)
    if plan is None:
        raise SystemExit("Fuera de alcance.")
    for nombre, ang, t in plan:
        q, pitch = b.directa_robot(ang)
        print(f"  {nombre:12s} x={q[0]:.3f} y={q[1]:+.3f} z={q[2]:.3f}  inclinación {pitch:.0f}°  ({t:.1f} s)")
    if not a.sin_deposito and "deposito_maduro" not in b.posturas:
        raise SystemExit("Falta la postura deposito_maduro (grábala con `leer` + G) o usa --sin-deposito.")
    confirmar("El brazo ejecutará la secuencia.", a.si)
    b.bus.torque(True)
    b.garra(abrir=True)
    b.ir(b.cb.postura_espera)
    ok = b.recoger(p, a.categoria, depositar=not a.sin_deposito)
    print("RECOGIDO" if ok else f"NO: {b.ultimo_motivo}")
    if ok and a.sin_deposito:
        input("Tiene el grano. ENTER para abrir la garra y volver a la postura de espera... ")
        b.garra(abrir=True)
        b.ir(b.cb.postura_espera)
    print(linea_salud(b))
    b.cerrar()


def _pantalla_teleop(scr, b):
    curses.curs_set(0)
    scr.nodelay(True)
    ang = b.bus.angulos()
    p, pitch = b.directa_robot(ang)
    obj = p.copy()
    pitch_obj = round(pitch)
    paso = 0.005
    msg = ""
    while True:
        k = scr.getch()
        mover = False
        if k in (ord("q"), ord("Q"), 27):
            return
        d = {ord("w"): (0, paso), ord("s"): (0, -paso), ord("a"): (1, paso), ord("d"): (1, -paso),
             ord("r"): (2, paso), ord("f"): (2, -paso)}
        if k in d:
            i, v = d[k]
            obj[i] += v
            mover = True
        elif k in (ord("t"), ord("g")):
            pitch_obj += -5 if k == ord("t") else 5
            mover = True
        elif k in (ord("1"), ord("2"), ord("3")):
            paso = {ord("1"): 0.002, ord("2"): 0.005, ord("3"): 0.01}[k]
        elif k == ord("o"):
            msg = f"garra {b.garra(abrir=True):.0f}°"
        elif k == ord("p"):
            msg = f"garra {b.garra(abrir=False):.0f}° (>{b.cb.garra_vacia_deg:.0f}° = vacía)"
        elif k in (ord("h"), ord("b")):
            nombre = "trabajo" if k == ord("h") else "descanso"
            b.ir(nombre)
            p, pitch = b.directa_robot(b.bus.angulos())
            obj, pitch_obj = p.copy(), round(pitch)
            msg = nombre
        elif k == ord("k"):
            msg = "agarrando aquí..."
            ok = b.recoger(obj.copy(), cancha.MADURO, depositar="deposito_maduro" in b.posturas)
            msg = "RECOGIDO" if ok else f"NO: {b.ultimo_motivo}"
            p, pitch = b.directa_robot(b.bus.angulos())
        if mover:
            s = b.ik(obj, pitch_obj)
            if s is None:
                msg = "fuera de alcance con esa inclinación"
            else:
                b.bus.mover(s[0], 0.25)
                msg = f"esfuerzo máx {100 * np.max(np.abs(s[1].tau)) / b.modelo.tau_max:.0f}%"
        ang = b.bus.angulos()
        real, pr = b.directa_robot(ang)
        scr.erase()
        _txt(scr, 0, 0, "W/S x  A/D y  R/F z  T/G inclinación  1/2/3 paso 2/5/10 mm  O/P garra  "
                         "H trabajo  B descanso  K agarrar aquí  Q salir")
        _txt(scr, 2, 0, f"Objetivo: x={obj[0]:+.3f} y={obj[1]:+.3f} z={obj[2]:.3f}  inclinación {pitch_obj}°  paso {paso*1000:.0f} mm")
        _txt(scr, 3, 0, f"Real:     x={real[0]:+.3f} y={real[1]:+.3f} z={real[2]:.3f}  inclinación {pr:.0f}°")
        _txt(scr, 4, 0, "Ángulos: " + "  ".join(f"{NOMBRES[s]} {math.degrees(v):+.0f}°" for s, v in ang.items()))
        _txt(scr, 6, 0, linea_salud(b))
        _txt(scr, 8, 0, (">> " + msg))
        _txt(scr, 9, 0, "Filas del árbol (centro del grano): baja z=0,407  media 0,557  alta 0,707 m")
        scr.refresh()
        time.sleep(0.05)


def cmd_teleop(a):
    b = crear(a)
    confirmar("El brazo se moverá con el teclado.", a.si)
    b.bus.torque(True)
    try:
        curses.wrapper(_pantalla_teleop, b)
    finally:
        print(linea_salud(b))
        print("El torque sigue activo. Para soltarlo: python3 -m tools.brazo apagar")
        b.cerrar()


def cmd_apagar(a):
    b = crear(a)
    b.bus.torque(True)
    b.garra(abrir=True)
    b.ir("descanso")
    confirmar("Se suelta el torque (sujeta el brazo si no queda apoyado).", a.si)
    b.bus.torque(False)
    print("Torque apagado.")
    b.cerrar()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default=None)
    ap.add_argument("--si", action="store_true", help="no pedir confirmación")
    comun = argparse.ArgumentParser(add_help=False)          # también se aceptan DESPUÉS del comando
    comun.add_argument("--puerto", default=argparse.SUPPRESS)
    comun.add_argument("--si", action="store_true", default=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    _add = sub.add_parser
    sub.add_parser = lambda *x, **k: _add(*x, parents=[comun], **k)
    sub.add_parser("salud").set_defaults(f=cmd_salud)
    sub.add_parser("leer").set_defaults(f=cmd_leer)
    s = sub.add_parser("grabar")
    s.add_argument("nombre")
    s.set_defaults(f=cmd_grabar)
    s = sub.add_parser("ir")
    s.add_argument("nombre")
    s.add_argument("--t", type=float, default=2.0, help="segundos")
    s.set_defaults(f=cmd_ir)
    s = sub.add_parser("garra")
    s.add_argument("que", help="abrir | cerrar | ángulo en grados")
    s.set_defaults(f=cmd_garra)
    for nombre, f in (("punto", cmd_punto), ("agarrar", cmd_agarrar)):
        s = sub.add_parser(nombre)
        s.add_argument("x", type=float)
        s.add_argument("y", type=float)
        s.add_argument("z", type=float)
        s.add_argument("--pitch", type=float, default=None, help="inclinación de la garra (90 = horizontal)")
        s.add_argument("--t", type=float, default=1.5)
        if nombre == "agarrar":
            s.add_argument("--categoria", default=cancha.MADURO, choices=(cancha.MADURO, cancha.SOBREMADURO))
            s.add_argument("--sin-deposito", action="store_true", help="queda con el grano (sin postura de depósito)")
        s.set_defaults(f=f)
    sub.add_parser("teleop").set_defaults(f=cmd_teleop)
    sub.add_parser("apagar").set_defaults(f=cmd_apagar)
    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
