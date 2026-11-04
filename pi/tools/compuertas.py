"""Calibrar y probar las COMPUERTAS de los depósitos: Dynamixel en el bus del brazo (U2D2). Detalle: docs/mision.md.
4 nov: id 12 = maduros, id 13 = sobremaduros (config.compuertas.ids). Ángulos en grados del SERVO, 0..360.

    python3 -m tools.compuertas leer                     # ángulo, modo, límites y calibración de las dos
    python3 -m tools.compuertas leer --seguir            # ídem cada 0,2 s, para moverlas a mano (Ctrl+C sale)
    python3 -m tools.compuertas soltar                   # sin torque: se mueven A MANO
    python3 -m tools.compuertas grabar maduros cerrada   # el ángulo ACTUAL pasa a ser "cerrada" (o "abierta")
    python3 -m tools.compuertas limites maduros 150 250  # rango permitido: se graba en el servo y no pasa de ahí
    python3 -m tools.compuertas mover maduros 200        # con torque, a ese ángulo (dentro de los límites)
    python3 -m tools.compuertas probar                   # abrir, esperar t_vaciado_s y cerrar (las dos, o una)
La compuerta se nombra maduros / sobremaduros, 0 / 1, o por su id (12 / 13).

Pasos: 1) soltar  2) llevar cada compuerta A MANO a cerrada y grabar; después a abierta y grabar  3) limites unos
grados más allá de cerrada y de abierta  4) probar. Se graba en pi/robot_local.json (sección "compuertas"): copiarlo
también a pi/cafebot/config.py (ConfigCompuertas), porque robot_local.json se pierde si se pierde la SD.
"""
from __future__ import annotations

import argparse
import time

from cafebot.arm.compuertas import NOMBRES, CompuertasDXL, abrir_bus, indice
from cafebot.config import cargar, guardar_local


def cual(cfg, texto) -> list:
    """Índices (0 maduros, 1 sobremaduros) que nombra `texto` (vacío = las dos)."""
    if texto in (None, "", "todas", "las-dos"):
        return [0, 1]
    if str(texto).isdigit() and int(texto) in tuple(cfg.compuertas.ids):
        return [tuple(cfg.compuertas.ids).index(int(texto))]
    return [indice(texto)]


def guardar(cfg, campo: str, k: int, valor):
    lista = [list(v) if isinstance(v, (list, tuple)) else v for v in getattr(cfg.compuertas, campo)]
    lista[k] = valor
    setattr(cfg.compuertas, campo, tuple(lista))
    ruta = guardar_local("compuertas", {campo: lista})
    print(f"Guardado en {ruta}: compuertas.{campo} = {lista}")
    print("  Copiarlo también a pi/cafebot/config.py (ConfigCompuertas): robot_local.json se pierde con la SD.")


def texto_estado(comp: CompuertasDXL, k: int) -> str:
    cc = comp.cc
    if not comp.responde(k):
        return f"{NOMBRES[k]:13s} id {comp.ids[k]}: NO RESPONDE (python3 -m tools.brazo escanear)"
    d = comp.leer(k)
    modo = {3: "posición", 4: "posición extendida"}.get(d["modo"], f"modo {d['modo']}")
    lo, hi = d["limites_servo"]
    cal = (f"cerrada {cc.cerrada_deg[k]}°, abierta {cc.abierta_deg[k]}°" if comp.calibrada(k)
           else f"SIN CALIBRAR (cerrada {cc.cerrada_deg[k]}, abierta {cc.abierta_deg[k]})")
    err = f" | ERROR de hardware {d['error']}" if d["error"] else ""
    return (f"{NOMBRES[k]:13s} id {d['id']} ({d['modelo']}): {d['angulo']:6.1f}° | torque {'sí' if d['torque'] else 'no'}"
            f" | {modo}, límites del servo {lo:.0f}..{hi:.0f}° | {cal} | {d['temp_c']} °C{err}")


def cmd_leer(a, cfg, comp):
    if not a.seguir:
        for k in cual(cfg, a.compuerta):
            print(texto_estado(comp, k))
        return
    print("Ángulos cada 0,2 s (Ctrl+C para salir):")
    try:
        while True:
            print("   ".join(f"{NOMBRES[k]} {comp.angulo(k):6.1f}°" for k in cual(cfg, a.compuerta)), end="\r", flush=True)
            time.sleep(0.2)
    except KeyboardInterrupt:
        print()


def cmd_soltar(a, cfg, comp):
    for k in cual(cfg, a.compuerta):
        comp.torque(k, False)
        print(f"{NOMBRES[k]}: sin torque (se mueve a mano). Para sujetarla otra vez: mover o probar.")


def cmd_grabar(a, cfg, comp):
    (k,) = cual(cfg, a.compuerta)
    g = round(comp.angulo(k), 1)
    guardar(cfg, "abierta_deg" if a.que == "abierta" else "cerrada_deg", k, g)
    lim = comp.limites(k)
    if lim is not None and not min(lim) <= g <= max(lim):
        print(f"  ¡OJO! {g}° está fuera de los límites {lim}: cambiarlos con  limites {NOMBRES[k]} MIN MAX")


def cmd_limites(a, cfg, comp):
    (k,) = cual(cfg, a.compuerta)
    lo, hi = sorted((a.min, a.max))
    if not 0 <= lo < hi <= 360:
        raise SystemExit("límites: 0 <= mín < máx <= 360")
    for que, g in (("cerrada", cfg.compuertas.cerrada_deg[k]), ("abierta", cfg.compuertas.abierta_deg[k])):
        if g is not None and not lo <= g <= hi:
            print(f"  ¡OJO! {que} = {g}° queda fuera de {lo}..{hi}°")
    tenia_torque = comp.leer(k)["torque"]
    guardar(cfg, "limites_deg", k, [lo, hi])
    comp.configurar(k)                                    # escribe los límites en la EEPROM (sin torque)
    if tenia_torque:
        comp.torque(k, True)
    print(f"  {NOMBRES[k]}: el servo ya no pasa de {lo}..{hi}°")


def cmd_mover(a, cfg, comp):
    (k,) = cual(cfg, a.compuerta)
    comp.configurar(k)
    comp.torque(k, True)
    llego = comp.mover(k, a.grados, a.t)
    print(f"{NOMBRES[k]}: pedí {a.grados:.1f}°, quedó en {llego:.1f}°")


def cmd_probar(a, cfg, comp):
    for k in cual(cfg, a.compuerta):
        if not comp.calibrada(k):
            print(f"{NOMBRES[k]}: sin calibrar (grabar abierta y cerrada primero)")
            continue
        comp.configurar(k)
        comp.torque(k, True)
        ok = comp.cerrar(k)
        print(f"{NOMBRES[k]}: abro ({cfg.compuertas.abierta_deg[k]}°), espero {cfg.mision.t_vaciado_s} s y cierro "
              f"({cfg.compuertas.cerrada_deg[k]}°)")
        ok = comp.abrir(k) and ok
        time.sleep(cfg.mision.t_vaciado_s)
        ok = comp.cerrar(k) and ok
        print(f"   {'bien' if ok else 'NO llegó a donde debía (ver arriba)'}: quedó en {comp.angulo(k):.1f}°")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default=None, help="puerto del U2D2 (por defecto el del perfil o la config)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("leer")
    s.add_argument("compuerta", nargs="?", default=None)
    s.add_argument("--seguir", action="store_true")
    s.set_defaults(f=cmd_leer)
    s = sub.add_parser("soltar")
    s.add_argument("compuerta", nargs="?", default=None)
    s.set_defaults(f=cmd_soltar)
    s = sub.add_parser("grabar")
    s.add_argument("compuerta")
    s.add_argument("que", choices=("abierta", "cerrada"))
    s.set_defaults(f=cmd_grabar)
    s = sub.add_parser("limites")
    s.add_argument("compuerta")
    s.add_argument("min", type=float)
    s.add_argument("max", type=float)
    s.set_defaults(f=cmd_limites)
    s = sub.add_parser("mover")
    s.add_argument("compuerta")
    s.add_argument("grados", type=float)
    s.add_argument("--t", type=float, default=None, help="segundos (por defecto compuertas.t_mov_s)")
    s.set_defaults(f=cmd_mover)
    s = sub.add_parser("probar")
    s.add_argument("compuerta", nargs="?", default=None)
    s.set_defaults(f=cmd_probar)
    a = ap.parse_args()
    cfg = cargar()
    bus = abrir_bus(cfg, puerto=a.puerto)
    try:
        a.f(a, cfg, CompuertasDXL(bus, cfg.compuertas))
    finally:
        bus.cerrar()


if __name__ == "__main__":
    main()
