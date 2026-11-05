"""Calibrar y probar las COMPUERTAS de los depósitos: Dynamixel en el bus del brazo (U2D2). Detalle: docs/mision.md.
4 nov: id 2 = maduros, id 13 = sobremaduros (antes 12/13) (config.compuertas.ids). Ángulos en grados del SERVO, 0..360.

    python3 -m tools.compuertas calibrar                 # GUIADO, las dos (o: calibrar maduros): cerrada, abierta,
                                                         #   límites, paso por 0° y prueba con torque
    python3 -m tools.compuertas leer                     # ángulo, modo, límites y calibración de las dos
    python3 -m tools.compuertas leer --seguir            # ídem cada 0,2 s, para moverlas a mano (Ctrl+C sale)
    python3 -m tools.compuertas soltar                   # sin torque: se mueven A MANO
    python3 -m tools.compuertas grabar maduros cerrada   # el ángulo ACTUAL pasa a ser "cerrada" (o "abierta")
    python3 -m tools.compuertas limites maduros 150 250  # rango permitido: se graba en el servo y no pasa de ahí
    python3 -m tools.compuertas mover maduros 200        # con torque, a ese ángulo (dentro de los límites)
    python3 -m tools.compuertas probar                   # abrir, esperar t_vaciado_s y cerrar (las dos, o una)
La compuerta se nombra maduros / sobremaduros, 0 / 1, o por su id (12 / 13).

A mano (lo mismo que hace `calibrar`): 1) soltar  2) llevar cada compuerta A MANO a cerrada y grabar; después a
abierta y grabar  3) limites unos grados más allá de cerrada y de abierta  4) probar. Se graba en pi/robot_local.json
(sección "compuertas"): copiarlo también a pi/cafebot/config.py (ConfigCompuertas), porque robot_local.json se pierde
si se pierde la SD.
Sentido: no se configura; se graban los ángulos reales y el servo va de uno al otro. Si entre cerrada y abierta se
pasa por 0°/360°, el servo daría la vuelta larga: la herramienta NO guarda esa compuerta y pide montar el cuerno
girado (que trabaje cerca de 180°) y volver a calibrar.
"""
from __future__ import annotations

import argparse
import threading
import time

from cafebot.arm.compuertas import NOMBRES, CompuertasDXL, abrir_bus, cruza_cero, diferencia, indice, limites_con_margen
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
    des = f", cero corrido {d['desfase']:+.0f}°" if abs(d["desfase"]) > 0.05 else ""
    return (f"{NOMBRES[k]:13s} id {d['id']} ({d['modelo']}): {d['angulo']:6.1f}° | torque {'sí' if d['torque'] else 'no'}"
            f", carga {abs(d['carga']):.0f} % | {modo}, límites del servo {lo:.0f}..{hi:.0f}°{des} | {cal} | "
            f"{d['temp_c']} °C{err}")


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


def cruce_cero(cfg, k) -> bool:
    """Si entre cerrada y abierta se pasa por 0°/360°, el servo (modo posición) daría la vuelta LARGA y rompería la
    compuerta: se avisa y se borra lo grabado de esa compuerta (sin calibrar no se mueve). True si pasaba."""
    cc = cfg.compuertas
    c, o = cc.cerrada_deg[k], cc.abierta_deg[k]
    if c is None or o is None or not cruza_cero(c, o):
        return False
    print(f"  ¡NO SIRVE ASÍ! Entre cerrada ({c}°) y abierta ({o}°) se pasa por 0°/360°: el servo no cruza el 0 e iría "
          f"por el otro lado ({360 - abs(diferencia(c, o)):.0f}° en vez de {abs(diferencia(c, o)):.0f}°).")
    print("  Soltar el tornillo del cuerno (la pieza entre el servo y la compuerta) y montarlo girado para que, con la "
          "compuerta a medio camino, el servo lea cerca de 180° (leer --seguir). Después volver a calibrar.")
    for campo in ("cerrada_deg", "abierta_deg"):
        guardar(cfg, campo, k, None)
    return True


def cmd_grabar(a, cfg, comp):
    (k,) = cual(cfg, a.compuerta)
    g = round(comp.angulo(k), 1)
    guardar(cfg, "abierta_deg" if a.que == "abierta" else "cerrada_deg", k, g)
    if cruce_cero(cfg, k):
        return
    lim = comp.limites(k)
    if lim is not None and not min(lim) <= g <= max(lim):
        print(f"  ¡OJO! {g}° está fuera de los límites {lim}: cambiarlos con  limites {NOMBRES[k]} MIN MAX")


def pedir_angulo(comp, k, que: str) -> float:
    """Muestra el ángulo en vivo mientras se mueve la compuerta a mano; ENTER lo graba."""
    print(f"  Lleva la compuerta a {que} y aprieta ENTER:")
    fin = threading.Event()

    def mostrar():
        while not fin.is_set():
            try:
                print(f"\r    ángulo ahora: {comp.angulo(k):6.1f}°   ", end="", flush=True)
            except IOError:
                pass
            fin.wait(0.25)
    h = threading.Thread(target=mostrar, daemon=True)
    h.start()
    try:
        input()
    finally:
        fin.set()
        h.join(1.0)
    g = round(comp.angulo(k), 1)
    print(f"    -> {g}°")
    return g


def preguntar(txt: str) -> bool:
    return input(f"  {txt} [s/N] ").strip().lower() in ("s", "si", "sí", "y")


def probar_una(cfg, comp, k, t: float = 1.0) -> bool:
    """Con torque y despacio: a cerrada, abrir, esperar t_vaciado_s, cerrar. Avisa si queda empujando un tope."""
    cc = cfg.compuertas
    comp.configurar(k)
    comp.torque(k, True)
    ok = comp.mover(k, float(cc.cerrada_deg[k]), t)
    print(f"    cerrada: pedí {cc.cerrada_deg[k]}°, quedó en {ok:.1f}°")
    llego = comp.mover(k, float(cc.abierta_deg[k]), t)
    print(f"    abierta: pedí {cc.abierta_deg[k]}°, quedó en {llego:.1f}°")
    time.sleep(cfg.mision.t_vaciado_s)
    vuelta = comp.mover(k, float(cc.cerrada_deg[k]), t)
    time.sleep(0.3)
    carga = comp.carga(k)
    print(f"    cerrada otra vez: {vuelta:.1f}° | carga sujetándola {abs(carga):.0f} %")
    bien = abs(llego - cc.abierta_deg[k]) < 5 and abs(vuelta - cc.cerrada_deg[k]) < 5
    if not bien:
        print("    ¡OJO! no llegó a donde debía: algo la traba o los límites la cortan (leer, limites)")
    if abs(carga) > 35:
        sentido = 1 if diferencia(cc.cerrada_deg[k], cc.abierta_deg[k]) > 0 else -1
        print(f"    ¡OJO! cerrada empuja contra algo ({abs(carga):.0f} %): se calienta. Correr 'cerrada' 2–3° hacia "
              f"abierta, p. ej. {round(cc.cerrada_deg[k] + 2.5 * sentido, 1)}° (grabar a mano o volver a calibrar)")
    return bien


def cmd_calibrar(a, cfg, comp):
    for k in cual(cfg, a.compuerta):
        print(f"\n=== Compuerta de {NOMBRES[k]} (id {comp.ids[k]}) ===")
        if not comp.responde(k):
            print(f"  NO RESPONDE: revisar el cable y el id (python3 -m tools.brazo escanear)")
            continue
        comp.configurar(k)                   # modo posición: el ángulo se lee dentro de una vuelta
        comp.torque(k, False)
        print("  Sin torque: se mueve a mano. Primero CERRADA, después ABIERTA.")
        c = pedir_angulo(comp, k, "CERRADA (tapando la salida, sin apretarla contra un tope)")
        o = pedir_angulo(comp, k, "ABIERTA (del todo: que los granos salgan solos)")
        if abs(diferencia(c, o)) < 10:
            print(f"  ¡OJO! abierta y cerrada están a {abs(diferencia(c, o)):.1f}°: ¿se movió de verdad? Repetir.")
            continue
        guardar(cfg, "cerrada_deg", k, c)
        guardar(cfg, "abierta_deg", k, o)
        if cruce_cero(cfg, k):
            continue
        lo, hi = limites_con_margen(c, o, a.margen)
        guardar(cfg, "limites_deg", k, [round(lo, 1), round(hi, 1)])
        comp.configurar(k)                   # los límites quedan en la EEPROM del servo
        print(f"  Límites {lo:.1f}..{hi:.1f}° grabados en el servo ({a.margen:.0f}° más allá de cada extremo)")
        if preguntar("¿Pruebo abrir y cerrar con torque, despacio?"):
            probar_una(cfg, comp, k)
    cc = cfg.compuertas
    fmt = lambda t: "(" + ", ".join("None" if v is None else str(tuple(v) if isinstance(v, list) else v) for v in t) + ")"
    print("\nPara pi/cafebot/config.py (ConfigCompuertas), que robot_local.json se pierde con la SD:")
    print(f"    abierta_deg: tuple = {fmt(cc.abierta_deg)}")
    print(f"    cerrada_deg: tuple = {fmt(cc.cerrada_deg)}")
    print(f"    limites_deg: tuple = {fmt(cc.limites_deg)}")


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
            print(f"{NOMBRES[k]}: sin calibrar (python3 -m tools.compuertas calibrar {NOMBRES[k]})")
            continue
        print(f"{NOMBRES[k]}: a cerrada, abro, espero {cfg.mision.t_vaciado_s} s y cierro")
        probar_una(cfg, comp, k, t=cfg.compuertas.t_mov_s)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default=None, help="puerto del U2D2 (por defecto el del perfil o la config)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("calibrar", help="guiado: cerrada, abierta, límites y prueba")
    s.add_argument("compuerta", nargs="?", default=None)
    s.add_argument("--margen", type=float, default=5.0, help="grados de límite más allá de abierta y cerrada")
    s.set_defaults(f=cmd_calibrar)
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
