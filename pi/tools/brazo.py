"""Calibrar y probar el brazo REAL (XL430 por U2D2) paso a paso, SIN cámara. Detalle: docs/brazo.md.

Cada brazo tiene su PERFIL: pi/brazos/<nombre>.json (ids, ceros, sentidos, límites, posturas, historial).
    python3 -m tools.brazo escanear                  # 1. qué servos hay en el bus (ids, velocidad, firmware)
    python3 -m tools.brazo identificar               #    solo si los ids no calzan: LED uno por uno, ¿cuál es?
    python3 -m tools.brazo salud                     # 2. °C, V, errores y ángulos
    python3 -m tools.brazo cero                      # 3. SIN torque: brazo recto hacia arriba -> 0° de cada una
    python3 -m tools.brazo verificar                 # 4. sentido de cada una (la mueves tú, las demás sostienen)
    python3 -m tools.brazo limites                   # 5. SIN torque: recorrer el rango seguro de cada una
    python3 -m tools.brazo articulacion hombro 10    # 6. primer movimiento con torque, de a una (--rel: relativo)
    python3 -m tools.brazo garra calibrar            # 7. vacía / con grano -> umbral "no había grano"
    python3 -m tools.brazo leer                      # 8. SIN torque: grabar posturas (tecla G)
    python3 -m tools.brazo punto 0.20 0.0 0.45       # 9. la garra a (x, y, z) del ROBOT (m): medir con regla
    python3 -m tools.brazo fila media                # 10. agarre completo con coordenadas fijas (soporte de prueba)
Además: perfil (ver todo + alcance), ir <postura>, garra abrir|cerrar|<°>, teleop, agarrar X Y Z,
        grabar <nombre>, cambiar-id VIEJO NUEVO (servo nuevo de fábrica), apagar (descanso y soltar el torque).
Opciones (antes o después del comando): --perfil NOMBRE (otro brazo)   --banco (brazo fuera del robot: las
coordenadas se miden desde SU base)   --puerto /dev/ttyUSBx   --si (no preguntar).

Puerto: el del perfil o el de la config (/dev/cafebot-u2d2 con las reglas udev); si no existe, busca el U2D2 (FTDI).
Sistema del robot: x adelante, y izquierda, z desde el SUELO. Base del brazo: ConfigMision.brazo_x/y/z.
"""
from __future__ import annotations

import argparse
import curses
import glob
import os
import time

import numpy as np

from cafebot.arm.brazo import MOSTRAR, SENTIDO_NEGATIVO, SENTIDO_POSITIVO, TODAS, nombre_articulacion
from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha

MARGEN_LIMITE_DEG = 3.0      # los límites quedan esto ADENTRO de lo recorrido a mano
RANGO_MIN_DEG = 10.0         # una articulación que se movió menos que esto conserva sus límites
UMBRAL_SENTIDO_DEG = 15.0    # verificar: cuánto hay que mover la articulación para decidir el sentido
CERO_TOLERANCIA_DEG = 5.0    # "recto" que lee más que esto: revisar el cero
POSTURAS_MISION = ("descanso", "deposito_maduro", "deposito_sobremaduro")


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


def preguntar(txt: str, si: bool = False) -> bool:
    if si:
        return True
    return input(f"{txt} [s/N] ").strip().lower() in ("s", "si", "sí", "y")


def _lista_art(texto) -> list:
    if not texto:
        return []
    try:
        return [nombre_articulacion(t) for t in texto.split(",") if t.strip()]
    except ValueError as e:
        raise SystemExit(str(e))


def crear(a, conectar=True, puerto=True):
    """BrazoXL430 con el perfil pedido (--perfil) y, si `conectar`, el bus abierto y configurado.
    puerto=False: solo el perfil (no hace falta el U2D2 conectado)."""
    from cafebot.arm.brazo import BrazoXL430
    cfg = cargar()
    if getattr(a, "perfil", None):
        cfg.brazo.perfil = a.perfil
    try:
        b = BrazoXL430(cfg, a.puerto, banco=a.banco)
    except ValueError as e:
        raise SystemExit(str(e))
    if not a.puerto and (puerto or conectar):
        b.puerto = buscar_u2d2(b.puerto)
    p = b.perfil
    print(f"Brazo '{p.nombre}': {p.ruta}" + ("" if p.ruta.exists() else "  (aún sin archivo: valores del código antiguo)")
          + ("  | BANCO: coordenadas desde la base del brazo" if b.banco else ""))
    if not conectar:
        return b
    print(f"U2D2 en {b.puerto} a {p.baudios} baudios. Conectando y configurando...")
    try:
        vivos = b.conectar()
    except (IOError, OSError) as e:
        raise SystemExit(f"{e}")
    print("Servos: " + ", ".join(f"{MOSTRAR[n]} {b.id[n]} (fw {vivos[b.id[n]][1]})" for n in TODAS))
    viejo = [s for s, (_, fw) in vivos.items() if fw < 42]
    if viejo:
        print(f"Firmware < 42 en {viejo}: se usa perfil por velocidad (actualizar con Dynamixel Wizard si se puede).")
    return b


def linea_salud(b) -> str:
    return "  ".join(f"{b.mostrar(s.id)}:{s.temp_c}°C {s.volt_v:.1f}V carga{s.carga_pct:+.0f}%"
                     + (f" ERROR {','.join(s.errores)}" if s.error else "") for s in b.bus.salud())


def marco(b) -> str:
    return "desde la BASE del brazo" if b.banco else "en el ROBOT"


def sostener_o_soltar(b):
    """Al terminar algo que dejó el brazo SIN torque: que no se caiga al cerrar el programa."""
    r = input("ENTER = el brazo se sostiene solo donde está (torque)  |  S + ENTER = dejarlo suelto: ").strip().lower()
    if r != "s":
        b.bus.torque(True)
        print("Torque activo (para soltarlo: python3 -m tools.brazo apagar).")


# ---------------------------------------------------------------------------- cálculos (sin bus: se prueban)
def analizar_escaneo(perfil, hallados: dict) -> tuple[list, int | None]:
    """Compara lo que respondió en el bus ({baudios: {id: (modelo, firmware)}}) con el perfil.
    Devuelve (líneas para mostrar, baudios a los que conviene cambiar el perfil o None)."""
    from cafebot.arm import dxl
    ids = perfil.ids
    en_bus = hallados.get(perfil.baudios, {})
    lineas = []
    for n in TODAS:
        sid = ids[n]
        if sid in en_bus:
            estado = "ok"
        else:
            otras = [bd for bd, s in sorted(hallados.items()) if sid in s]
            estado = f"responde a {otras[0]} baudios (el perfil usa {perfil.baudios})" if otras else "NO RESPONDE"
        lineas.append(f"  {MOSTRAR[n]:7s} id {sid:3d}: {estado}")
    for bd, s in sorted(hallados.items()):
        for sid in sorted(s):
            if sid not in ids.values():
                extra = (" (servo de FÁBRICA: cambiar-id 1 <id> con SOLO ese servo conectado)" if (sid, bd) == (1, 57_600)
                         else " (¿servo cambiado o ids cruzados? identificar)")
                lineas.append(f"  id {sid} a {bd} baudios: no es de este perfil{extra}")
    viejos = sorted(sid for s in hallados.values() for sid, (_, fw) in s.items() if fw < dxl.FIRMWARE_PERFIL_TIEMPO)
    if viejos:
        lineas.append(f"  firmware < 42 en {viejos}: se usa perfil por velocidad (actualizar con Dynamixel Wizard)")
    sugerida = None
    if not all(ids[n] in en_bus for n in TODAS):
        for bd, s in sorted(hallados.items()):
            if bd != perfil.baudios and all(ids[n] in s for n in TODAS):
                sugerida = bd
    return lineas, sugerida


def esperar_movimiento(leer, n, umbral=UMBRAL_SENTIDO_DEG, timeout_s=30.0, mostrar=None, periodo=0.05):
    """Espera a que la articulación `n` se mueva más de `umbral` grados (leer() -> {articulación: grados}).
    Devuelve (lo que se movió n, {otra: lo que se movió} de las que también se movieron > 4°) o None."""
    inicio = leer()
    fin = time.monotonic() + timeout_s
    while time.monotonic() < fin:
        ahora = leer()
        d = ahora[n] - inicio[n]
        if mostrar:
            mostrar(f"{MOSTRAR[n]}: {ahora[n]:+7.1f}°  (movido {d:+6.1f}°)")
        if abs(d) >= umbral:
            otras = {k: ahora[k] - inicio[k] for k in ahora if k != n and abs(ahora[k] - inicio[k]) > 4}
            return d, otras
        time.sleep(periodo)
    return None


def limites_desde_recorrido(visto: dict, margen=MARGEN_LIMITE_DEG, rango_min=RANGO_MIN_DEG) -> tuple[dict, list]:
    """{articulación: (mín, máx) recorrido a mano} -> ({articulación: (mín, máx) a guardar}, sin cambio)."""
    nuevos, sin_cambio = {}, []
    for n, (lo, hi) in visto.items():
        if hi - lo < rango_min:
            sin_cambio.append(n)
        else:
            nuevos[n] = (lo + margen, hi - margen)
    return nuevos, sin_cambio


def sugerir_garra(vacia: float, con_grano: float, cb) -> tuple[dict, list]:
    """Ángulos medidos al cerrar la garra vacía y con un grano -> (cambios para ConfigBrazo, avisos)."""
    cambios, avisos = {}, []
    if con_grano >= cb.garra_cerrada_deg - 3:
        avisos.append(f"con el grano cerró hasta {con_grano:.0f}° (casi del todo): los dedos no lo aprietan. "
                      "Revisar los dedos o subir brazo.garra_cerrada_deg")
        return cambios, avisos
    if vacia - con_grano < 8:
        avisos.append(f"vacía ({vacia:.0f}°) y con grano ({con_grano:.0f}°) quedan muy cerca: no se distinguiría "
                      "bien si tomó el grano")
        return cambios, avisos
    cambios["garra_vacia_deg"] = round((vacia + con_grano) / 2, 1)
    if vacia < cb.garra_cerrada_deg - 5:
        avisos.append(f"vacía se detiene en {vacia:.0f}°, antes de los {cb.garra_cerrada_deg:.0f}° pedidos: los dedos "
                      "chocan entre sí y el servo empuja contra sí mismo")
        cambios["garra_cerrada_deg"] = round(vacia + 2, 1)
    return cambios, avisos


def punto_fila(cfg, fila: str, lateral: float = 0.0, holgura: float | None = None, banco: bool = False) -> np.ndarray:
    """Centro del grano de un soporte de la fila `fila` como lo deja la misión frente al árbol: cara del tablero
    a `holgura` del frente del robot (mision.holgura_agarre_m), `lateral` m a la izquierda del eje del brazo.
    En el sistema del robot, o desde la base del brazo si `banco`."""
    cm = cfg.mision
    h = cm.holgura_agarre_m if holgura is None else holgura
    p = np.array([cfg.geometria.largo / 2 + h - cancha.GRANO_DELANTE_M, cm.brazo_y + lateral,
                  cancha.ALTURA_SOPORTE[fila] + cancha.CENTRO_SOBRE_SOPORTE])
    return p - np.array([cm.brazo_x, cm.brazo_y, cm.brazo_z]) if banco else p


def instrucciones_fila(cfg, fila, lateral, holgura, p, banco) -> str:
    cm = cfg.mision
    h = cm.holgura_agarre_m if holgura is None else holgura
    soporte = cancha.ALTURA_SOPORTE[fila]
    lado = f"{100 * abs(lateral):.1f} cm a la {'izquierda' if lateral > 0 else 'derecha'} del" if lateral else "frente al"
    if banco:
        return (f"Montaje en el BANCO, fila {fila.upper()} (lo mismo que verá el brazo montado en el robot):\n"
                f"  · Cara del tablero (o del soporte de prueba) a {100 * (cfg.geometria.largo / 2 + h - cm.brazo_x):.1f} cm "
                f"del eje de la base, mirándola de frente.\n"
                f"  · Cara de arriba del soporte a {100 * (soporte - cm.brazo_z):.1f} cm SOBRE el plano de la base; "
                f"el grano centrado {lado} eje del brazo.\n"
                f"  · Centro del grano: x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f} m desde la base.")
    return (f"Montaje para la fila {fila.upper()} (como la deja la misión frente al árbol):\n"
            f"  · Cara del tablero (o del soporte de prueba) PARALELA al frente del robot, a {100 * h:.1f} cm de él.\n"
            f"  · Cara de arriba del soporte a {100 * soporte:.1f} cm del SUELO; el grano centrado {lado} eje del brazo.\n"
            f"  · Centro del grano: x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f} m en el robot "
            f"({100 * cancha.GRANO_DELANTE_M:.1f} cm delante de la cara: GRANO_DELANTE_M).")


def informe_alcance(b, holgura=None) -> list:
    """¿Alcanza cada fila frente al árbol (de frente y con ±6 cm de lateral)? Con los límites del perfil."""
    out = []
    for fila in cancha.FILAS:
        partes = []
        for lat in (0.0, 0.06, -0.06):
            plan = b.plan_agarre(punto_fila(b.cfg, fila, lat, holgura, b.banco))
            partes.append(f"{100 * lat:+3.0f} cm " + (f"sí ({b.directa_robot(plan[2][1])[1]:.0f}°)" if plan else "NO"))
        out.append(f"    fila {fila:5s}  " + "   ".join(partes))
    return out


# ---------------------------------------------------------------------------- pantallas (curses)
def _txt(scr, y, x, texto):
    """addstr que no se cae si la terminal es angosta o baja."""
    alto, ancho = scr.getmaxyx()
    if y < alto - 1:
        try:
            scr.addstr(y, x, texto[:max(0, ancho - x - 1)])
        except curses.error:
            pass


def _tecla_enter(k) -> bool:
    return k in (10, 13, curses.KEY_ENTER)


def _pantalla_cero(scr, b, arts):
    curses.curs_set(0)
    scr.nodelay(True)
    while True:
        pasos, ang = b.pasos_nombre(), b.angulos_nombre()
        scr.erase()
        _txt(scr, 0, 0, f"CERO del brazo '{b.perfil.nombre}': SIN torque en {', '.join(MOSTRAR[n] for n in arts)} (sujétalo)")
        _txt(scr, 2, 0, "Pon el brazo RECTO HACIA ARRIBA: cada tramo alineado con el anterior (usa una regla o escuadra),")
        _txt(scr, 3, 0, "la base mirando hacia ADELANTE del robot y la garra ABIERTA del todo.")
        _txt(scr, 5, 2, "articulación  id    pasos   ángulo con el cero actual")
        for i, n in enumerate(arts):
            aviso = "   <- cambia mucho: ¿está recto?" if abs(ang[n]) > 20 else ""
            _txt(scr, 6 + i, 2, f"{MOSTRAR[n]:12s} {b.id[n]:3d} {pasos[n]:8d}   {ang[n]:+8.1f}°{aviso}")
        _txt(scr, 13, 0, "ENTER = fijar el cero en esta posición     Q = cancelar (no cambia nada)")
        scr.refresh()
        k = scr.getch()
        if _tecla_enter(k):
            return True
        if k in (ord("q"), ord("Q"), 27):
            return False
        time.sleep(0.1)


def _pantalla_limites(scr, b):
    curses.curs_set(0)
    scr.nodelay(True)
    visto = {}
    while True:
        ang = b.angulos_nombre()
        for n, v in ang.items():
            lo, hi = visto.get(n, (v, v))
            visto[n] = (min(lo, v), max(hi, v))
        scr.erase()
        _txt(scr, 0, 0, f"LÍMITES del brazo '{b.perfil.nombre}': SIN torque (sujétalo)")
        _txt(scr, 1, 0, "Mueve CADA articulación por todo su recorrido SEGURO, con el resto del brazo más o menos recto:")
        _txt(scr, 2, 0, "hasta justo antes de chocar con el robot u otra pieza, o de tirar de un cable.")
        _txt(scr, 3, 0, f"Se guardan {MARGEN_LIMITE_DEG:.0f}° hacia adentro. Las que muevas menos de {RANGO_MIN_DEG:.0f}° "
                        "conservan sus límites.")
        _txt(scr, 5, 2, "articulación     ahora      recorrido visto        límites del perfil")
        for i, n in enumerate(TODAS):
            lo, hi = visto[n]
            a = b.perfil.art[n]
            _txt(scr, 6 + i, 2, f"{MOSTRAR[n]:12s} {ang[n]:+8.1f}°   {lo:+7.1f} .. {hi:+7.1f}°     "
                                f"{a['min_deg']:+6.0f} .. {a['max_deg']:+6.0f}°")
        _txt(scr, 13, 0, "ENTER = terminar (muestra lo que se guardaría)    R = empezar de nuevo    Q = cancelar")
        scr.refresh()
        k = scr.getch()
        if _tecla_enter(k):
            return visto
        if k in (ord("r"), ord("R")):
            visto = {}
        if k in (ord("q"), ord("Q"), 27):
            return None
        time.sleep(0.05)


def _pantalla_leer(scr, b):
    curses.curs_set(0)
    scr.nodelay(True)
    msg = "G = grabar la postura actual con un nombre | Q = salir"
    while True:
        ang = b.bus.angulos()
        ang_n, pasos = b.angulos_nombre(), b.pasos_nombre()
        p, pitch = b.directa_robot(ang)
        esf = b.esfuerzo_pct(ang)
        scr.erase()
        _txt(scr, 0, 0, f"BRAZO '{b.perfil.nombre}' SIN TORQUE: muévelo a mano (sujétalo: se cae si lo sueltas estirado)")
        _txt(scr, 1, 2, "articulación  id    pasos    ángulo    límites")
        for i, n in enumerate(TODAS):
            a = b.perfil.art[n]
            fuera = "  <- FUERA de los límites" if not a["min_deg"] <= ang_n[n] <= a["max_deg"] else ""
            _txt(scr, 2 + i, 2, f"{MOSTRAR[n]:12s} {b.id[n]:3d} {pasos[n]:8d} {ang_n[n]:+8.1f}°   "
                                f"{a['min_deg']:+5.0f}..{a['max_deg']:+4.0f}°{fuera}")
        _txt(scr, 9, 0, f"Garra {marco(b)}: x={p[0]:+.3f}  y={p[1]:+.3f}  z={p[2]:.3f} m   inclinación {pitch:5.1f}° "
                         "(90 = horizontal)")
        _txt(scr, 10, 0, f"Esfuerzo por gravedad (hombro, codo1, codo2, muñeca): {', '.join(f'{x:.0f}%' for x in esf)}")
        _txt(scr, 12, 0, linea_salud(b))
        _txt(scr, 14, 0, msg)
        _txt(scr, 15, 0, "Posturas: " + ", ".join(sorted(b.posturas))
             + ("   FALTAN: " + ", ".join(x for x in POSTURAS_MISION if x not in b.posturas)
                if any(x not in b.posturas for x in POSTURAS_MISION) else ""))
        scr.refresh()
        k = scr.getch()
        if k in (ord("q"), ord("Q"), 27):
            return
        if k in (ord("g"), ord("G")):
            scr.nodelay(False)
            curses.echo()
            curses.curs_set(1)
            _txt(scr, 16, 0, "Nombres que usa la misión: " + ", ".join(POSTURAS_MISION) + " (trabajo: opcional)")
            _txt(scr, 17, 0, "Nombre: ")
            nombre = scr.getstr(17, 8, 40).decode().strip()
            curses.noecho()
            curses.curs_set(0)
            scr.nodelay(True)
            if nombre:
                post = b.grabar_postura(nombre)
                msg = (f"Grabada '{nombre}': " + " ".join(f"{MOSTRAR[k]} {v:+.0f}°" for k, v in post.items())
                       + f"  -> {b.perfil.ruta.name}")
        time.sleep(0.15)


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
            if nombre in b.posturas:
                b.ir(nombre)
                p, pitch = b.directa_robot(b.bus.angulos())
                obj, pitch_obj = p.copy(), round(pitch)
                msg = nombre
            else:
                msg = f"no hay postura '{nombre}'"
        elif k == ord("k"):
            msg = "agarrando aquí..."
            ok = b.recoger(obj.copy(), cancha.MADURO, depositar="deposito_maduro" in b.posturas)
            msg = "RECOGIDO" if ok else f"NO: {b.ultimo_motivo}"
            p, pitch = b.directa_robot(b.bus.angulos())
        if mover:
            s = b.ik(obj, pitch_obj)
            if s is None:
                msg = "fuera de alcance (o de los límites) con esa inclinación"
            else:
                b.bus.mover(s[0], 0.25)
                msg = f"esfuerzo máx {100 * np.max(np.abs(s[1].tau)) / b.modelo.tau_max:.0f}%"
        ang_n = b.angulos_nombre()
        real, pr = b.directa_robot(b.bus.angulos())
        scr.erase()
        _txt(scr, 0, 0, "W/S x  A/D y  R/F z  T/G inclinación  1/2/3 paso 2/5/10 mm  O/P garra  "
                         "H trabajo  B descanso  K agarrar aquí  Q salir")
        _txt(scr, 2, 0, f"Objetivo ({marco(b)}): x={obj[0]:+.3f} y={obj[1]:+.3f} z={obj[2]:.3f}  inclinación {pitch_obj}°  "
                         f"paso {paso * 1000:.0f} mm")
        _txt(scr, 3, 0, f"Real:     x={real[0]:+.3f} y={real[1]:+.3f} z={real[2]:.3f}  inclinación {pr:.0f}°")
        _txt(scr, 4, 0, "Ángulos: " + "  ".join(f"{MOSTRAR[n]} {ang_n[n]:+.0f}°" for n in TODAS))
        _txt(scr, 6, 0, linea_salud(b))
        _txt(scr, 8, 0, (">> " + msg))
        if b.banco:
            z = [cancha.ALTURA_SOPORTE[f] + cancha.CENTRO_SOBRE_SOPORTE - b.cm.brazo_z for f in cancha.FILAS]
            _txt(scr, 9, 0, f"Filas del árbol (centro del grano, sobre la base): baja z={z[0]:+.3f}  media {z[1]:+.3f}  "
                             f"alta {z[2]:+.3f} m")
        else:
            _txt(scr, 9, 0, "Filas del árbol (centro del grano): baja z=0,407  media 0,557  alta 0,707 m")
        scr.refresh()
        time.sleep(0.05)


# ---------------------------------------------------------------------------- comandos: calibración
def cmd_escanear(a):
    from cafebot.arm import dxl
    b = crear(a, conectar=False)
    print(f"Buscando servos en {b.puerto} (a 1 000 000, 57 600, 115 200 y 2 000 000 baudios)...")
    try:
        hallados = dxl.escanear(b.puerto)
    except (IOError, OSError) as e:
        raise SystemExit(f"{e}")
    if not hallados:
        raise SystemExit("Ningún servo responde. Revisar: alimentación de los servos (12 V), el cable TTL de 3 pines, "
                         "que el U2D2 sea este puerto (ls -l /dev/serial/by-id) y que ningún otro programa lo use.")
    for bd, servos in sorted(hallados.items()):
        print(f"  {bd:>9} baudios: " + ", ".join(f"id {sid} ({dxl.MODELOS.get(m, f'modelo {m}')}, fw {fw})"
                                                 for sid, (m, fw) in sorted(servos.items())))
    lineas, sugerida = analizar_escaneo(b.perfil, hallados)
    print(f"Perfil '{b.perfil.nombre}' ({b.perfil.baudios} baudios):")
    print("\n".join(lineas))
    if sugerida and preguntar(f"Los 6 servos del perfil responden a {sugerida} baudios: ¿usar esa velocidad en el perfil?",
                              a.si):
        b.perfil.baudios = sugerida
        b.perfil.anotar(f"velocidad del bus {sugerida} baudios")
        print(f"Guardado en {b.perfil.guardar()}.")
    elif all(l.endswith("ok") for l in lineas[:len(TODAS)]):
        print("Todo en orden. Siguiente: python3 -m tools.brazo salud  y luego  cero")


def cmd_identificar(a):
    from cafebot.arm import dxl
    b = crear(a, conectar=False)
    bd = b.perfil.baudios
    try:
        hallados = dxl.escanear(b.puerto, (bd,)).get(bd, {})
    except (IOError, OSError) as e:
        raise SystemExit(f"{e}")
    if not hallados:
        raise SystemExit(f"Nadie responde a {bd} baudios (escanear prueba otras velocidades).")
    print(f"Responden {len(hallados)} servos. Se enciende el LED de uno a la vez: di qué articulación es.")
    bus = dxl.BusDXL(b.puerto, bd, [dxl.ConfigServo(s) for s in sorted(hallados)])
    asignados = {}
    opciones = "/".join(MOSTRAR[n] for n in TODAS)
    try:
        for sid in sorted(hallados):
            bus.led(sid, True)
            while True:
                r = input(f"  id {sid} con el LED ENCENDIDO: ¿qué articulación es? [{opciones}, ENTER = ninguna]: ").strip()
                if not r:
                    break
                try:
                    n = nombre_articulacion(r)
                except ValueError as e:
                    print(f"  {e}")
                    continue
                if n in asignados:
                    print(f"  {MOSTRAR[n]} ya es el id {asignados[n]}")
                    continue
                asignados[n] = sid
                break
            bus.led(sid, False)
    finally:
        for sid in hallados:
            try:
                bus.led(sid, False)
            except IOError:
                pass
        bus.cerrar()
    ids = b.perfil.ids
    cambios = {n: sid for n, sid in asignados.items() if ids[n] != sid}
    sin = [MOSTRAR[n] for n in TODAS if n not in asignados]
    if sin:
        print("Sin asignar (quedan con su id del perfil): " + ", ".join(sin))
    if not cambios:
        print("Los ids coinciden con el perfil: no hay nada que cambiar.")
        return
    print("Cambios: " + ", ".join(f"{MOSTRAR[n]} {ids[n]} → {sid}" for n, sid in cambios.items()))
    confirmar("Se guardan en el perfil.", a.si)
    try:
        b.perfil.asignar_ids(cambios)
    except ValueError as e:
        raise SystemExit(f"No se guardó: {e}")
    b.perfil.anotar("ids " + ", ".join(f"{n}={sid}" for n, sid in cambios.items()))
    print(f"Guardado en {b.perfil.guardar()}.")
    print("El cero viejo de esas articulaciones era de otro servo: rehacerlo con\n"
          f"  python3 -m tools.brazo cero --solo {','.join(cambios)} --rearmado   (y revisar sus posturas)")


def cmd_salud(a):
    b = crear(a)
    pasos, ang = b.pasos_nombre(), b.angulos_nombre()
    sal = {s.id: s for s in b.bus.salud()}
    for n in TODAS:
        s = sal[b.id[n]]
        print(f"  {MOSTRAR[n]:7s} id {s.id:3d}  {s.temp_c:3d} °C  {s.volt_v:4.1f} V  carga {s.carga_pct:+5.1f} %  "
              f"pasos {pasos[n]:6d}  ángulo {ang[n]:+7.1f}°  {'ERROR: ' + ', '.join(s.errores) if s.error else 'ok'}")
    ok, motivo = b.bus.puede_trabajar()
    print("Puede trabajar:", "SÍ" if ok else f"NO ({motivo})")
    p, pitch = b.directa_robot(b.bus.angulos())
    print(f"Garra {marco(b)}: x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f} m, inclinación {pitch:.0f}°")
    b.cerrar()


def cmd_cero(a):
    arts = _lista_art(a.solo) or list(TODAS)
    b = crear(a)
    confirmar(f"Se suelta el torque de: {', '.join(MOSTRAR[n] for n in arts)}. SUJETA el brazo.", a.si)
    b.bus.torque(False, [b.id[n] for n in arts])
    if not curses.wrapper(_pantalla_cero, b, arts):
        print("Cancelado: no se cambió nada.")
        sostener_o_soltar(b)
        b.cerrar()
        return
    hecho = b.cero(arts, rearmado=a.rearmado)
    b.bus.torque(True)                 # recto y equilibrado: casi no hace fuerza
    for n, (viejo, nuevo, d) in hecho.items():
        print(f"  {MOSTRAR[n]:7s} cero {viejo:5d} → {nuevo:5d} pasos ({d:+6.1f}°)"
              + ("   <- ¡grande! ¿estaba recto?" if abs(d) > 30 else ""))
    if not a.rearmado and any(abs(d) > 0.5 for _, _, d in hecho.values()):
        print("Las posturas grabadas y los límites se corrigieron para quedar FÍSICAMENTE iguales.\n"
              "(Si desarmaste o cambiaste un servo, repite con --rearmado: así siguen al cero nuevo).")
    print(f"Guardado en {b.perfil.ruta}. El brazo queda sostenido recto (torque).")
    print("Siguiente: python3 -m tools.brazo verificar")
    b.cerrar()


def cmd_verificar(a):
    arts = _lista_art(a.solo) or list(TODAS)
    b = crear(a)
    print("Se revisa el SENTIDO de cada articulación: la que toca queda libre y la mueves tú; las demás sostienen.")
    confirmar("Primero el brazo queda SIN torque para que lo pongas recto.", a.si)
    b.bus.torque(False)
    input("SIN torque: pon el brazo RECTO HACIA ARRIBA, base mirando adelante, garra abierta. Sostenlo y ENTER... ")
    b.bus.torque(True)
    ang = b.angulos_nombre()
    print("Recto, lee: " + "  ".join(f"{MOSTRAR[n]} {ang[n]:+.1f}°" for n in TODAS))
    lejos = [n for n in TODAS if abs(ang[n]) > CERO_TOLERANCIA_DEG]
    if lejos:
        print(f"  ¡Ojo! {', '.join(MOSTRAR[n] for n in lejos)} lejos de 0°: si está recto, rehacer el cero "
              f"(cero --solo {','.join(lejos)})")
    resultado = {}
    for n in arts:
        sid = b.id[n]
        print(f"\n[{MOSTRAR[n]}] queda LIBRE. Muévela {SENTIDO_POSITIVO[n]} unos 20–30° (sostenla, no la sueltes).")
        b.bus.torque(False, [sid])
        try:
            r = esperar_movimiento(b.angulos_nombre, n, mostrar=lambda t: print("\r  " + t + "    ", end="", flush=True))
        finally:
            print()
        if r is None:
            print("  No se movió en 30 s. Si estaba DURA, ese id es otro servo: python3 -m tools.brazo identificar")
            resultado[n] = "no se movió"
        else:
            d, otras = r
            if otras:
                print("  ¡Ojo! también se movió " + ", ".join(f"{MOSTRAR[k]} {v:+.0f}°" for k, v in otras.items())
                      + ": ¿ids cruzados? (identificar)")
            if d > 0:
                print(f"  ✓ la lectura SUBIÓ ({d:+.0f}°): sentido correcto")
                resultado[n] = "ok"
            elif preguntar(f"  La lectura BAJÓ ({d:+.0f}°). ¿La moviste {SENTIDO_POSITIVO[n]}?"):
                b.invertir_sentido(n)
                print("  → sentido INVERTIDO y guardado (posturas y límites corregidos)")
                resultado[n] = "invertido"
            else:
                print(f"  No se cambió nada (repetir: verificar --solo {n})")
                resultado[n] = "repetir"
        input("  " + ("ÁBRELA del todo" if n == "garra" else "Devuélvela a la posición RECTA")
              + " y ENTER (queda sostenida)... ")
        b.bus.torque(True, [sid])
        quedo = b.angulos_nombre()[n]
        if abs(quedo) > CERO_TOLERANCIA_DEG:
            print(f"  {'Abierta' if n == 'garra' else 'Recta'} lee {quedo:+.1f}°: revisar el cero (cero --solo {n})")
    print("\nResumen: " + ", ".join(f"{MOSTRAR[n]} {v}" for n, v in resultado.items()))
    print(f"Perfil: {b.perfil.ruta}. El brazo queda sostenido (soltarlo: apagar). Siguiente: python3 -m tools.brazo limites")
    b.cerrar()


def cmd_limites(a):
    from cafebot.arm import brazo as mod
    b = crear(a, conectar=not a.reiniciar, puerto=not a.reiniciar)
    if a.reiniciar:
        confirmar("Los límites del perfil vuelven a los del modelo.", a.si)
        p = b.perfil
        for n in TODAS:
            p.art[n]["min_deg"], p.art[n]["max_deg"] = mod._LIMITES[n]
        p.anotar("límites de fábrica")
        print(f"Guardado en {p.guardar()}.")
        return
    confirmar("El brazo queda SIN torque (sujétalo).", a.si)
    b.bus.torque(False)
    visto = curses.wrapper(_pantalla_limites, b)
    if visto is None:
        print("Cancelado: no se cambió nada.")
    else:
        nuevos, sin_cambio = limites_desde_recorrido(visto)
        for n in TODAS:
            ant = b.perfil.art[n]
            txt = (f"{nuevos[n][0]:+6.1f} .. {nuevos[n][1]:+6.1f}°" if n in nuevos else "sin cambio (se movió poco)")
            print(f"  {MOSTRAR[n]:7s} {ant['min_deg']:+6.0f} .. {ant['max_deg']:+6.0f}°  →  {txt}")
        if nuevos and preguntar("¿Guardar en el perfil?", a.si):
            try:
                b.fijar_limites(nuevos)
            except ValueError as e:
                raise SystemExit(f"No se guardó: {e}")
            print(f"Guardado en {b.perfil.ruta}. Alcance con los límites nuevos (frente al árbol):")
            print("\n".join(informe_alcance(b)))
            print("Si una fila dice NO y antes alcanzaba, el límite quedó corto: repetir o  limites --reiniciar")
    sostener_o_soltar(b)
    b.cerrar()


def cmd_articulacion(a):
    try:
        n = nombre_articulacion(a.nombre)
    except ValueError as e:
        raise SystemExit(str(e))
    b = crear(a)
    actual = b.angulos_nombre()[n]
    meta = actual + a.grados if a.rel else a.grados
    lim = b.perfil.art[n]
    print(f"{MOSTRAR[n]} (id {b.id[n]}): ahora {actual:+.1f}°, meta {meta:+.1f}° | límites "
          f"{lim['min_deg']:+.0f}..{lim['max_deg']:+.0f}° | + = {SENTIDO_POSITIVO[n]}")
    if not lim["min_deg"] <= meta <= lim["max_deg"]:
        raise SystemExit("La meta está fuera de los límites del perfil (ver: python3 -m tools.brazo limites).")
    d = meta - actual
    sentido = SENTIDO_POSITIVO[n] if d > 0 else SENTIDO_NEGATIVO[n]
    confirmar(("MOVIMIENTO GRANDE: " if abs(d) > 30 else "") + f"{MOSTRAR[n]} se moverá {abs(d):.0f}° {sentido}.", a.si)
    b.bus.torque(True)
    ok = b.mover_articulacion(n, meta, a.t)
    real = b.angulos_nombre()[n]
    print(("OK" if ok else f"NO llegó bien: {b.ultimo_motivo}") + f" | {MOSTRAR[n]} en {real:+.1f}°")
    print(linea_salud(b))
    print(f"¿Se movió {sentido}? Si fue al revés: python3 -m tools.brazo verificar --solo {n}")
    print("El torque sigue activo. Para soltarlo: python3 -m tools.brazo apagar")
    b.cerrar()


def calibrar_garra(b, a):
    cb = b.cb
    print(f"Config: abierta {cb.garra_abierta_deg:.0f}°, cerrada {cb.garra_cerrada_deg:.0f}°, "
          f"'no había grano' si cierra más de {cb.garra_vacia_deg:.0f}°")
    b.garra(abrir=True)
    input("1) Garra VACÍA. ENTER para cerrarla... ")
    vacia = b.garra(abrir=False)
    carga = {s.id: s.carga_pct for s in b.bus.salud()}[b.ID_GARRA]
    print(f"   vacía cierra hasta {vacia:.1f}° (carga {carga:+.0f} %)")
    b.garra(abrir=True)
    input("2) Pon un grano (o una esfera de ~42 mm) entre los dedos y sostenlo. ENTER para cerrar... ")
    con = b.garra(abrir=False)
    carga = {s.id: s.carga_pct for s in b.bus.salud()}[b.ID_GARRA]
    print(f"   con grano se detiene en {con:.1f}° (carga {carga:+.0f} %)")
    input("3) Sujeta el grano. ENTER para abrir... ")
    b.garra(abrir=True)
    cambios, avisos = sugerir_garra(vacia, con, cb)
    for t in avisos:
        print("   ¡Ojo! " + t)
    if not cambios:
        return
    print("Propuesta: " + ", ".join(f"brazo.{k} = {v}" for k, v in cambios.items()))
    if preguntar("¿Guardar en pi/robot_local.json?", a.si):
        print(f"Guardado en {guardar_local('brazo', cambios)} (súbelo al repo).")


# ---------------------------------------------------------------------------- comandos: prueba
def cmd_leer(a):
    b = crear(a)
    confirmar("El brazo queda SIN torque (cae si no lo sujetas).", a.si)
    b.bus.torque(False)
    try:
        curses.wrapper(_pantalla_leer, b)
        sostener_o_soltar(b)
    finally:
        b.cerrar()


def cmd_grabar(a):
    b = crear(a)
    post = b.grabar_postura(a.nombre)
    print(f"Grabada '{a.nombre}' en {b.perfil.ruta}: " + " ".join(f"{MOSTRAR[k]} {v:+.1f}°" for k, v in post.items()))
    b.cerrar()


def cmd_ir(a):
    b = crear(a)
    b.bus.torque(True)
    try:
        ok = b.ir(a.nombre, a.t)
    except KeyError as e:
        raise SystemExit(e.args[0])
    print("OK" if ok else f"NO llegó bien: {b.ultimo_motivo}")
    print(linea_salud(b))
    print("El torque sigue activo. Para soltarlo: python3 -m tools.brazo apagar")
    b.cerrar()


def cmd_garra(a):
    b = crear(a)
    b.bus.torque(True, [b.ID_GARRA])
    if a.que == "calibrar":
        calibrar_garra(b, a)
    elif a.que in ("abrir", "cerrar"):
        ang = b.garra(abrir=a.que == "abrir")
        print(f"Garra en {ang:.1f}°  (umbral 'no hay grano': > {b.cb.garra_vacia_deg:.0f}°)")
    else:
        ang = b.garra(angulo_deg=float(a.que))
        print(f"Garra en {ang:.1f}°  (umbral 'no hay grano': > {b.cb.garra_vacia_deg:.0f}°)")
    b.cerrar()


def cmd_punto(a):
    b = crear(a)
    p = np.array([a.x, a.y, a.z])
    s = b.ik(p, a.pitch)
    if s is None:
        raise SystemExit("Fuera de alcance o de los límites (prueba otro punto o --pitch).")
    ang, sol = s
    print("Ángulos: " + "  ".join(f"{b.mostrar(k)} {np.degrees(v):+.1f}°" for k, v in ang.items())
          + f"  | inclinación {np.degrees(sol.pitch):.0f}°  | esfuerzo máx {100 * np.max(np.abs(sol.tau)) / b.modelo.tau_max:.0f}%")
    confirmar("El brazo se moverá.", a.si)
    b.bus.torque(True)
    ok = b.ir_a_punto(p, a.pitch, a.t)
    real, pitch = b.directa_robot(b.bus.angulos())
    print(("OK" if ok else f"NO llegó bien: {b.ultimo_motivo}")
          + f" | garra {marco(b)} en x={real[0]:.3f} y={real[1]:+.3f} z={real[2]:.3f} (pedido {a.x:.3f} {a.y:+.3f} {a.z:.3f})")
    if b.banco:
        print("Mide con una regla desde el eje de la base: la diferencia corrige brazo.garra_largo_m o un cero.")
    else:
        print("Mide con una regla dónde quedó la garra: la diferencia corrige mision.brazo_x/y/z (o un cero).")
    b.cerrar()


def _agarrar(b, p, a):
    plan = b.plan_agarre(p)
    if plan is None:
        raise SystemExit("Fuera de alcance (o de los límites del perfil): python3 -m tools.brazo perfil muestra el alcance.")
    for nombre, ang, t in plan:
        q, pitch = b.directa_robot(ang)
        print(f"  {nombre:12s} x={q[0]:.3f} y={q[1]:+.3f} z={q[2]:.3f}  inclinación {pitch:.0f}°  ({t:.1f} s)")
    aprox = float(np.linalg.norm(b.directa_robot(plan[2][1])[0] - b.directa_robot(plan[0][1])[0]))
    if aprox < b.cb.aproximacion_m - 0.002:
        print(f"  (aproximación acortada a {100 * aprox:.1f} cm: el grano está a {100 * b.a_plano(p)[1]:.1f} cm del eje "
              f"de la base y la garra no se le acerca a menos de {100 * b.cb.r_min_m:.0f} cm, brazo.r_min_m. "
              "Revisar mision.brazo_x)")
    destino = "deposito_maduro" if a.categoria == cancha.MADURO else "deposito_sobremaduro"
    if not a.sin_deposito and destino not in b.posturas:
        raise SystemExit(f"Falta la postura {destino} (grábala con `leer` + G) o usa --sin-deposito.")
    if b.cb.postura_espera not in b.posturas:
        raise SystemExit(f"Falta la postura '{b.cb.postura_espera}' (grábala con `leer` + G).")
    confirmar("El brazo ejecutará la secuencia.", a.si)
    b.bus.torque(True)
    b.garra(abrir=True)
    b.ir(b.cb.postura_espera)
    ok = b.recoger(p, a.categoria, depositar=not a.sin_deposito)
    print("RECOGIDO" if ok else f"NO: {b.ultimo_motivo}")
    if ok and a.sin_deposito:
        input("Tiene el grano. ENTER para abrir la garra (sujeta el grano) y volver a la postura de espera... ")
        b.garra(abrir=True)
        b.ir(b.cb.postura_espera)
    print(linea_salud(b))


def cmd_agarrar(a):
    b = crear(a)
    _agarrar(b, np.array([a.x, a.y, a.z]), a)
    b.cerrar()


def cmd_fila(a):
    b = crear(a)
    p = punto_fila(b.cfg, a.fila, a.lateral, a.holgura, b.banco)
    print(instrucciones_fila(b.cfg, a.fila, a.lateral, a.holgura, p, b.banco))
    _agarrar(b, p, a)
    b.cerrar()


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


def cmd_perfil(a):
    b = crear(a, conectar=False, puerto=False)
    p = b.perfil
    print(f"  bus: {p.baudios} baudios, puerto {p.puerto or '(el de la config)'}")
    print("  articulación  id   cero (pasos)  invertir   límites")
    for n in TODAS:
        x = p.art[n]
        print(f"  {MOSTRAR[n]:12s} {x['id']:3d}   {x['centro']:6d}       {'sí' if x['invertir'] else 'no':3s}    "
              f"{x['min_deg']:+6.1f} .. {x['max_deg']:+6.1f}°")
    print("  posturas:")
    for nombre, post in p.posturas.items():
        print(f"    {nombre:22s} " + "  ".join(f"{MOSTRAR[k]} {v:+.1f}°" for k, v in post.items()))
    faltan = [x for x in POSTURAS_MISION if x not in p.posturas]
    if faltan:
        print("    FALTAN para la misión: " + ", ".join(faltan) + "  (leer + G)")
    print(f"  alcance frente al árbol (cara a {100 * b.cm.holgura_agarre_m:.0f} cm del frente; lateral 0 / ±6 cm):")
    print("\n".join(informe_alcance(b)))
    if p.historial:
        print("  historial:")
        for h in p.historial[-12:]:
            print("    " + h)


def cmd_cambiar_id(a):
    from cafebot.arm import dxl
    b = crear(a, conectar=False)
    destino = a.baudios_nuevo or b.perfil.baudios
    print(f"Servo id {a.viejo} a {a.baudios} baudios  →  id {a.nuevo} a {destino} baudios (EEPROM).")
    print("Los servos de fábrica son TODOS id 1 a 57 600: conectar SOLO el servo nuevo al bus.")
    confirmar("Se escribe la EEPROM del servo.", a.si)
    try:
        dxl.cambiar_id(b.puerto, a.viejo, a.nuevo, a.baudios, destino)
    except (IOError, OSError, ValueError) as e:
        raise SystemExit(f"No se cambió: {e}")
    print(f"Listo: responde como id {a.nuevo}. Conéctalo al brazo, asígnalo con  identificar  y haz su cero con\n"
          "  python3 -m tools.brazo cero --solo <articulación> --rearmado")


def cmd_apagar(a):
    b = crear(a)
    b.bus.torque(True)
    b.garra(abrir=True)
    if "descanso" in b.posturas:
        b.ir("descanso")
    confirmar("Se suelta el torque (sujeta el brazo si no queda apoyado).", a.si)
    b.bus.torque(False)
    print("Torque apagado.")
    b.cerrar()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default=None)
    ap.add_argument("--perfil", default=None, help="brazo (pi/brazos/<perfil>.json); por defecto brazo.perfil")
    ap.add_argument("--banco", action="store_true", help="brazo fuera del robot: coordenadas desde su base")
    ap.add_argument("--si", action="store_true", help="no pedir confirmación")
    comun = argparse.ArgumentParser(add_help=False)          # también se aceptan DESPUÉS del comando
    comun.add_argument("--puerto", default=argparse.SUPPRESS)
    comun.add_argument("--perfil", default=argparse.SUPPRESS)
    comun.add_argument("--banco", action="store_true", default=argparse.SUPPRESS)
    comun.add_argument("--si", action="store_true", default=argparse.SUPPRESS)
    sub = ap.add_subparsers(dest="cmd", required=True)
    _add = sub.add_parser
    sub.add_parser = lambda *x, **k: _add(*x, parents=[comun], **k)
    # calibración
    sub.add_parser("escanear", help="servos en el bus").set_defaults(f=cmd_escanear)
    sub.add_parser("identificar", help="LED uno por uno: qué id es cada articulación").set_defaults(f=cmd_identificar)
    sub.add_parser("salud").set_defaults(f=cmd_salud)
    s = sub.add_parser("cero", help="brazo recto hacia arriba = 0°")
    s.add_argument("--solo", default="", help="articulaciones separadas por coma (p. ej. hombro,codo1)")
    s.add_argument("--rearmado", action="store_true",
                   help="se desmontó o cambió el servo: las posturas NO se corrigen (siguen al cero nuevo)")
    s.set_defaults(f=cmd_cero)
    s = sub.add_parser("verificar", help="sentido de cada articulación")
    s.add_argument("--solo", default="")
    s.set_defaults(f=cmd_verificar)
    s = sub.add_parser("limites", help="recorrer a mano el rango seguro de cada articulación")
    s.add_argument("--reiniciar", action="store_true", help="volver a los límites del modelo")
    s.set_defaults(f=cmd_limites)
    s = sub.add_parser("articulacion", help="mover UNA articulación (grados)")
    s.add_argument("nombre")
    s.add_argument("grados", type=float)
    s.add_argument("--rel", action="store_true", help="relativo a la posición actual")
    s.add_argument("--t", type=float, default=2.0, help="segundos")
    s.set_defaults(f=cmd_articulacion)
    s = sub.add_parser("garra")
    s.add_argument("que", help="abrir | cerrar | calibrar | ángulo en grados")
    s.set_defaults(f=cmd_garra)
    # prueba
    sub.add_parser("leer").set_defaults(f=cmd_leer)
    s = sub.add_parser("grabar")
    s.add_argument("nombre")
    s.set_defaults(f=cmd_grabar)
    s = sub.add_parser("ir")
    s.add_argument("nombre")
    s.add_argument("--t", type=float, default=2.0, help="segundos")
    s.set_defaults(f=cmd_ir)
    for nombre, f in (("punto", cmd_punto), ("agarrar", cmd_agarrar), ("fila", cmd_fila)):
        s = sub.add_parser(nombre)
        if nombre == "fila":
            s.add_argument("fila", choices=cancha.FILAS)
            s.add_argument("--lateral", type=float, default=0.0, help="m a la izquierda del eje del brazo")
            s.add_argument("--holgura", type=float, default=None, help="m del frente del robot a la cara del tablero")
        else:
            s.add_argument("x", type=float)
            s.add_argument("y", type=float)
            s.add_argument("z", type=float)
            s.add_argument("--pitch", type=float, default=None, help="inclinación de la garra (90 = horizontal)")
            s.add_argument("--t", type=float, default=1.5)
        if nombre != "punto":
            s.add_argument("--categoria", default=cancha.MADURO, choices=(cancha.MADURO, cancha.SOBREMADURO))
            s.add_argument("--sin-deposito", action="store_true", help="queda con el grano (sin postura de depósito)")
        s.set_defaults(f=f)
    sub.add_parser("teleop").set_defaults(f=cmd_teleop)
    sub.add_parser("perfil", help="ver el perfil, las posturas y el alcance").set_defaults(f=cmd_perfil)
    s = sub.add_parser("cambiar-id", help="servo nuevo de fábrica -> id y velocidad del brazo")
    s.add_argument("viejo", type=int)
    s.add_argument("nuevo", type=int)
    s.add_argument("--baudios", type=int, default=57_600, help="velocidad ACTUAL del servo (fábrica: 57600)")
    s.add_argument("--baudios-nuevo", type=int, default=None, help="por defecto la del perfil")
    s.set_defaults(f=cmd_cambiar_id)
    sub.add_parser("apagar").set_defaults(f=cmd_apagar)
    a = ap.parse_args()
    a.f(a)


if __name__ == "__main__":
    main()
