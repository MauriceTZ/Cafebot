"""Sensores de línea (4 × QTR-1A en A8..A11 del Mega): ver lecturas, calibrar UMBRALES y probar la detección.

    python3 -m tools.ir                          # monitor en vivo + calibración con teclas (pantalla de texto)
    python3 -m tools.ir prueba                   # cruzar una línea a varias velocidades y medir la detección
    python3 -m tools.ir aplicar                  # mandar al Mega los umbrales de robot_local.json (sección "ir")
    python3 -m tools.ir borde [--ajustar]        # retroceder hasta el borde de atrás, enderezarse y cangrejo hasta
                                                 # el borde derecho; mide blanco/negro y sugiere umbrales
    python3 -m tools.ir --sim [prueba]           # sin robot: firmware simulado + una línea a 30 cm delante
    (--puerto /dev/ttyUSB1 si no están instaladas las reglas udev)

Teclas del monitor:
    W  blanco: los 4 sensores sobre la lona            N  negro: los 4 sobre una línea (o 1..4: uno solo)
    A  auto-calibración: A para empezar, pasar los sensores sobre la línea (a mano o con ↑/↓), A para terminar
    B  barrido automático (avanza 30 cm sobre una línea y vuelve, registrando mín./máx.); B otra vez confirma
    TAB elegir sensor   +/- umbral ±10   [ ] histéresis ±10   D valores de fábrica
    ↑/↓ mover el robot 2 cm   S freno al ver línea   G GUARDAR (EEPROM del Mega + pi/robot_local.json)   Q salir
"""
from __future__ import annotations

import argparse
import curses
import math
import time

import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.drivers.mega import NOMBRES_IR, Mega

LINEA_SIM_X = 0.30     # m: en --sim hay una línea de 19 mm perpendicular al avance, a 30 cm del centro del robot
BORDE_SIM = (0.35, 0.40)   # m: en --sim borde, líneas a 35 cm detrás y 40 cm a la derecha del centro del robot


# ---------------------------------------------------------------------------- conexión
def conectar(a, cfg):
    ir_sim = None
    if a.sim:
        from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
        from cafebot.mundo.cancha import Rect
        from tests.sim_serial import SerialSimulado, compilar
        compilar()
        mega = Mega("sim")
        mega.conectar(serial_obj=SerialSimulado())
        mega.telemetria(50)
        if getattr(a, "cmd", None) == "borde":     # esquina: borde 35 cm detrás y 40 cm a la derecha del centro
            lineas = [Rect(-BORDE_SIM[0], 0.0, 0.019, 3.0), Rect(0.0, -BORDE_SIM[1], 3.0, 0.019)]
            mega.fijar_pose(0, 0, a.torcido)       # el robot parte torcido respecto de los bordes
        else:
            lineas = [Rect(LINEA_SIM_X, 0.0, 0.019, 3.0)]
        ir_sim = IRSimulado(mega, cfg.ir.posiciones, lineas, lambda: pose_mega_extrapolada(mega)).iniciar()
        time.sleep(0.3)
    else:
        mega = Mega(a.puerto or cfg.puertos.mega)
        print("Conectando con el Mega (se reinicia, ~2 s)...")
        mega.conectar()
        mega.telemetria(50)
    return mega, ir_sim


def barra(v, u, h, ancho=40) -> str:
    """Barra 0..1023 con el umbral marcado (|) y la banda de histéresis (:)."""
    pos = lambda x: max(0, min(ancho - 1, int(x / 1024 * ancho)))
    b = ["█" if i < pos(v) else "·" for i in range(ancho)]
    for x in (u - h / 2, u + h / 2):
        b[pos(x)] = ":"
    b[pos(u)] = "|"
    return "".join(b)


# ---------------------------------------------------------------------------- monitor
def _txt(scr, y, x, t, attr=0):
    alto, ancho = scr.getmaxyx()
    if y < alto - 1:
        try:
            scr.addstr(y, x, t[:max(0, ancho - x - 1)], attr)
        except curses.error:
            pass


def monitor(scr, mega, cfg):
    curses.curs_set(0)
    scr.nodelay(True)
    mega.ir_monitor(25)
    est = mega.ir_estado()
    sel = 0
    msg = "Robot sobre la lona: W. Luego los sensores sobre la línea: N (o A/B para la auto-calibración)."
    mins, maxs = [1023] * 4, [0] * 4
    cambios = False
    confirmar_barrido = False
    n_cruces = 0
    while True:
        while not mega.ir_eventos.empty():
            ev = mega.ir_eventos.get_nowait()
            n_cruces += bin(ev.subida).count("1")
        while not mega.eventos.empty():
            e = mega.eventos.get_nowait()
            if e.tipo == "IRCAL":
                msg = f"¡{NOMBRES_IR[int(e.args[1])]} sin contraste! lectura {e.args[2]} vs blanco {e.args[3]}: ¿está sobre la línea? ¿muy alto?"
        crudo = mega.ir_crudo
        vals, mask = (crudo[0], crudo[1]) if crudo else (est.valores, est.mascara)
        for k in range(4):
            mins[k], maxs[k] = min(mins[k], vals[k]), max(maxs[k], vals[k])
        scr.erase()
        _txt(scr, 0, 0, "SENSORES DE LÍNEA (QTR-1A)   negro = lectura alta   umbral = punto medio blanco/negro", curses.A_BOLD)
        _txt(scr, 2, 0, f"     {'lectura (0..1023)':40s}  valor  estado  blanco negro umbral contraste  mín/máx sesión")
        for k in range(4):
            u, h = est.umbrales[k], est.histeresis
            contraste = abs(est.negro[k] - est.blanco[k])
            negro = bool(mask & (1 << k))
            marca = ">" if k == sel else " "
            _txt(scr, 3 + k, 0, f"{marca}{NOMBRES_IR[k]}  {barra(vals[k], u, h)}  {vals[k]:5d}  "
                                f"{'NEGRO ' if negro else 'blanco'}  {est.blanco[k]:5d} {est.negro[k]:5d} {u:6d} "
                                f"{contraste:9d}  {mins[k]:4d}/{maxs[k]:<4d}",
                 curses.A_REVERSE if negro else 0)
        _txt(scr, 8, 0, f"Histéresis {est.histeresis} | freno al ver línea: {'SÍ' if est.seguridad else 'no'} | "
                        f"auto-calibración: {'REGISTRANDO' if est.auto else '—'} | cruces vistos: {n_cruces} | "
                        f"{'CAMBIOS SIN GUARDAR (G)' if cambios else 'guardado'}")
        _txt(scr, 10, 0, "W blanco | N negro (4) / 1-4 uno | A auto | B barrido | TAB sensor | +/- umbral | [ ] histéresis")
        _txt(scr, 11, 0, "↑/↓ mover 2 cm | S freno | D fábrica | G guardar (Mega + robot_local.json) | Q salir")
        _txt(scr, 13, 0, ">> " + msg)
        bajos = [NOMBRES_IR[k] for k in range(4) if abs(est.negro[k] - est.blanco[k]) < 300]
        if bajos:
            _txt(scr, 14, 0, f"Contraste bajo (< 300) en {', '.join(bajos)}: acercar el sensor al piso (óptimo 3 mm) o recalibrar.")
        scr.refresh()

        k = scr.getch()
        if k == -1:
            time.sleep(0.05)
            continue
        if k != ord("b"):
            confirmar_barrido = False
        try:
            if k in (ord("q"), ord("Q")):
                if cambios:
                    msg = "Hay cambios sin guardar: G para guardar, Q otra vez para salir sin guardar."
                    cambios = False
                    continue
                break
            elif k in (ord("w"), ord("W")):
                est = mega.ir_calibrar_blanco()
                cambios, msg = True, f"Blanco medido: {est.blanco}. Ahora los sensores sobre la línea: N (o 1-4)."
            elif k in (ord("n"), ord("N")):
                est = mega.ir_calibrar_negro()
                cambios, msg = True, f"Negro medido: {est.negro}. Umbrales: {est.umbrales}."
            elif k in (ord("1"), ord("2"), ord("3"), ord("4")):
                s = k - ord("1")
                est = mega.ir_calibrar_negro(s)
                cambios, msg = True, f"Negro de {NOMBRES_IR[s]}: {est.negro[s]} → umbral {est.umbrales[s]}."
            elif k in (ord("a"), ord("A")):
                if not est.auto:
                    mega.ir_auto(True)
                    msg = "Registrando: pasa los 4 sensores sobre la línea (a mano o ↑/↓) y A para terminar."
                else:
                    r = mega.ir_auto(False)
                    ok = [NOMBRES_IR[s] for s in range(4) if r["calibrados"] & (1 << s)]
                    cambios, msg = True, f"Auto: calibrados {', '.join(ok) or 'ninguno'} | mín {r['min']} máx {r['max']}"
                est = mega.ir_estado()
            elif k in (ord("b"), ord("B")):
                if not confirmar_barrido:
                    confirmar_barrido = True
                    msg = "Barrido: el robot avanza 30 cm y vuelve. Debe haber una línea delante. B otra vez para empezar."
                else:
                    confirmar_barrido = False
                    msg = barrido(mega)
                    cambios = True
                    est = mega.ir_estado()
            elif k == 9:                                           # TAB
                sel = (sel + 1) % 4
            elif k in (ord("+"), ord("="), ord("-")):
                d = 10 if k != ord("-") else -10
                est = mega.ir_umbral(sel, max(0, min(1023, est.umbrales[sel] + d)))
                cambios, msg = True, f"Umbral de {NOMBRES_IR[sel]}: {est.umbrales[sel]}"
            elif k in (ord("["), ord("]")):
                est = mega.ir_histeresis(max(0, min(400, est.histeresis + (10 if k == ord("]") else -10))))
                cambios, msg = True, f"Histéresis: {est.histeresis}"
            elif k in (ord("d"), ord("D")):
                est = mega.ir_defecto()
                cambios, msg = True, "Valores de fábrica (config.h). G para guardarlos."
            elif k in (ord("s"), ord("S")):
                mega.seguridad_ir(not est.seguridad)
                est = mega.ir_estado()
                msg = "Freno al ver línea " + ("ACTIVADO (todos los sensores)" if est.seguridad else "desactivado")
            elif k in (curses.KEY_UP, curses.KEY_DOWN):
                mega.avanzar(20 if k == curses.KEY_UP else -20, 60, timeout=5)
            elif k in (ord("g"), ord("G")):
                mega.ir_guardar()
                ruta = guardar_local("ir", {"umbrales": list(est.umbrales), "histeresis": est.histeresis})
                cambios, msg = False, f"Guardado en la EEPROM del Mega y en {ruta.name} (súbelo al repo)."
        except Exception as e:
            msg = f"ERROR: {e}"
    mega.ir_monitor(0)


def barrido(mega, dist_mm=300, vel=60) -> str:
    """Auto-calibración pasando sobre una línea: IA 1 → avanzar → volver → IA 0."""
    mega.ir_auto(True)
    r1 = mega.avanzar(dist_mm, vel, timeout=dist_mm / vel + 10)
    r2 = mega.avanzar(-dist_mm, vel, timeout=dist_mm / vel + 10)
    r = mega.ir_auto(False)
    if not (r1.ok and r2.ok):
        return f"El barrido se interrumpió ({r1.motivo or r2.motivo}). {r['respuesta']}"
    ok = [NOMBRES_IR[s] for s in range(4) if r["calibrados"] & (1 << s)]
    no = [NOMBRES_IR[s] for s in range(4) if not r["calibrados"] & (1 << s)]
    return (f"Barrido: calibrados {', '.join(ok) or 'ninguno'}" + (f"; SIN línea: {', '.join(no)}" if no else "")
            + f" | mín {r['min']} máx {r['max']}")


# ---------------------------------------------------------------------------- prueba de cruce
def prueba(mega, cfg, dist_mm, velocidades, veces):
    """El robot, perpendicular a una línea que tiene a pocos cm delante, la cruza y vuelve. Para cada
    sensor: dónde entra y sale (mm recorridos desde el punto de partida), ancho visto y diferencia ida/vuelta."""
    mega.seguridad_ir(False)
    sx = [p[0] for p in cfg.ir.posiciones]
    resumen = []
    print(f"Línea delante (perpendicular). Cruce de {dist_mm} mm a {velocidades} mm/s, {veces} vez/veces cada una.\n")
    print(f"{'vel':>5} {'sentido':>7} {'sensor':>6} {'entra':>7} {'sale':>7} {'ancho':>6}  {'línea (centro)':>14}")
    for vel in velocidades:
        centros = {}
        for _ in range(veces):
            p0 = np.array(mega.pose()) / np.array([1000.0, 1000.0, 1.0])   # ida y vuelta medidas desde aquí
            for sentido in (1, -1):
                while not mega.ir_eventos.empty():
                    mega.ir_eventos.get_nowait()
                r = mega.avanzar(sentido * dist_mm, vel, timeout=dist_mm / vel + 10)
                time.sleep(0.2)
                if not r.ok:
                    print(f"  movimiento abortado: {r.motivo}")
                    return
                u = np.array([math.cos(p0[2]), math.sin(p0[2])])
                entra, sale = {}, {}
                while not mega.ir_eventos.empty():
                    ev = mega.ir_eventos.get_nowait()
                    d = float((np.array([ev.x, ev.y]) - p0[:2]) @ u)        # recorrido (m) en el evento
                    for k in range(4):
                        if ev.subida & (1 << k):
                            entra[k] = d
                        if ev.bajada & (1 << k) and k in entra:
                            sale[k] = d
                for k in sorted(sale):
                    # línea en el marco de partida: posición del sensor + recorrido en el punto medio
                    centro = sx[k] + (entra[k] + sale[k]) / 2
                    centros.setdefault((k, sentido), []).append(centro)
                    print(f"{vel:5.0f} {'ida' if sentido > 0 else 'vuelta':>7} {NOMBRES_IR[k]:>6} "
                          f"{1000 * entra[k]:7.1f} {1000 * sale[k]:7.1f} {1000 * abs(sale[k] - entra[k]):6.1f}  "
                          f"{1000 * centro:11.1f} mm")
        for k in range(4):
            ida, vuelta = centros.get((k, 1), []), centros.get((k, -1), [])
            if ida and vuelta:
                # la detección llega tarde: en la ida la línea "aparece" más lejos y en la vuelta más cerca
                retraso_mm = (np.mean(ida) - np.mean(vuelta)) / 2 * 1000
                resumen.append((vel, NOMBRES_IR[k], np.std(ida + vuelta) * 1000, retraso_mm, retraso_mm / vel * 1000))
    print("\nResumen (por sensor y velocidad):  retraso = cuánto después del borde real se avisa")
    print(f"{'vel':>5} {'sensor':>6} {'dispersión':>11} {'retraso':>9} {'≈ ms':>6}")
    for vel, n, disp, ret, ms in resumen:
        print(f"{vel:5.0f} {n:>6} {disp:9.1f} mm {ret:7.1f} mm {ms:6.1f}")
    print("\nBueno: ancho ≈ 19 mm (cruzando perpendicular), dispersión < 3 mm, retraso de pocos mm. "
          "Si un sensor no aparece, no vio la línea: revisar altura y umbral.")


# ---------------------------------------------------------------------------- buscar los bordes
DI, DD, TI, TD = 0, 1, 2, 3


def _vaciar(mega) -> list:
    evs = []
    while not mega.ir_eventos.empty():
        evs.append(mega.ir_eventos.get_nowait())
    return evs


def _lecturas(mega, n=5) -> np.ndarray:
    """Promedio de n lecturas de los 4 sensores (robot quieto)."""
    return np.mean([mega.ir_estado().valores for _ in range(n)], axis=0)


def _hasta_linea(mega, mover, mascara: int):
    """Mueve con el freno de borde activo SOLO para los sensores de `mascara`. Devuelve (resultado, {sensor:
    EventoIR de su primera entrada a negro}). El Mega frena en el mismo ciclo en que el sensor ve la línea."""
    _vaciar(mega)
    mega.seguridad_ir(True, mascara)
    try:
        r = mover()
    finally:
        mega.seguridad_ir(False)
    time.sleep(0.15)
    toque = {}
    for ev in _vaciar(mega):
        for k in range(4):
            if ev.subida & mascara & (1 << k) and k not in toque:
                toque[k] = ev
    return r, toque


def _borde(mega, cfg, nombre, mover, mover_lento, par, log):
    """Busca un borde con el par de sensores `par` = (a, b). Si toca primero uno, sigue despacio hasta que toca
    el otro: la distancia extra dice cuán torcido está el robot respecto del borde.
    Devuelve {primero, ambos, extra_m, sep_m} o None si no encontró el borde."""
    a, b = par
    r, toque = _hasta_linea(mega, mover, (1 << a) | (1 << b))
    if r.ok or r.motivo != "BORDE":
        log(f"  {nombre}: NO se encontró ({'recorrió todo sin ver línea' if r.ok else r.motivo})")
        return None
    pa, pb = (np.array(p, float) for p in (cfg.ir.posiciones[a], cfg.ir.posiciones[b]))
    sep = float(np.linalg.norm(pa - pb))
    if len(toque) == 2 or not toque:
        log(f"  {nombre}: {NOMBRES_IR[a]} y {NOMBRES_IR[b]} lo tocaron juntos")
        return {"primero": None, "ambos": True, "extra_m": 0.0, "sep_m": sep}
    primero = next(iter(toque))
    otro = b if primero == a else a
    log(f"  {nombre}: primero {NOMBRES_IR[primero]}; sigo despacio hasta que {NOMBRES_IR[otro]} lo toque...")
    r2, toque2 = _hasta_linea(mega, mover_lento, 1 << otro)
    if otro not in toque2:
        log(f"  {nombre}: {NOMBRES_IR[otro]} no lo tocó ({r2.motivo or 'fin del recorrido'}): el robot está muy torcido "
            "o ese sensor no ve la línea (revisar su umbral)")
        return {"primero": primero, "ambos": False, "extra_m": None, "sep_m": sep}
    e1, e2 = toque[primero], toque2[otro]
    extra = float(np.hypot(e2.x - e1.x, e2.y - e1.y))
    return {"primero": primero, "ambos": True, "extra_m": extra, "sep_m": sep}


def buscar_bordes(mega, cfg, vel=80.0, max_mm=1500.0, despegar_mm=40.0, alinear=True, log=print) -> dict:
    """Desde cualquier punto de la cancha: retrocede hasta el borde de ATRÁS (sensores TI/TD), se endereza con él,
    y va en cangrejo hacia la DERECHA hasta el borde derecho (DD/TD). Al tocar cada borde mide las lecturas sobre
    el negro, para comprobar los umbrales. Las ruedas quedan junto a la línea: solo para el taller.
    Devuelve un resumen con lo medido (blanco/negro por sensor, umbrales sugeridos, torcimiento)."""
    out = {"atras": None, "derecha": None, "torcido_atras_deg": None, "torcido_derecha_deg": None}
    est = mega.ir_estado()
    if est.mascara:
        log(f"¡Ojo! ya hay sensores sobre negro al empezar: "
            f"{', '.join(NOMBRES_IR[k] for k in range(4) if est.mascara & (1 << k))}. Alejar el robot de las líneas.")
        out["error"] = "sensores sobre negro al empezar"
        return out
    blanco = _lecturas(mega)
    negro = [None] * 4
    out["umbrales_antes"] = list(est.umbrales)
    log(f"Blanco (al partir): " + "  ".join(f"{NOMBRES_IR[k]} {blanco[k]:.0f}" for k in range(4))
        + f"   | umbrales {est.umbrales}, histéresis {est.histeresis}")
    lento = max(30.0, vel / 2)

    def torcimiento(res, signo_primero):
        if not res or res["extra_m"] is None:
            return None
        if res["primero"] is None:
            return 0.0
        g = math.degrees(math.atan2(res["extra_m"], res["sep_m"]))
        return signo_primero[res["primero"]] * g     # giro (°, + = antihorario) que deja el robot paralelo al borde

    # 1) Atrás: TI (izquierda) primero = el robot está girado a la izquierda -> corregir girando a la derecha
    log("1) Retrocediendo hasta el borde de ATRÁS...")
    atras = lambda mm, v: (lambda: mega.avanzar(-mm, v, timeout=mm / v + 10))
    res = _borde(mega, cfg, "borde de atrás", atras(max_mm, vel), atras(100, lento), (TI, TD), log)
    if res is None:
        out["error"] = "no encontró el borde de atrás"
        return out
    giro = torcimiento(res, {TI: -1, TD: +1})
    out["atras_inicial"], out["torcido_inicial_deg"] = res, giro
    out["torcido_atras_deg"] = giro
    if giro is not None:
        log(f"  torcido {abs(giro):.1f}° respecto del borde de atrás")
    if alinear and giro is not None and abs(giro) > 0.7:
        mega.avanzar(despegar_mm, 60, timeout=10)
        log(f"  enderezando: giro {giro:+.1f}° y vuelvo a buscar el borde")
        mega.girar(giro, timeout=10)
        res = _borde(mega, cfg, "borde de atrás (2ª vez)", atras(despegar_mm + 150, lento), atras(60, 30), (TI, TD), log)
        if res is None:
            out["error"] = "perdió el borde de atrás al enderezar"
            return out
        out["torcido_atras_deg"] = g2 = torcimiento(res, {TI: -1, TD: +1})
        if g2 is not None:
            log(f"  después de enderezar: torcido {abs(g2):.1f}°")
    out["atras"] = res
    mega.avanzar(-8, 30, timeout=5)                       # los sensores al medio de la huincha (19 mm)
    v = _lecturas(mega)
    negro[TI], negro[TD] = v[TI], v[TD]
    log(f"  sobre el borde: TI {v[TI]:.0f}  TD {v[TD]:.0f}")
    mega.avanzar(despegar_mm + 8, 60, timeout=10)         # despegarse del borde

    # 2) Derecha (cangrejo a −90°): DD (adelante) primero = el robot está girado a la derecha -> girar a la izquierda
    log("2) Cangrejo a la DERECHA hasta el borde derecho...")
    der = lambda mm, v: (lambda: mega.cangrejo(-90, mm, v, timeout=mm / v + 10))
    res = _borde(mega, cfg, "borde derecho", der(max_mm, vel), der(100, lento), (DD, TD), log)
    if res is None:
        out["error"] = "no encontró el borde derecho"
    else:
        out["derecha"] = res
        g = torcimiento(res, {DD: +1, TD: -1})
        out["torcido_derecha_deg"] = g
        if g is not None:
            log(f"  torcido {abs(g):.1f}° respecto del borde derecho"
                + (" (ya enderezado con el de atrás: si da varios grados, el cangrejo se desvía o los bordes no son "
                   "perpendiculares)" if alinear and abs(g) > 1.5 else ""))
        mega.cangrejo(-90, 8, 30, timeout=5)
        v = _lecturas(mega)
        negro[DD] = v[DD]
        negro[TD] = v[TD] if negro[TD] is None else (negro[TD] + v[TD]) / 2
        log(f"  sobre el borde: DD {v[DD]:.0f}  TD {v[TD]:.0f}")
        mega.cangrejo(90, despegar_mm + 8, 60, timeout=10)

    # 3) Umbrales: punto medio entre el blanco y el negro medidos
    est = mega.ir_estado()
    out["blanco"] = [round(float(x)) for x in blanco]
    out["negro"] = [None if x is None else round(float(x)) for x in negro]
    out["sugeridos"] = list(est.umbrales)
    log("3) Umbrales (sensor: blanco / negro medidos -> umbral actual, sugerido):")
    for k in range(4):
        if negro[k] is None:
            log(f"  {NOMBRES_IR[k]}: blanco {blanco[k]:.0f} / negro sin medir (no tocó ningún borde) -> umbral {est.umbrales[k]}")
            continue
        c = negro[k] - blanco[k]
        sug = int(round((blanco[k] + negro[k]) / 2))
        out["sugeridos"][k] = sug if c >= 120 else est.umbrales[k]
        nota = ("CONTRASTE BAJO: acercar el sensor al piso (3 mm)" if c < 300 else "ok")
        margen = min(est.umbrales[k] - est.histeresis / 2 - blanco[k], negro[k] - est.umbrales[k] - est.histeresis / 2)
        log(f"  {NOMBRES_IR[k]}: blanco {blanco[k]:.0f} / negro {negro[k]:.0f} (contraste {c:.0f}, {nota}) -> umbral "
            f"{est.umbrales[k]} (margen {margen:.0f}), sugerido {sug}")
    return out


def cmd_borde(mega, cfg, a):
    print("Busca el borde de ATRÁS (retrocede) y luego el DERECHO (cangrejo a la derecha), con el freno de borde.\n"
          f"Velocidad {a.vel:.0f} mm/s, hasta {a.max:.0f} mm. Deja espacio libre atrás y a la derecha. "
          "Las ruedas quedan junto a la línea (en la cancha sería falta: es solo para el taller).")
    if not a.si and input("¿Seguir? [s/N] ").strip().lower() not in ("s", "si", "sí", "y"):
        return
    mega.ir_monitor(0)
    res = buscar_bordes(mega, cfg, vel=a.vel, max_mm=a.max, alinear=not a.sin_alinear)
    if "error" in res:
        print(f"\nNO TERMINÓ: {res['error']}")
    if res.get("sugeridos") and a.ajustar:
        cambios = [(k, u) for k, (u, v) in enumerate(zip(res["sugeridos"], res["umbrales_antes"])) if u != v]
        for k, u in cambios:
            mega.ir_umbral(k, u)
        if cambios:
            est = mega.ir_estado()
            mega.ir_guardar()
            ruta = guardar_local("ir", {"umbrales": list(est.umbrales), "histeresis": est.histeresis})
            print(f"Umbrales aplicados {est.umbrales}: guardados en la EEPROM del Mega y en {ruta.name}.")
    elif res.get("sugeridos"):
        print("Para aplicar los sugeridos: repetir con --ajustar, o a mano en el monitor (python3 -m tools.ir, TAB y +/-).")


# ---------------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", default=None, help="puerto del Mega (por defecto el de config)")
    ap.add_argument("--sim", action="store_true", help="firmware simulado + una línea simulada a 30 cm")
    sub = ap.add_subparsers(dest="cmd")
    sub.add_parser("monitor")
    s = sub.add_parser("prueba")
    s.add_argument("--dist", type=float, default=400, help="mm a recorrer (la línea debe quedar en medio)")
    s.add_argument("--vel", default="80,150,250", help="velocidades en mm/s, separadas por coma")
    s.add_argument("--veces", type=int, default=2)
    sub.add_parser("aplicar")
    s = sub.add_parser("borde", help="retroceder hasta el borde de atrás y luego cangrejo hasta el borde derecho")
    s.add_argument("--vel", type=float, default=80, help="mm/s de la búsqueda")
    s.add_argument("--max", type=float, default=1500, help="mm máximos en cada dirección")
    s.add_argument("--sin-alinear", action="store_true", help="no enderezar el robot con el borde de atrás")
    s.add_argument("--ajustar", action="store_true", help="aplicar y guardar los umbrales sugeridos")
    s.add_argument("--si", action="store_true", help="no pedir confirmación")
    s.add_argument("--torcido", type=float, default=4.0, help="(--sim) grados que parte torcido el robot")
    a = ap.parse_args()
    cfg = cargar()
    mega, ir_sim = conectar(a, cfg)
    try:
        if a.cmd == "borde":
            cmd_borde(mega, cfg, a)
        elif a.cmd == "prueba":
            prueba(mega, cfg, a.dist, [float(v) for v in a.vel.split(",")], a.veces)
        elif a.cmd == "aplicar":
            if cfg.ir.umbrales is None and cfg.ir.histeresis is None:
                print("robot_local.json no tiene umbrales (sección \"ir\"): el Mega usa los de su EEPROM.")
            else:
                est = mega.ir_aplicar(cfg.ir.umbrales, cfg.ir.histeresis)
                print(mega.ir_guardar(), "| umbrales", est.umbrales, "histéresis", est.histeresis)
        else:
            curses.wrapper(monitor, mega, cfg)
            est = mega.ir_estado()
            print(f"Umbrales {est.umbrales} | histéresis {est.histeresis} | blanco {est.blanco} | negro {est.negro}")
    finally:
        if ir_sim:
            ir_sim.detener()
        mega.cerrar()


if __name__ == "__main__":
    main()
