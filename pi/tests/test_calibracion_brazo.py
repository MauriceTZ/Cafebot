"""Calibración del brazo SIN hardware (bus Dynamixel falso): perfil de cada brazo, cero, sentidos, límites,
ids, escaneo del bus, servo nuevo de fábrica y los comandos de tools.brazo (las pantallas curses se simulan).

    python3 -m tests.test_calibracion_brazo
"""
import argparse
import builtins
import contextlib
import curses
import io
import json
import math
import pathlib
import tempfile

import numpy as np

from tests import bus_dxl_falso

bus = bus_dxl_falso.instalar()

from cafebot.arm import brazo as mb  # noqa: E402
from cafebot.arm import dxl  # noqa: E402
from cafebot.config import cargar  # noqa: E402
from cafebot.mundo import cancha  # noqa: E402
import tools.brazo as tb  # noqa: E402

TMP = pathlib.Path(tempfile.mkdtemp())
mb.RAIZ_PI = TMP                       # los perfiles se escriben en una carpeta temporal
K = 4096 / 360.0


def crudo(perfil, n, grados):
    """Paso del encoder que corresponde a `grados` de la articulación `n` (lo FÍSICO)."""
    a = perfil.art[n]
    return a["centro"] + (-1 if a["invertir"] else 1) * grados * K


def nuevo_brazo(cfg, perfil="principal", banco=False):
    cfg.brazo.perfil = perfil
    b = mb.BrazoXL430(cfg, "falso", log=lambda t: None, banco=banco)
    b.conectar()
    return b


def ns(**k):
    base = dict(puerto="falso", perfil=None, banco=False, si=True)
    base.update(k)
    return argparse.Namespace(**base)


@contextlib.contextmanager
def respuestas(*rs):
    """input() responde en orden; una respuesta puede ser (texto, acción) para simular lo que hace la persona."""
    pendientes = list(rs)
    original = builtins.input

    def falso(prompt=""):
        assert pendientes, f"input() inesperado: {prompt}"
        r = pendientes.pop(0)
        if isinstance(r, tuple):
            r[1]()
            r = r[0]
        return r
    builtins.input = falso
    try:
        yield
    finally:
        builtins.input = original
    assert not pendientes, f"sobraron respuestas: {pendientes}"


@contextlib.contextmanager
def pantalla(valor):
    """curses.wrapper falso: la pantalla 'devuelve' `valor` (ENTER en cero = True, recorrido en limites)."""
    original = curses.wrapper
    curses.wrapper = lambda f, *a: valor
    try:
        yield
    finally:
        curses.wrapper = original


def callado(f, *a):
    with contextlib.redirect_stdout(io.StringIO()) as out:
        f(*a)
    return out.getvalue()


# ------------------------------------------------------------------------------------------------------------
def prueba_perfil():
    p = mb.PerfilBrazo.de_fabrica()
    assert p.ids == {"base": 9, "hombro": 8, "codo1": 11, "codo2": 7, "muneca": 5, "garra": 4}
    assert [(c.id, c.centro_pasos, c.invertir) for c in p.servos()] == \
           [(c.id, c.centro_pasos, c.invertir) for c in dxl.BRAZO + [dxl.GARRA]]
    # las posturas del código antiguo (en pasos) quedan en grados y dentro de los límites de fábrica
    for nombre in ("trabajo", "descanso"):
        for n, v in p.posturas[nombre].items():
            assert p.art[n]["min_deg"] <= v <= p.art[n]["max_deg"], (nombre, n, v)
    # guardar / cargar
    carpeta = TMP / "brazos"
    p.ruta = carpeta / "ida_vuelta.json"
    p.posturas["deposito_maduro"] = {"base": 170.0, "hombro": -20.0, "codo1": 90.0, "codo2": 60.0, "muneca": 10.0}
    p.anotar("prueba")
    p.guardar()
    q = mb.PerfilBrazo.cargar("ida_vuelta", carpeta)
    assert q.art == p.art and q.posturas == p.posturas and q.historial == p.historial
    # posturas del formato anterior (claves = id del servo) pasan al perfil nuevo
    viejo = TMP / "brazo_posturas.json"
    viejo.write_text(json.dumps({"deposito_maduro": {"9": 175.0, "8": -10.0, "11": 80.0, "7": 50.0, "5": 5.0}}))
    q = mb.PerfilBrazo.cargar("nuevo", carpeta, posturas_antiguas=viejo)
    assert q.posturas["deposito_maduro"] == {"base": 175.0, "hombro": -10.0, "codo1": 80.0, "codo2": 50.0,
                                             "muneca": 5.0}
    # un perfil roto NO se ignora (con ceros de fábrica el brazo iría a cualquier lado)
    (carpeta / "roto.json").write_text("{ esto no es json")
    try:
        mb.PerfilBrazo.cargar("roto", carpeta)
        raise AssertionError("debía fallar")
    except ValueError as e:
        assert "roto.json" in str(e)
    assert mb.nombre_articulacion("Muñeca") == "muneca"
    print("  perfil: fábrica, guardar/cargar, formato anterior y perfil roto")


def prueba_cero_y_sentido():
    p = mb.PerfilBrazo.de_fabrica()
    antes = {n: crudo(p, n, v) for n, v in p.posturas["descanso"].items()}
    lim = {n: (crudo(p, n, p.art[n]["min_deg"]), crudo(p, n, p.art[n]["max_deg"])) for n in mb.TODAS}
    # El cero real del hombro estaba 57 pasos (5°) más allá: posturas y límites quedan FÍSICAMENTE iguales
    d = p.fijar_cero("hombro", p.art["hombro"]["centro"] + 57)
    assert abs(abs(d) - 57 / K) < 1e-9
    assert abs(crudo(p, "hombro", p.posturas["descanso"]["hombro"]) - antes["hombro"]) < 0.2   # redondeo a 0,01°
    assert abs(crudo(p, "hombro", p.art["hombro"]["min_deg"]) - lim["hombro"][0]) < 1.5       # redondeo a 0,1°
    # rearmado (servo desmontado): las posturas siguen al cero nuevo (mismos grados)
    g = p.posturas["descanso"]["codo1"]
    p.fijar_cero("codo1", 100, rearmado=True)
    assert p.posturas["descanso"]["codo1"] == g and p.art["codo1"]["centro"] == 100
    # cambiar el sentido: postura con el signo cambiado = el mismo paso del encoder; límites espejados
    antes_m = crudo(p, "muneca", p.posturas["descanso"]["muneca"])
    lo, hi = p.art["muneca"]["min_deg"], p.art["muneca"]["max_deg"]
    p.invertir("muneca")
    assert abs(crudo(p, "muneca", p.posturas["descanso"]["muneca"]) - antes_m) < 1e-6
    assert (p.art["muneca"]["min_deg"], p.art["muneca"]["max_deg"]) == (-hi, -lo)
    try:
        p.asignar_ids({"hombro": 9})          # ya es la base
        raise AssertionError("debía fallar")
    except ValueError:
        pass
    print("  cero y sentido: posturas y límites quedan físicamente iguales (y con --rearmado siguen al cero)")


def prueba_con_bus(cfg):
    bus.__init__()
    b = nuevo_brazo(cfg)
    ids = b.id
    # 1) Postura enseñada a mano con los ceros del código antiguo
    c0 = {c.id: c.centro_pasos for c in dxl.BRAZO + [dxl.GARRA]}
    pose = {9: c0[9] + 300, 8: c0[8] - 200, 11: c0[11] + 400, 7: c0[7] + 500, 5: c0[5] - 100}
    for sid, v in pose.items():
        bus.mem[(sid, 132)] = v
    b.grabar_postura("prueba")
    # 2) El brazo recto de ESTE brazo no coincide con los ceros antiguos: cero
    recto = {9: c0[9] + 20, 8: c0[8] - 30, 11: c0[11] + 15, 7: c0[7] + 40, 5: c0[5] - 25, 4: c0[4] + 10}
    for sid, v in recto.items():
        bus.mem[(sid, 132)] = v
    assert max(abs(v) for v in b.angulos_nombre().values()) > 2
    hecho = b.cero()
    assert all(abs(v) < 1e-9 for v in b.angulos_nombre().values()), b.angulos_nombre()
    assert hecho["codo2"][1] == c0[7] + 40
    assert json.loads(b.perfil.ruta.read_text())["articulaciones"]["codo2"]["centro"] == c0[7] + 40
    # la postura enseñada sigue llevando el brazo al MISMO lugar físico
    assert b.ir("prueba")
    for sid, v in pose.items():
        assert abs(bus.mem[(sid, 132)] - v) <= 1, (sid, bus.mem[(sid, 132)], v)
    # 3) Una articulación con el sentido al revés: tras invertir, +10° mueve el encoder hacia el otro lado
    for sid, v in recto.items():
        bus.mem[(sid, 132)] = v
    b.bus.torque(True)
    assert b.mover_articulacion("codo1", 10, t=0.01)
    d1 = bus.mem[(ids["codo1"], 132)] - recto[ids["codo1"]]
    b.mover_articulacion("codo1", 0, t=0.01)
    b.invertir_sentido("codo1")
    assert b.mover_articulacion("codo1", 10, t=0.01)
    d2 = bus.mem[(ids["codo1"], 132)] - recto[ids["codo1"]]
    assert d1 == -d2 and abs(d1) == round(10 * K), (d1, d2)
    b.invertir_sentido("codo1")
    b.mover_articulacion("codo1", 0, t=0.01)
    # 4) Límites: la IK y los movimientos de a una los respetan
    p = np.array([0.20, 0.0, 0.45])
    assert b.ik(p) is not None
    lim = (b.perfil.art["hombro"]["min_deg"], b.perfil.art["hombro"]["max_deg"])
    b.fijar_limites({"hombro": (-5.0, 5.0)})
    s = b.ik(p)
    assert s is None or abs(math.degrees(s[0][ids["hombro"]])) <= 5 + 1e-6
    assert not b.mover_articulacion("hombro", 20, t=0.01) and "fuera de los límites" in b.ultimo_motivo
    b.fijar_limites({"hombro": lim})
    try:
        b.fijar_limites({"codo1": (10, 12)})
        raise AssertionError("debía fallar")
    except ValueError:
        pass
    b.fijar_limites({"base": (-20.0, 20.0)})               # la base solo ±20°: un grano a 45° queda fuera
    assert b.ik([0.10 + 0.15, 0.15, 0.45]) is None and b.ik([0.25, 0.0, 0.45]) is not None
    b.fijar_limites({"base": (-150.0, 150.0)})
    assert len(b.perfil.historial) >= 5
    b.cerrar()
    print("  con el bus: cero de un brazo real, postura físicamente igual, sentido y límites")


def prueba_escaneo_e_ids(cfg):
    bus.__init__()
    perfil = mb.PerfilBrazo.de_fabrica()
    hallados = dxl.escanear("falso")
    assert set(hallados) == {1_000_000} and set(hallados[1_000_000]) == {9, 8, 11, 7, 5, 4}
    lineas, sugerida = tb.analizar_escaneo(perfil, hallados)
    assert all(l.endswith("ok") for l in lineas) and sugerida is None, lineas
    # un servo nuevo, de fábrica (id 1 a 57 600)
    bus.agregar(1, 2048, 57_600)
    lineas, _ = tb.analizar_escaneo(perfil, dxl.escanear("falso"))
    assert any("FÁBRICA" in l for l in lineas), lineas
    dxl.cambiar_id("falso", 1, 12, 57_600, 1_000_000)
    h = dxl.escanear("falso")
    assert 57_600 not in h and 12 in h[1_000_000]
    # otro de fábrica que se quiere llamar como uno que ya existe: no se toca
    bus.agregar(1, 2048, 57_600)
    try:
        dxl.cambiar_id("falso", 1, 8, 57_600, 1_000_000)
        raise AssertionError("debía fallar")
    except IOError as e:
        assert "ya hay un servo con id 8" in str(e)
    assert bus.baud_de[1] == 57_600
    # todos los del perfil a otra velocidad: se propone cambiarla
    bus.__init__()
    for sid in list(bus.baud_de):
        bus.baud_de[sid] = 57_600
    lineas, sugerida = tb.analizar_escaneo(perfil, dxl.escanear("falso"))
    assert sugerida == 57_600 and "responde a 57600" in lineas[0], lineas
    print("  escaneo: ids del perfil, servo de fábrica (cambiar-id), id ocupado y velocidad del bus")


def prueba_ayudas(cfg):
    # esperar_movimiento: lecturas guionadas
    def lector(secuencia):
        it = iter(secuencia)
        ultimo = [None]

        def leer():
            ultimo[0] = next(it, ultimo[0])
            return ultimo[0]
        return leer
    cero = {n: 0.0 for n in mb.TODAS}
    sec = [dict(cero, hombro=v) for v in (0, 4, 9, 16)]
    assert tb.esperar_movimiento(lector(sec), "hombro", periodo=0) == (16, {})
    sec = [dict(cero, hombro=v, codo1=v / 2) for v in (0, -8, -18)]
    d, otras = tb.esperar_movimiento(lector(sec), "hombro", periodo=0)
    assert d == -18 and set(otras) == {"codo1"}
    assert tb.esperar_movimiento(lector([cero]), "hombro", timeout_s=0.05, periodo=0.01) is None
    # límites con margen; lo que casi no se movió no cambia
    nuevos, sin = tb.limites_desde_recorrido({"hombro": (-80.0, 90.0), "codo1": (-3.0, 4.0)})
    assert nuevos == {"hombro": (-77.0, 87.0)} and sin == ["codo1"]
    # garra: umbral entre vacía y con grano
    cb = cfg.brazo
    assert tb.sugerir_garra(65.0, 40.0, cb) == ({"garra_vacia_deg": 52.5}, [])
    c, av = tb.sugerir_garra(55.0, 40.0, cb)
    assert c == {"garra_vacia_deg": 47.5, "garra_cerrada_deg": 57.0} and av
    assert tb.sugerir_garra(64.0, 63.0, cb)[0] == {} and tb.sugerir_garra(60.0, 55.0, cb)[0] == {}
    # banco = mismas articulaciones que montado en el robot
    bus.__init__()
    b_robot, b_banco = nuevo_brazo(cfg), nuevo_brazo(cfg, banco=True)
    for fila in cancha.FILAS:
        pr = b_robot.plan_agarre(tb.punto_fila(cfg, fila))
        pb = b_banco.plan_agarre(tb.punto_fila(cfg, fila, banco=True))
        assert pr is not None and pb is not None, fila
        for (n1, a1, _), (n2, a2, _) in zip(pr, pb):
            assert n1 == n2 and all(abs(a1[k] - a2[k]) < 1e-9 for k in a1), (fila, n1)
    assert len(tb.informe_alcance(b_robot)) == 3
    texto = tb.instrucciones_fila(cfg, "alta", 0.06, None, tb.punto_fila(cfg, "alta", 0.06, banco=True), True)
    assert "BANCO" in texto and "izquierda" in texto
    print("  ayudas: detección del movimiento, límites, garra, banco = robot")


def prueba_comandos(cfg):
    """Los comandos de tools.brazo de punta a punta (input y pantallas simuladas)."""
    bus.__init__()
    cfg_ruta = TMP / "brazos" / "principal.json"
    if cfg_ruta.exists():
        cfg_ruta.unlink()
    callado(tb.cmd_perfil, ns())
    callado(tb.cmd_escanear, ns())
    callado(tb.cmd_salud, ns())
    # cero: el brazo "recto" en otra posición; ENTER en la pantalla
    recto = {9: 3100, 8: 1030, 11: 3080, 7: 1550, 5: 1000, 4: 340}
    for sid, v in recto.items():
        bus.mem[(sid, 132)] = v
    with pantalla(True):
        out = callado(tb.cmd_cero, ns(solo="", rearmado=False))
    assert "1030" in out and bus.mem[(8, 64)] == 1          # cero guardado y el brazo sostenido (torque)
    perfil = mb.PerfilBrazo.cargar("principal")
    assert perfil.art["hombro"]["centro"] == 1030 and perfil.art["base"]["centro"] == 3100
    # verificar el codo2: la persona lo mueve hacia adelante y la lectura BAJA -> se invierte
    inv0 = perfil.art["codo2"]["invertir"]
    original = tb.esperar_movimiento
    tb.esperar_movimiento = lambda *x, **k: (-20.0, {})
    try:
        with respuestas("", "s", ""):
            out = callado(tb.cmd_verificar, ns(solo="codo2"))
        assert "INVERTIDO" in out, out
        assert mb.PerfilBrazo.cargar("principal").art["codo2"]["invertir"] == (not inv0)
        tb.esperar_movimiento = lambda *x, **k: (20.0, {})
        with respuestas("", ""):
            out = callado(tb.cmd_verificar, ns(solo="codo2"))
        assert "sentido correcto" in out
    finally:
        tb.esperar_movimiento = original
    # límites recorridos a mano
    visto = {n: (-2.0, 2.0) for n in mb.TODAS}
    visto["hombro"] = (-70.0, 80.0)
    with pantalla(visto), respuestas(""):
        out = callado(tb.cmd_limites, ns(reiniciar=False))
    perfil = mb.PerfilBrazo.cargar("principal")
    assert (perfil.art["hombro"]["min_deg"], perfil.art["hombro"]["max_deg"]) == (-67.0, 77.0), perfil.art["hombro"]
    assert "fila alta" in out
    callado(tb.cmd_limites, ns(reiniciar=True))
    assert mb.PerfilBrazo.cargar("principal").art["hombro"]["max_deg"] == 100.0
    # primer movimiento con torque, de a una articulación
    out = callado(tb.cmd_articulacion, ns(nombre="hombro", grados=10.0, rel=True, t=0.01))
    assert out.count("OK") >= 1 and "hacia ADELANTE" in out, out
    # garra: vacía cierra hasta 65°, con grano se detiene en ~40° -> umbral ~52,5° (el cero de la garra
    # quedó en 340 con `cero`: el grano la frena en 336 + 40°, o sea en 39,6°)
    guardado = {}
    original_g = tb.guardar_local
    tb.guardar_local = lambda seccion, valores: guardado.update(valores) or "robot_local.json"
    bus.grano = False
    try:
        with respuestas("", ("", lambda: setattr(bus, "grano", True)), ("", lambda: setattr(bus, "grano", False))):
            callado(tb.cmd_garra, ns(que="calibrar"))
    finally:
        tb.guardar_local = original_g
    con_grano = (336 + int(40 / 360 * 4096) - 340) / K
    assert guardado == {"garra_vacia_deg": round((65 + con_grano) / 2, 1)}, guardado
    # agarre con coordenadas fijas (fila media), sin depósito: queda con el grano
    bus.grano = True
    with respuestas(""):
        out = callado(tb.cmd_fila, ns(fila="media", lateral=0.0, holgura=None, categoria=cancha.MADURO,
                                      sin_deposito=True))
    assert "RECOGIDO" in out, out
    bus.grano = False
    # identificar con otro perfil: la persona dice que el 8 es el codo1 y el 11 el hombro
    with respuestas("garra", "muñeca", "codo2", "codo1", "base", "hombro"):
        out = callado(tb.cmd_identificar, ns(perfil="segundo"))
    seg = mb.PerfilBrazo.cargar("segundo")
    assert seg.ids["hombro"] == 11 and seg.ids["codo1"] == 8 and "--rearmado" in out, out
    # un servo nuevo de fábrica
    bus.agregar(1, 2048, 57_600)
    callado(tb.cmd_cambiar_id, ns(viejo=1, nuevo=12, baudios=57_600, baudios_nuevo=None))
    assert bus.baud_de.get(12) == 1_000_000 and 1 not in bus.baud_de
    print("  comandos: perfil, escanear, salud, cero, verificar, limites, articulacion, garra, fila, identificar, "
          "cambiar-id")


def prueba_segundo_brazo(cfg):
    """Otro brazo con otros ids en el mismo bus: cada uno con su perfil."""
    bus.__init__()
    nuevos = {"base": 21, "hombro": 22, "codo1": 23, "codo2": 24, "muneca": 25, "garra": 26}
    for n, sid in nuevos.items():
        bus.agregar(sid, mb.PerfilBrazo.de_fabrica().art[n]["centro"])
    p = mb.PerfilBrazo.cargar("brazo2")
    p.asignar_ids(nuevos)
    p.guardar()
    b2 = nuevo_brazo(cfg, "brazo2")
    assert b2.id == nuevos and b2.IDS == [21, 22, 23, 24, 25] and b2.ID_GARRA == 26
    antes = bus.mem[(8, 132)]
    b2.bus.torque(True)
    assert b2.mover_articulacion("hombro", 10, t=0.01)
    assert bus.mem[(8, 132)] == antes and bus.mem[(22, 132)] != mb.PerfilBrazo.de_fabrica().art["hombro"]["centro"]
    b1 = nuevo_brazo(cfg, "principal")
    assert b1.id["hombro"] == 8
    print("  segundo brazo: su perfil, sus ids, no mueve al otro")


def main():
    cfg = cargar()
    cfg.brazo.t_mov_s = cfg.brazo.t_fino_s = 0.01          # sin esperas en la prueba
    print("Calibración del brazo (bus falso)")
    prueba_perfil()
    prueba_cero_y_sentido()
    prueba_con_bus(cfg)
    prueba_escaneo_e_ids(cfg)
    prueba_ayudas(cfg)
    prueba_comandos(cfg)
    prueba_segundo_brazo(cfg)
    print("OK: calibración del brazo")


if __name__ == "__main__":
    main()
