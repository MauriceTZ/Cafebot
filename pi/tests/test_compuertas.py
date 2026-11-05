"""Compuertas Dynamixel (config.compuertas.ids: maduros, sobremaduros) y botón de inicio (GPIO 23), SIN hardware.
    python3 -m tests.test_compuertas

  1. Las compuertas quedan en modo posición con su PWM limitado y torque, en el MISMO bus que el brazo (el brazo
     sigue en posición extendida). Sin calibrar no se mueven; calibradas, vaciar = abrir, esperar y cerrar.
  2. Los límites se graban en el servo y no deja pedir un ángulo fuera; una compuerta trabada se informa; si saltó
     una protección, se reinicia y vuelve a tener torque.
  3. armar_reales (tools.mision) arma las compuertas Dynamixel (o las del Mega con compuertas.tipo = "mega").
  4. Botón (gpiozero falso): parte al soltar; un pulso de ruido de 10 ms no cuenta; si al empezar está apretado,
     espera que lo suelten y que lo vuelvan a apretar.
  5. Calibración guiada (tools.compuertas calibrar) con un "usuario" falso que mueve las compuertas a mano: graba
     cerrada, abierta y límites, y la prueba con torque cierra, abre y cierra; si entre cerrada y abierta se pasa por
     0°, esa compuerta no se guarda ni se mueve (el servo daría la vuelta larga).
"""
import sys
import threading
import time
import types

from tests import bus_dxl_falso

falso = bus_dxl_falso.instalar()            # antes de importar cafebot.arm

from cafebot.arm import dxl   # noqa: E402
from cafebot.arm.compuertas import ADDR_MAX_POS, ADDR_MIN_POS, CompuertasDXL, DepositosDXL, grados_a_pasos   # noqa: E402
from cafebot.arm.brazo import PerfilBrazo   # noqa: E402
from cafebot.config import cargar   # noqa: E402

ADDR_MODO, ADDR_PWM, ADDR_TORQUE, ADDR_GOAL, ADDR_HW = 11, 36, 64, 116, 70
M, S = cargar().compuertas.ids          # maduros, sobremaduros


def metas(sid):
    return [v for s, a, v in falso.escrituras if s == sid and a == ADDR_GOAL]


def sin_calibrar(cfg):
    """Las pruebas parten de compuertas sin calibrar (config.py ya trae los ángulos medidos en el robot)."""
    cc = cfg.compuertas
    cc.abierta_deg = cc.cerrada_deg = cc.limites_deg = (None, None)
    return cfg


def nuevo_bus(cfg):
    for sid in cfg.compuertas.ids:
        falso.agregar(sid, 2048)                         # 180°
    perfil = PerfilBrazo.de_fabrica()                    # los ids que tiene el bus falso
    bus = dxl.BusDXL("/dev/falso", perfil.baudios, perfil.servos())
    bus.configurar()                                      # el brazo: posición extendida
    return bus, perfil


def prueba_compuertas(cfg):
    bus, perfil = nuevo_bus(cfg)
    log = []
    cc = cfg.compuertas
    dep = DepositosDXL(cfg, lambda: bus, log=log.append)
    cfg.mision.t_vaciado_s = 0.05
    assert dep.preparar()
    for sid in cc.ids:
        assert falso.leer(sid, ADDR_MODO) == 3 and falso.leer(sid, ADDR_PWM) == cc.pwm_limit
        assert falso.leer(sid, ADDR_TORQUE) == 1
    brazo_ids = [c.id for c in perfil.servos()]
    assert all(falso.leer(s, ADDR_MODO) == 4 for s in brazo_ids), "el brazo cambió de modo"
    assert any("SIN CALIBRAR" in t for t in log)
    n = len(metas(M))
    assert dep.vaciar(0) is False and len(metas(M)) == n           # sin calibrar: no se mueve
    print(f"  ids {cc.ids} en modo posición, PWM {cc.pwm_limit}, con torque; el brazo ({brazo_ids}) sigue en modo 4; "
          "sin calibrar no se mueven")

    cc.cerrada_deg, cc.abierta_deg = (180.0, 180.0), (250.0, 110.0)
    n12, n13 = len(metas(M)), len(metas(S))
    assert dep.vaciar(0) is True and dep.vaciar(1) is True
    assert metas(M)[n12:] == [grados_a_pasos(250), grados_a_pasos(180)], metas(M)[n12:]
    assert metas(S)[n13:] == [grados_a_pasos(110), grados_a_pasos(180)], metas(S)[n13:]
    print("  calibradas: vaciar = abrir (250° / 110°), esperar y cerrar (180°)")
    return bus


def prueba_limites_y_fallas(cfg, bus):
    cc = cfg.compuertas
    log = []
    comp = CompuertasDXL(bus, cc, log=log.append)
    cc.limites_deg = ([170.0, 260.0], None)
    comp.configurar(0)
    assert falso.leer(M, ADDR_MIN_POS) == grados_a_pasos(170) and falso.leer(M, ADDR_MAX_POS) == grados_a_pasos(260)
    comp.torque(0, True)
    try:
        comp.mover(0, 300.0)
        raise AssertionError("debía rechazar 300°")
    except ValueError:
        pass
    # trabada: la de sobremaduros no pasa de 150° al abrir hacia 110°
    original = falso.escribir

    def trabada(sid, addr, val):
        if sid == S and addr == ADDR_GOAL:
            val = max(val, grados_a_pasos(150))
        original(sid, addr, val)
    falso.escribir = trabada
    try:
        assert comp.abrir(1) is False and any("traba" in t for t in log), log
    finally:
        falso.escribir = original
    # sobrecarga: el servo apagó el torque; antes de moverla se reinicia y se le devuelve el torque
    falso.mem[(M, ADDR_HW)] = 32
    falso.mem[(M, ADDR_TORQUE)] = 0
    assert comp.abrir(0) is True and M in falso.reboots and falso.leer(M, ADDR_TORQUE) == 1
    print("  límites 170..260° grabados en el servo (300° rechazado); trabada en 150° se informa; tras una "
          "sobrecarga se reinicia y vuelve el torque")


def prueba_armar(cfg):
    from tools.mision import armar_reales
    cfg.compuertas.tipo = "dxl"
    _, _, dep = armar_reales(cfg, False, "nula", mega=None)
    assert isinstance(dep, DepositosDXL) and dep.preparar() and dep.comp.ids == (M, S)
    cfg.compuertas.tipo = "mega"
    _, _, dep2 = armar_reales(cfg, False, "nula", mega=None)
    assert type(dep2).__name__ == "Depositos"
    print("  tools.mision: compuertas Dynamixel con un bus propio sin --brazo (con --brazo, el del brazo); "
          "tipo 'mega' = X 0 / X 1")


class BotonFalso:
    """gpiozero.Button de mentira: estado controlado por la prueba."""

    def __init__(self, pin, pull_up=True, bounce_time=None, pin_factory=None):
        assert pull_up, "el botón va con el pull-up interno"
        self.pin, self.estado, self.cv = pin, False, threading.Condition()

    @property
    def is_pressed(self):
        return self.estado

    def poner(self, apretado):
        with self.cv:
            self.estado = apretado
            self.cv.notify_all()

    def wait_for_press(self, timeout=None):
        with self.cv:
            return self.cv.wait_for(lambda: self.estado, timeout)

    def wait_for_release(self, timeout=None):
        with self.cv:
            return self.cv.wait_for(lambda: not self.estado, timeout)


def prueba_boton(cfg):
    sys.modules["gpiozero"] = types.SimpleNamespace(Button=BotonFalso)
    from cafebot.mision.interfaces import BotonGPIO

    def correr(inicial, secuencia):
        """secuencia: [(espera_s, apretado)]. Devuelve en qué paso (índice) terminó esperar()."""
        b = BotonGPIO(cfg.mision.pin_boton, log=lambda t: None)
        b.b.poner(inicial)
        fin = {}
        h = threading.Thread(target=lambda: (b.esperar(), fin.setdefault("t", time.monotonic())), daemon=True)
        h.start()
        marcas = []
        for espera, apretado in secuencia:
            time.sleep(espera)
            marcas.append(time.monotonic())
            b.b.poner(apretado)
        h.join(1.0)
        assert "t" in fin, "esperar() no terminó"
        return max(i for i, t in enumerate(marcas) if t <= fin["t"])

    assert cfg.mision.pin_boton == 23
    # normal: apretar 0,2 s y soltar -> parte al soltar (paso 1)
    assert correr(False, [(0.1, True), (0.2, False)]) == 1
    # ruido de 10 ms, después un apretón de verdad -> parte al soltar el de verdad (paso 3)
    assert correr(False, [(0.1, True), (0.01, False), (0.2, True), (0.2, False)]) == 3
    # apretado desde antes (trabado): lo sueltan, lo aprietan y lo sueltan -> paso 2
    assert correr(True, [(0.2, False), (0.2, True), (0.2, False)]) == 2
    print("  GPIO 23 con pull-up: parte al soltar; ignora un pulso de 10 ms; apretado al empezar no parte solo")


def prueba_calibracion(tmp):
    import argparse
    import json
    import pathlib
    from cafebot.arm import compuertas as C
    from cafebot.config import guardar_local
    import tools.compuertas as T

    assert not C.cruza_cero(180, 250) and C.cruza_cero(350, 20) and C.cruza_cero(20, 350) and not C.cruza_cero(90, 10)
    ruta = pathlib.Path(tmp) / "robot_local.json"
    T.guardar_local = lambda sec, val: guardar_local(sec, val, ruta=str(ruta))

    def mano(sid, grados):                       # el "usuario" mueve la compuerta (sin torque) a ese ángulo
        def f():
            falso.mem[(sid, 132)] = grados_a_pasos(grados)
        return f
    respuestas = [(mano(M, 180.0), ""), (mano(M, 250.0), ""), (None, "s"),       # maduros + prueba con torque
                  (mano(S, 350.0), ""), (mano(S, 20.0), "")]                     # sobremaduros: pasa por 0°

    def entrada(prompt=""):
        accion, r = respuestas.pop(0)
        if accion:
            accion()
        return r
    T.input = entrada
    cfg = sin_calibrar(cargar())
    cfg.mision.t_vaciado_s = 0.05
    escrituras_antes = len(falso.escrituras)
    bus = dxl.BusDXL("/dev/falso", 1_000_000, [dxl.ConfigServo(i) for i in cfg.compuertas.ids])
    comp = CompuertasDXL(bus, cfg.compuertas, log=lambda t: None)
    T.cmd_calibrar(argparse.Namespace(compuerta=None, margen=5.0), cfg, comp)
    assert not respuestas, respuestas
    guardado = json.loads(ruta.read_text())["compuertas"]
    assert guardado["cerrada_deg"] == [180.0, None] and guardado["abierta_deg"] == [250.0, None], guardado
    assert guardado["limites_deg"][0] == [175.0, 255.0], guardado
    assert falso.leer(M, ADDR_MIN_POS) == grados_a_pasos(175) and falso.leer(M, ADDR_MAX_POS) == grados_a_pasos(255)
    assert metas(M)[-3:] == [grados_a_pasos(180), grados_a_pasos(250), grados_a_pasos(180)], metas(M)[-3:]
    nuevas = falso.escrituras[escrituras_antes:]
    assert not any(s == S and a in (20, ADDR_MIN_POS, ADDR_MAX_POS, ADDR_GOAL) for s, a, _ in nuevas)
    print("  maduros: cerrada 180°, abierta 250°, límites 175..255° en el servo y la prueba cerró, abrió y cerró | "
          "sobremaduros 350° -> 20° pasa por 0°: no se guarda ni se mueve (pide montar el cuerno girado)")


def main():
    cfg = sin_calibrar(cargar())
    print("1) Compuertas en el bus del brazo")
    bus = prueba_compuertas(cfg)
    print("2) Límites, compuerta trabada y sobrecarga")
    prueba_limites_y_fallas(cfg, bus)
    print("3) La misión arma las compuertas")
    prueba_armar(sin_calibrar(cargar()))
    print("4) Botón de inicio")
    prueba_boton(cfg)
    print("5) Calibración guiada")
    import tempfile
    with tempfile.TemporaryDirectory() as tmp:
        prueba_calibracion(tmp)
    print("OK: compuertas y botón")


if __name__ == "__main__":
    main()
