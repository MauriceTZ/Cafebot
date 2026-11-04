"""Compuertas Dynamixel (id 12 maduros, 13 sobremaduros) y botón de inicio (GPIO 23), SIN hardware.
    python3 -m tests.test_compuertas

  1. Las compuertas quedan en modo posición con su PWM limitado y torque, en el MISMO bus que el brazo (el brazo
     sigue en posición extendida). Sin calibrar no se mueven; calibradas, vaciar = abrir, esperar y cerrar.
  2. Los límites se graban en el servo y no deja pedir un ángulo fuera; una compuerta trabada se informa; si saltó
     una protección, se reinicia y vuelve a tener torque.
  3. armar_reales (tools.mision) arma las compuertas Dynamixel (o las del Mega con compuertas.tipo = "mega").
  4. Botón (gpiozero falso): parte al soltar; un pulso de ruido de 10 ms no cuenta; si al empezar está apretado,
     espera que lo suelten y que lo vuelvan a apretar.
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


def metas(sid):
    return [v for s, a, v in falso.escrituras if s == sid and a == ADDR_GOAL]


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
    n = len(metas(12))
    assert dep.vaciar(0) is False and len(metas(12)) == n           # sin calibrar: no se mueve
    print(f"  ids {cc.ids} en modo posición, PWM {cc.pwm_limit}, con torque; el brazo ({brazo_ids}) sigue en modo 4; "
          "sin calibrar no se mueven")

    cc.cerrada_deg, cc.abierta_deg = (180.0, 180.0), (250.0, 110.0)
    n12, n13 = len(metas(12)), len(metas(13))
    assert dep.vaciar(0) is True and dep.vaciar(1) is True
    assert metas(12)[n12:] == [grados_a_pasos(250), grados_a_pasos(180)], metas(12)[n12:]
    assert metas(13)[n13:] == [grados_a_pasos(110), grados_a_pasos(180)], metas(13)[n13:]
    print("  calibradas: vaciar = abrir (250° / 110°), esperar y cerrar (180°)")
    return bus


def prueba_limites_y_fallas(cfg, bus):
    cc = cfg.compuertas
    log = []
    comp = CompuertasDXL(bus, cc, log=log.append)
    cc.limites_deg = ([170.0, 260.0], None)
    comp.configurar(0)
    assert falso.leer(12, ADDR_MIN_POS) == grados_a_pasos(170) and falso.leer(12, ADDR_MAX_POS) == grados_a_pasos(260)
    comp.torque(0, True)
    try:
        comp.mover(0, 300.0)
        raise AssertionError("debía rechazar 300°")
    except ValueError:
        pass
    # trabada: la de sobremaduros no pasa de 150° al abrir hacia 110°
    original = falso.escribir

    def trabada(sid, addr, val):
        if sid == 13 and addr == ADDR_GOAL:
            val = max(val, grados_a_pasos(150))
        original(sid, addr, val)
    falso.escribir = trabada
    try:
        assert comp.abrir(1) is False and any("traba" in t for t in log), log
    finally:
        falso.escribir = original
    # sobrecarga: el servo apagó el torque; antes de moverla se reinicia y se le devuelve el torque
    falso.mem[(12, ADDR_HW)] = 32
    falso.mem[(12, ADDR_TORQUE)] = 0
    assert comp.abrir(0) is True and 12 in falso.reboots and falso.leer(12, ADDR_TORQUE) == 1
    print("  límites 170..260° grabados en el servo (300° rechazado); trabada en 150° se informa; tras una "
          "sobrecarga se reinicia y vuelve el torque")


def prueba_armar(cfg):
    from tools.mision import armar_reales
    cfg.compuertas.tipo = "dxl"
    _, _, dep = armar_reales(cfg, False, "nula", mega=None)
    assert isinstance(dep, DepositosDXL) and dep.preparar() and dep.comp.ids == (12, 13)
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


def main():
    cfg = cargar()
    print("1) Compuertas en el bus del brazo")
    bus = prueba_compuertas(cfg)
    print("2) Límites, compuerta trabada y sobrecarga")
    prueba_limites_y_fallas(cfg, bus)
    print("3) La misión arma las compuertas")
    prueba_armar(cargar())
    print("4) Botón de inicio")
    prueba_boton(cfg)
    print("OK: compuertas y botón")


if __name__ == "__main__":
    main()
