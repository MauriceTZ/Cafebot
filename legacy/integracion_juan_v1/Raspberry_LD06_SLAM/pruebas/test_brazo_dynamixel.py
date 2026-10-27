"""Prueba del adaptador BrazoDynamixel SIN hardware: dynamixel_sdk y mujoco falsos.

Verifica: (1) postura de trabajo al iniciar, (2) rechazos con motivo, (3) secuencia completa OK,
(4) cancelación inmediata, (5) cerrar() estaciona en reposo ANTES de soltar el torque.
Uso:  python pruebas/test_brazo_dynamixel.py
"""
import math
import os
import sys
import time
import types
from unittest.mock import MagicMock

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(AQUI))

# ---------------------------------------------------------------- dynamixel_sdk falso
COMM_SUCCESS = 0
LOG = []            # ("goal", {id: pasos}) | ("torque", id, valor)


class _Port:
    is_open = True

    def __init__(self, p):
        pass

    def openPort(self):
        return True

    def setBaudRate(self, b):
        return True

    def closePort(self):
        self.is_open = False


class _Packet:
    def __init__(self, v):
        pass

    def read4ByteTxRx(self, port, sid, addr):
        import main_stress_test as a
        return a.OFFSETS_APAGARSE[sid], COMM_SUCCESS, 0      # el brazo arranca en reposo

    def write1ByteTxRx(self, port, sid, addr, v):
        import main_stress_test as a
        if addr == a.ADDR_TORQUE_ENABLE:
            LOG.append(("torque", sid, v))
        return COMM_SUCCESS, 0

    write2ByteTxRx = write4ByteTxRx = lambda self, port, sid, addr, v: (COMM_SUCCESS, 0)

    def getTxRxResult(self, c):
        return f"comm {c}"


class _Sync:
    def __init__(self, *a):
        self.p = {}

    def clearParam(self):
        self.p = {}

    def addParam(self, sid, data):
        self.p[sid] = int.from_bytes(bytes(data), "little", signed=True)
        return True

    def txPacket(self):
        LOG.append(("goal", dict(self.p)))
        return COMM_SUCCESS


sdk = types.ModuleType("dynamixel_sdk")
sdk.PortHandler, sdk.PacketHandler, sdk.COMM_SUCCESS = _Port, _Packet, COMM_SUCCESS
sdk.GroupSyncWrite = sdk.GroupSyncRead = _Sync
sys.modules["dynamixel_sdk"] = sdk
for n in ("mujoco", "mujoco.viewer"):
    sys.modules.setdefault(n, MagicMock())

import brazo_dynamixel as BD          # noqa: E402
import main_stress_test as arm        # noqa: E402

CG = {9: False, 8: True, 11: True, 7: True, 5: True, 4: False}


def pasos_a_ang(g, centros):
    return [arm.dynamixel_a_rad(g[s], centros[s], CG[s]) for s in arm.SERVO_IDS_BRAZO]


class G:       # grano mínimo
    def __init__(s, dx=0.0, dist=0.32, cat="MADURO"):
        s.dx_mm, s.dy_mm, s.dist_m, s.categoria = dx, 0.0, dist, cat
        s.track_id, s.clase = 1, "ROJO"


def esperar(b, t=30.0):
    t0 = time.time()
    while b.estado() == "OCUPADO" and time.time() - t0 < t:
        time.sleep(0.02)
    return b.estado()


def nuevo():
    LOG.clear()
    return BD.BrazoDynamixel(mujoco=False)


def main():
    # ---- 1) arranque: debe llevar el brazo a la postura de trabajo (PRENDERSE) -------------
    b = nuevo()
    objetivo = arm.obtener_angulos_desde_tabla(arm.OFFSETS_PRENDERSE, CG)
    ultimo = [e for e in LOG if e[0] == "goal"][-1][1]
    err = max(abs(x - y) for x, y in zip(pasos_a_ang(ultimo, b.centros), objetivo))
    assert err < math.radians(0.2), f"no llegó a la postura de trabajo (err {math.degrees(err):.2f}°)"
    print("[1] postura de trabajo al iniciar: OK")

    # ---- 2) rechazos con motivo -------------------------------------------------------------
    assert not b.iniciar_recogida(G(dist=None)) and "calibrar" in b.ultimo_error()
    assert not b.iniciar_recogida(G(dist=0.9)) and "alcance" in b.ultimo_error()
    assert not b.iniciar_recogida(G(dx=400)) and "lateral" in b.ultimo_error()
    assert b.estado() == "LIBRE"
    print("[2] rechazos con motivo: OK ->", b.ultimo_error())

    # ---- 3) secuencia completa (tiempos acelerados x10) ------------------------------------
    orig = b._move
    b._move = lambda a, g, d: orig(a, g, d / 10.0)
    assert b.iniciar_recogida(G(dx=-30)) and b.estado() == "OCUPADO"
    assert esperar(b) == "OK", b.ultimo_error()
    b.reconocer()
    fin = [e for e in LOG if e[0] == "goal"][-1][1]
    err = max(abs(x - y) for x, y in zip(pasos_a_ang(fin, b.centros), objetivo))
    assert err < math.radians(0.5), "no volvió a la postura de trabajo"
    cierres = [e[1][arm.SERVO_ID_GRIPPER] for e in LOG if e[0] == "goal"]
    assert max(abs(c - b.centros[4]) for c in cierres) > 600, "la garra nunca cerró"
    print("[3] recogida completa y regreso a la postura de trabajo: OK")
    b._move = orig

    # ---- 4) cancelación inmediata ------------------------------------------------------------
    assert b.iniciar_recogida(G())
    time.sleep(0.5)                                    # a mitad del primer movimiento (2.0 s)
    n0 = len(LOG)
    t0 = time.time()
    b.cancelar()
    dt = time.time() - t0
    time.sleep(0.3)
    nuevos = len(LOG) - n0
    assert b.estado() == "FALLO" and dt < 0.3, (b.estado(), dt)
    assert nuevos <= 3, f"el brazo siguió moviéndose tras cancelar ({nuevos} paquetes)"
    assert "cancelada" in b.ultimo_error()
    print(f"[4] cancelación: brazo detenido en {dt * 1000:.0f} ms ({nuevos} paquetes tras cancelar): OK")
    b.reconocer()

    # ---- 5) cerrar(): reposo antes de soltar el torque ----------------------------------------
    LOG.clear()
    b._move = lambda a, g, d: orig(a, g, d / 10.0)     # acelerar el estacionamiento en la prueba
    b.cerrar()
    b.cerrar()                                         # idempotente
    idx_goal = max(i for i, e in enumerate(LOG) if e[0] == "goal")
    idx_torque = min(i for i, e in enumerate(LOG) if e[0] == "torque" and e[2] == 0)
    assert idx_goal < idx_torque, "soltó el torque antes de estacionar"
    reposo = arm.obtener_angulos_desde_tabla(arm.OFFSETS_APAGARSE, CG)
    err = max(abs(x - y) for x, y in zip(pasos_a_ang(LOG[idx_goal][1], b.centros), reposo))
    assert err < math.radians(0.5), "no llegó a la postura de reposo"
    assert {e[1] for e in LOG if e[0] == "torque" and e[2] == 0} == set(arm.TODOS_LOS_SERVOS)
    print("[5] cerrar(): estaciona en reposo y luego suelta el torque: OK")
    print("OK test_brazo_dynamixel")


if __name__ == "__main__":
    main()
