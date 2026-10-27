"""Odometría del EKF (escala en metros) y sensores IR (gpiozero simulado). Sin hardware."""
import _stubs  # noqa: F401
import math
import sys
import types

import ld06_slam_nav_rpi_imu_integrado as M
import sensores_ir as IR


def test_odometria():
    tpm = 800.0 / (math.pi * 0.053)                      # ticks por metro de avance
    e = M.OdometryEKF()
    e.initialize([0.0, 0.0, 0.0], (0, 0, 0, 0, 0))
    e.predict_encoders((round(tpm),) * 4 + (0,))
    assert abs(e.pose[1] - 1.0) < 0.01 and abs(e.pose[0]) < 1e-9, e.pose
    # giro puro (postura 1): M1,M3 +, M2,M4 - ; una vuelta completa = pi*D ticks por rueda
    e = M.OdometryEKF()
    e.initialize([0.0, 0.0, 0.0], (0, 0, 0, 0, 1))
    t = round(tpm * math.pi * M.OdometryEKF.TURN_DIAGONAL_M / 4.0)          # 1/4 de vuelta
    e.predict_encoders((t, -t, t, -t, 1))
    assert abs(e.pose[2] - math.pi / 2) < 0.02 and math.hypot(e.pose[0], e.pose[1]) < 1e-9, e.pose
    # avance a 90 grados (mira a -X)
    e = M.OdometryEKF()
    e.initialize([0.0, 0.0, math.pi / 2], (0, 0, 0, 0, 0))
    e.predict_encoders((round(tpm * 0.5),) * 4 + (0,))
    assert abs(e.pose[0] + 0.5) < 0.01 and abs(e.pose[1]) < 0.01, e.pose
    print("OK odometria EKF: 1 m de ticks = 1 m de pose; giro y rumbo correctos")


def test_ir():
    creados = []

    class DigitalInputDevice:
        def __init__(self, pin, pull_up=True, active_state=None, **k):
            # Réplica de la regla de gpiozero: pull_up=None EXIGE active_state
            if pull_up is None and active_state is None:
                raise RuntimeError("floating pin sin active_state")
            self.pin, self.value = pin, 0
            creados.append(self)

        def close(self):
            pass

    sys.modules["gpiozero"] = types.SimpleNamespace(DigitalInputDevice=DigitalInputDevice)
    # deshabilitado por defecto: no toca GPIO
    s = IR.SensoresIRBorde(enabled=True)
    assert not s.enabled and not creados and set(s.leer().values()) == {None}
    assert "IR_ENABLED=False" in s.estado_texto()
    IR.IR_ENABLED = True
    s = IR.SensoresIRBorde(enabled=True)
    assert s.enabled and len(creados) == 4
    creados[1].value = 1                      # frontal_derecho ve el borde
    assert s.hay_borde() == ["frontal_derecho"] and "BORDE" in s.estado_texto()
    s.cerrar()
    IR.IR_ENABLED = False
    print("OK sensores IR: sin GPIO por defecto; con IR_ENABLED crea los 4 con active_state y lee el borde")


if __name__ == "__main__":
    test_odometria()
    test_ir()
    print("OK test_odometria_ir")
