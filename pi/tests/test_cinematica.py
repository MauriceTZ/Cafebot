"""IK de mínimo esfuerzo vs IK antigua, en los puntos del árbol. Sin hardware.
Correr desde pi/:  python3 -m tests.test_cinematica
"""
import math

import numpy as np

from cafebot.arm.cinematica import ModeloBrazo, directa, inversa, inversa_legado, torques_gravedad, costo

BASE_BRAZO_Z = 0.36                       # base del brazo sobre el suelo
NOMBRES = ["hombro", "codo1", "codo2", "muñeca"]
FILAS = {"baja": 0.407, "media": 0.557, "alta": 0.707}   # centro del grano sobre el suelo


def main():
    m = ModeloBrazo()
    # 1) IK -> FK debe volver al mismo punto
    for r, z, p in [(0.12, 0.10, math.pi / 2), (0.15, 0.25, math.radians(70)), (0.08, 0.30, math.radians(60))]:
        s = inversa(m, r, z, p)
        assert s is not None, (r, z)
        rr, zz, pp = directa(m, s.q)
        assert abs(rr - r) < 1e-6 and abs(zz - z) < 1e-6 and abs(pp - p) < 1e-6
    # 2) Postura estirada horizontal = la prueba que falló
    tau = torques_gravedad(m, np.array([math.pi / 2, 0, 0, 0]))
    print(f"Estirado horizontal: hombro {tau[0]:.2f} N·m = {100 * tau[0] / m.tau_max:.0f}% del bloqueo\n")

    print(f"{'fila':6} {'r[cm]':>5} | {'IK antigua':>22} | {'mín. esfuerzo, garra 90°':>26} | {'mín. esfuerzo, garra libre 55-95°':>34}")
    for fila, zg in FILAS.items():
        z = zg - BASE_BRAZO_Z
        for r in (0.10, 0.13, 0.16):
            def fmt(q, pitch=None):
                if q is None:
                    return "inalcanzable"
                t = np.abs(torques_gravedad(m, q))
                j = int(np.argmax(t))
                extra = f" garra {math.degrees(pitch):.0f}°" if pitch is not None else ""
                return f"máx {100 * t[j] / m.tau_max:3.0f}% ({NOMBRES[j]}){extra}"
            ql = inversa_legado(m, r, z, math.radians(94), 0.93)
            rr, zz, _ = directa(m, ql)
            leg = fmt(ql) if abs(rr - r) < 2e-3 and abs(zz - z) < 2e-3 else "inalcanzable"
            s90 = inversa(m, r, z, math.pi / 2)
            slib = inversa(m, r, z, (math.radians(55), math.radians(95)))
            print(f"{fila:6} {r*100:5.0f} | {leg:>22} | {fmt(s90.q if s90 else None):>26} | "
                  f"{fmt(slib.q if slib else None, slib.pitch if slib else None):>34}")
    print("\nOK: cinemática")


if __name__ == "__main__":
    main()
