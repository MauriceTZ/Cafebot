"""La línea del cultivo con los sensores DELANTEROS (frente al árbol), SIN robot.   python3 -m tests.test_linea_arbol

  1. Geometría: con los dos puntos de cruce (DI y DD) sale la distancia del robot a la línea y cuánto está girado.
  2. Firmware real simulado + sensores de línea simulados: el robot parte torcido (0°, +6°, −10°) a ~20 cm de una
     línea; Bordes.medir_adelante la toca con un sensor, sigue hasta el otro y mide distancia y ángulo; girando
     −ángulo queda de frente. Se compara con la verdad (la odometría simulada, sin error).
"""
import math
import time

import numpy as np

from cafebot.config import cargar
from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
from cafebot.drivers.mega import Mega
from cafebot.mundo.cancha import Rect
from cafebot.navegacion.bordes import Bordes, medida_linea
from cafebot.navegacion.geometria import transformar
from tests.sim_serial import SerialSimulado, compilar

LINEA_X = 0.40             # centro de la línea (19 mm): el borde cercano está en 0,3905


def prueba_geometria():
    for th_deg, x in ((0.0, 0.10), (5.0, 0.0), (-12.0, 0.25)):
        th = math.radians(th_deg)
        pose = np.array([x, 0.3, th])
        # dónde cruza cada sensor la recta x = 0.6 avanzando en línea recta desde `pose`
        cruces = []
        for py in (0.1085, -0.1085):
            s0 = transformar(np.array([[0.1115, py]]), pose)[0]
            t = (0.6 - s0[0]) / math.cos(th)
            cruces.append(s0 + t * np.array([math.cos(th), math.sin(th)]))
        m = medida_linea(cruces[0], cruces[1], pose)
        assert abs(m.dist_m - (0.6 - x)) < 1e-9 and abs(math.degrees(m.angulo) - th_deg) < 1e-9, (m, th_deg)
    print("  distancia y ángulo exactos desde los dos cruces (0°, +5°, −12°)")


def prueba_sim(cfg):
    compilar()
    peor_d, peor_a = 0.0, 0.0
    for th0 in (0.0, 6.0, -10.0):
        mega = Mega("sim")
        mega.conectar(serial_obj=SerialSimulado())
        mega.telemetria(50)
        mega.fijar_pose(0, 0, th0)
        ir = IRSimulado(mega, cfg.ir.posiciones, [Rect(LINEA_X, 0.0, 0.019, 3.0)],
                        lambda: pose_mega_extrapolada(mega)).iniciar()
        time.sleep(0.3)
        b = Bordes(mega, cfg, log=lambda t: None)
        try:
            med = b.medir_adelante(400, vel=100)
            assert med is not None, f"no midió la línea partiendo torcido {th0}°"
            x, y, th = mega.pose()
            dist_real = (LINEA_X - 0.0095) - x / 1000.0            # el robot ve la línea de frente: perpendicular
            err_d, err_a = abs(med.dist_m - dist_real), abs(math.degrees(med.angulo - th))
            mega.girar(-math.degrees(med.angulo), timeout=10)
            th_final = math.degrees(mega.pose()[2])
            print(f"  torcido {th0:+.0f}°: midió {100 * med.dist_m:.1f} cm (real {100 * dist_real:.1f}) y "
                  f"{math.degrees(med.angulo):+.1f}° (real {math.degrees(th):+.1f}°); enderezado queda a {th_final:+.1f}°")
            assert err_d < 0.004 and err_a < 0.8 and abs(th_final) < 1.0, (err_d, err_a, th_final)
            peor_d, peor_a = max(peor_d, err_d), max(peor_a, err_a)
        finally:
            b.cerrar()
            ir.detener()
            mega.cerrar()
    print(f"  peor error: {1000 * peor_d:.1f} mm y {peor_a:.2f}°")


def main():
    cfg = cargar()
    print("1) Geometría de los dos cruces")
    prueba_geometria()
    print("2) Firmware + sensores simulados")
    prueba_sim(cfg)
    print("OK: línea del cultivo")


if __name__ == "__main__":
    main()
