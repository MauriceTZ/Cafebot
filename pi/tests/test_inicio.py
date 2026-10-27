"""Localización inicial con LiDAR en rondas simuladas (Monte Carlo).
Correr desde pi/:  python3 -m tests.test_inicio [n_rondas]
"""
import math
import sys
import time

import numpy as np

from cafebot.drivers.ld06 import ConfigLD06
from cafebot.localizacion.inicio import estimar_inicio
from cafebot.mundo import cancha
from cafebot.mundo.simulador_lidar import simular_scan


def main(n=100, semilla=7):
    rng = np.random.default_rng(semilla)
    cfg = ConfigLD06()
    zonas_y = cancha.centros_zonas_beneficiadero()
    ok_zona = ok_pose = ok_alb = ok_caja = 0
    errores, tiempos, vistos_caja = [], [], []
    baja_conf = baja_conf_mal = 0
    for i in range(n):
        esc = cancha.Escenario.sorteo(rng)
        # El capitán lo deja "centrado y mirando a los árboles", con error humano
        real = (cancha.ZONA_X_CENTRO + rng.uniform(-0.06, 0.06),
                zonas_y[esc.zona_robot - 1] + rng.uniform(-0.06, 0.06),
                math.radians(rng.uniform(-8, 8)))
        scan = simular_scan(real, esc.segmentos(), cfg, rng)
        t0 = time.perf_counter()
        r = estimar_inicio(scan)
        tiempos.append(time.perf_counter() - t0)
        e = math.hypot(r.x - real[0], r.y - real[1])
        eth = abs(math.degrees(r.th - real[2]))
        bien_zona = r.zona == esc.zona_robot
        bien_pose = bien_zona and e < 0.03 and eth < 1.5
        bien_alb = sorted(r.albercas) == sorted(esc.albercas)
        # Desde la partida una caja puede tapar a la otra: se exige que NO invente cajas
        bien_caja = set(r.zonas_con_caja) <= {esc.zona_caja_roja, esc.zona_caja_azul}
        vistos_caja.append(len(r.zonas_con_caja))
        ok_zona += bien_zona; ok_pose += bien_pose; ok_alb += bien_alb; ok_caja += bien_caja
        if bien_zona:
            errores.append((e, eth))
        if r.confianza < 0.15:
            baja_conf += 1
            baja_conf_mal += not bien_pose
        if not (bien_pose and bien_alb):
            print(f"  ronda {i}: {esc} real=({real[0]:.3f},{real[1]:.3f},{math.degrees(real[2]):.1f}°) "
                  f"-> zona {r.zona} ({r.x:.3f},{r.y:.3f},{math.degrees(r.th):.1f}°) conf={r.confianza:.2f} "
                  f"alb={r.albercas} diag={r.puntos_alberca} cajas={r.zonas_con_caja}")
    e = np.array(errores)
    print(f"\nzona correcta {ok_zona}/{n} | pose <3 cm y <1,5° {ok_pose}/{n} | albercas {ok_alb}/{n} | "
          f"cajas sin falsos positivos {ok_caja}/{n} (vio en promedio {np.mean(vistos_caja):.1f} de 2)")
    print(f"rondas con confianza baja (<0,15): {baja_conf}; de ellas mal localizadas: {baja_conf_mal}. "
          f"Mal localizadas con confianza alta: {n - ok_pose - baja_conf_mal}")
    if len(e):
        print(f"error posición mediana {1000*np.median(e[:,0]):.1f} mm, máx {1000*e[:,0].max():.1f} mm; "
              f"ángulo mediana {np.median(e[:,1]):.2f}°, máx {e[:,1].max():.2f}°")
    print(f"tiempo por estimación: {1000*np.median(tiempos):.0f} ms en este PC")
    return ok_zona == n


if __name__ == "__main__":
    main(int(sys.argv[1]) if len(sys.argv) > 1 else 100)
