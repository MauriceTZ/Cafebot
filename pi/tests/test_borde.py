"""Recorrido del borde de la cancha SIN robot: firmware real simulado + LiDAR y sensores de línea simulados en una
pista de taller (rectángulo de huincha dentro de una sala).   python3 -m tests.test_borde

  1. Filtro del LiDAR: lo que cae en el cuadrado de 45 cm detrás del LiDAR (el propio robot) se descarta.
  2. El robot parte torcido en una esquina, recorre los 4 bordes (el borde a la derecha, tocándolo con los IR) y
     vuelve: mide el rectángulo con los toques y el error de cierre de la odometría y de la pose fusionada.
"""
import math
import time

import numpy as np

from cafebot.config import cargar
from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
from cafebot.drivers.ld06 import ConfigLD06, mascara_huella
from cafebot.drivers.lidar_simulado import LD06Simulado
from cafebot.drivers.mega import Mega
from cafebot.mundo import taller
from cafebot.navegacion.bordes import RecorridoBorde
from cafebot.navegacion.sistema import SistemaNavegacion
from tests.sim_serial import SerialSimulado, compilar

LARGO, ANCHO = 1.5, 1.1


def prueba_filtro(cfg):
    g = cfg.geometria
    c: ConfigLD06 = g.config_ld06()
    (x0, x1), (y0, y1) = g.huella()
    print(f"  cuadrado del propio robot: x {x0:+.3f}..{x1:+.3f}, y {y0:+.3f}..{y1:+.3f} m (LiDAR en x={g.lidar_x})")
    assert abs((g.lidar_x - x0) - 0.45) < 1e-9 and abs((y1 - y0) - 0.45) < 1e-9
    pts = np.array([[0.0, 0.0], [-0.25, 0.20], [g.lidar_x - 0.02, 0.21],     # base, rueda/brazo: propios
                    [g.lidar_x + 0.05, 0.0], [0.0, 0.30], [-0.40, 0.0]])     # delante del LiDAR, al lado, detrás
    m = mascara_huella(pts[:, 0], pts[:, 1], c)
    assert list(m) == [True, True, True, False, False, False], m


def prueba_recorrido(cfg):
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    real0 = taller.inicio(torcido_deg=4.0)
    lidar = LD06Simulado(cfg.geometria.config_ld06(), taller.segmentos(LARGO, ANCHO), real0,
                         lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th), semilla=3)
    ir = IRSimulado(mega, cfg.ir.posiciones, taller.lineas(LARGO, ANCHO),
                    lambda: lidar.pose_real_ahora(pose_mega_extrapolada(mega)), semilla=3).iniciar()
    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega)
    log = []
    try:
        fin = time.monotonic() + 5
        while not nav.ekf.inicializada and time.monotonic() < fin:
            nav.actualizar()
            time.sleep(0.02)
        rb = RecorridoBorde(mega, cfg, pose_mapa_fn=nav.pose, log=log.append, vel=150, sep_m=0.10, tramo_m=0.6)
        rb.iniciar()
        t0 = time.monotonic()
        reales = []
        while rb.activo and time.monotonic() - t0 < 600:
            nav.actualizar()
            if len(rb.esquinas) > len(reales):
                reales.append(lidar.pose_real_ahora(pose_mega_extrapolada(mega)))
            time.sleep(0.02)
        while len(reales) < len(rb.esquinas):          # la última esquina: el robot ya no se mueve
            reales.append(lidar.pose_real_ahora(pose_mega_extrapolada(mega)))
        print("  " + "\n  ".join(l for l in log if "[borde]" in l))
        assert rb.estado.startswith("VUELTA COMPLETA"), rb.estado
        r = rb.resultado
        print(f"  {time.monotonic() - t0:.0f} s, {len(rb.toques)} toques")
        # El rectángulo medido con los toques (mapa = LiDAR + odometría)
        for marco in ("mapa", "mega"):
            rr = r["rect_" + marco]
            print(f"  rectángulo ({marco}): lados {', '.join(f'{x:.3f}' for x in rr['largos'])} m "
                  f"(real {LARGO} × {ANCHO}), ángulos {', '.join(f'{a:.1f}' for a in rr['angulos'])}°, "
                  f"residuo máx {1000 * rr['residuo_max']:.1f} mm")
        rm = r["rect_mapa"]
        assert all(abs(a - b) < 0.05 for a, b in zip(rm["largos"], (LARGO, ANCHO, LARGO, ANCHO))), rm["largos"]
        assert all(abs(a - 90) < 2.5 for a in rm["angulos"]), rm["angulos"]
        # Error de cierre: la pose final debería ser la de partida (la verdad: la pose REAL del simulador)
        d_real = np.hypot(*(reales[4][:2] - reales[0][:2]))
        th_real = math.degrees((reales[4][2] - reales[0][2] + math.pi) % (2 * math.pi) - math.pi)
        print(f"  cierre REAL (simulador): {100 * d_real:.1f} cm, {th_real:+.1f}° | odometría dice {100 * r['mega']['error_m']:.1f} cm "
              f"| LiDAR+odometría {100 * r['mapa']['error_m']:.1f} cm, {r['mapa']['error_deg']:+.1f}°")
        assert d_real < 0.03, d_real                         # los bordes la traen de vuelta a la esquina
        assert r["mapa"]["error_m"] < 0.06 and abs(r["mapa"]["error_deg"] - th_real) < 2.0
        # La odometría sola deriva (el simulador le pone 3 % de error de distancia y 2 % de giro)
        assert r["mega"]["error_m"] > r["mapa"]["error_m"]
    finally:
        ir.detener()
        lidar.detener()
        mega.cerrar()


def main():
    compilar()
    cfg = cargar()
    print("1) Filtro del propio robot en el LiDAR")
    prueba_filtro(cfg)
    print("2) Recorrido del borde de una pista de taller")
    prueba_recorrido(cfg)
    print("OK: recorrido del borde")


if __name__ == "__main__":
    main()
