"""Prueba la localización inicial con el LD06 REAL (robot quieto en un sector del
beneficiadero, mirando a los árboles).

    python3 -m tools.localizar --puerto /dev/cafebot-lidar
    python3 -m tools.localizar --sim        # en una ronda simulada al azar
"""
import argparse
import math

import numpy as np

from cafebot.drivers.ld06 import LD06, ConfigLD06
from cafebot.localizacion.inicio import estimar_inicio


def mostrar(r):
    print(f"Zona de partida: {r.zona}   pose x={1000*r.x:.0f} mm  y={1000*r.y:.0f} mm  θ={math.degrees(r.th):.1f}°")
    print(f"Puntos sobre los árboles: {r.inliers}  (error rms {1000*r.error_rms:.1f} mm)  confianza {r.confianza:.2f}"
          + ("   <-- BAJA: las albercas tapan los árboles, confirmar con la cámara" if r.confianza < 0.15 else ""))
    print(f"Albercas en las posiciones: {r.albercas or 'ninguna'}   (puntos por posición: {r.puntos_alberca})")
    print(f"Cajas vistas en los sectores: {r.zonas_con_caja or 'ninguna'}  (una puede tapar a la otra)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="/dev/cafebot-lidar")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--vueltas", type=int, default=3, help="vueltas a juntar (más puntos)")
    a = ap.parse_args()
    if a.sim:
        from cafebot.mundo import cancha
        from cafebot.mundo.simulador_lidar import simular_scan
        rng = np.random.default_rng()
        esc = cancha.Escenario.sorteo(rng)
        real = (cancha.ZONA_X_CENTRO, cancha.centros_zonas_beneficiadero()[esc.zona_robot - 1], math.radians(3))
        print("Ronda simulada:", esc)
        mostrar(estimar_inicio(simular_scan(real, esc.segmentos(), rng=rng)))
        return
    lidar = LD06(a.puerto, ConfigLD06())
    lidar.iniciar()
    try:
        scans = [s for s in (lidar.esperar_scan(2.0) for _ in range(a.vueltas)) if s is not None]
        if not scans:
            raise SystemExit("No llegan datos del LD06")
        from cafebot.drivers.ld06 import Scan
        junto = Scan(scans[-1].t, np.concatenate([s.angulos for s in scans]),
                     np.concatenate([s.distancias for s in scans]), np.concatenate([s.intensidades for s in scans]))
        mostrar(estimar_inicio(junto))
    finally:
        lidar.detener()


if __name__ == "__main__":
    main()
