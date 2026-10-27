"""Prueba del filtro de autodetección con paquetes LD06 simulados y SLAM real.
  1) Con el chasis visible, SIN filtro hay puntos propios en el scan; CON filtro no.
  2) Efecto en el SLAM: el robot avanza 0.6 m; se compara la pose estimada con y sin filtro."""
import _stubs  # noqa: F401
import math
import numpy as np
import ld06_slam_nav_rpi_imu_integrado as M
from sim_lidar import ray_scan, packets_from_scan

# chasis "real" algo más grande que lo nominal 23x30 (ruedas, tornillos): hx=.12 hy=.16
CHASIS_REAL = (0.12, 0.16)


def scan_via_assembler(pose, footprint, rng):
    asm = M.ScanAssembler(footprint)
    for _ in range(2):    # dos vueltas: la segunda cierra la primera
        a, d = ray_scan(pose, chasis=CHASIS_REAL, rng=rng)
        for p in packets_from_scan(a, d):
            asm.feed(p)
    sid, scan = asm.get_latest_scan()
    return asm, scan


def en_huella(scan, fp):
    x = np.sin(scan[:, 0]) * scan[:, 1]
    y = np.cos(scan[:, 0]) * scan[:, 1]
    return int(sum(fp.contains(a, b) for a, b in zip(x, y)))


def main():
    rng = np.random.default_rng(1)
    fp = M.RobotFootprint(0.30, 0.23, 0.03)
    print(f"Huella: semiancho X={fp.hx:.3f} m, semilargo Y={fp.hy:.3f} m, radio circunscrito={fp.circumradius:.3f} m")

    _, sin_f = scan_via_assembler((0, 0, 0), None, rng)
    asm, con_f = scan_via_assembler((0, 0, 0), fp, rng)
    p_sin, p_con = en_huella(sin_f, fp), en_huella(con_f, fp)
    print(f"[1] puntos dentro de la huella  SIN filtro: {p_sin} de {len(sin_f)} | CON filtro: {p_con} de {len(con_f)} "
          f"(descartados {asm.self_last_rev}/vuelta)")
    assert p_sin > 20 and p_con == 0
    # que no se hayan perdido las paredes: puntos lejanos intactos
    lejos_sin = int((sin_f[:, 1] > 0.5).sum())
    lejos_con = int((con_f[:, 1] > 0.5).sum())
    print(f"    puntos del entorno (>0.5 m): sin={lejos_sin} con={lejos_con}")
    assert lejos_con == lejos_sin

    # --- 2) SLAM: avance de 0.6 m en +Y (30 pasos de 2 cm) ---
    # Efecto real de la autodetección: el chasis se "pega" al mapa como obstáculo ESTATICO en el
    # punto de partida (aunque el robot ya se fue), y el planificador lo ve como pared.
    resultados = {}
    for etiqueta, f in (("SIN filtro", None), ("CON filtro", fp)):
        rng = np.random.default_rng(2)
        slam = M.LidarSLAM2D(map_size_m=12.0, resolution_m=0.04)
        for k in range(0, 31):
            pose = (0.0, 0.02 * k, 0.0)
            _, scan = scan_via_assembler(pose, f, rng)
            slam.step(scan)
        ys, xs = np.nonzero(slam.static_mask())
        wx = (xs - slam.center_cell + 0.5) * slam.res
        wy = (ys - slam.center_cell + 0.5) * slam.res
        # zona alrededor del punto de partida (0,0) que en el mundo real esta VACIA (el robot ya se fue)
        fantasma = int(np.sum(np.hypot(wx - 0.0, wy - 0.0) < 0.19))
        # lo que ve el robot al final: celdas estaticas a menos de 0.3 m de su posicion actual
        pegadas = int(np.sum(np.hypot(wx - slam.pose[0], wy - slam.pose[1]) < 0.19))
        resultados[etiqueta] = (fantasma, pegadas)
        print(f"[2] {etiqueta}: pose SLAM final=({slam.pose[0]:+.3f}, {slam.pose[1]:+.3f}) m (real y=+0.600) | "
              f"celdas ESTATICAS fantasma en el punto de partida: {fantasma} | pegadas al robot: {pegadas}")
    assert resultados["SIN filtro"][0] > 5, "sin filtro deberia dejar obstaculos fantasma"
    assert resultados["CON filtro"][0] == 0 and resultados["CON filtro"][1] == 0


if __name__ == "__main__":
    main()
    print("OK test_autofiltro")
