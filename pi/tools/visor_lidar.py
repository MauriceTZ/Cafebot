"""Visor del LD06 en el sistema del robot (x adelante = arriba en pantalla).

    python3 -m tools.visor_lidar --puerto /dev/cafebot-lidar
Rueda del mouse: zoom.  F11: pantalla completa sí/no.  Necesita escritorio (pygame).
Lanzado por SSH con la ventana en la pantalla de la Raspberry (HDMI), se abre en pantalla completa (--pantalla).
"""
import argparse

import pygame as pg

from cafebot.drivers.ld06 import LD06, ConfigLD06
from tools import pantalla


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--puerto", default="/dev/cafebot-lidar")
    ap.add_argument("--yaw", type=float, default=0.0, help="grados: hacia dónde mira el 0° del LiDAR")
    pantalla.agregar_opcion(ap)
    a = ap.parse_args()
    lidar = LD06(a.puerto, ConfigLD06(yaw_deg=a.yaw))
    lidar.iniciar()
    ventana = pantalla.Pantalla("Cafebot - LiDAR", (800, 800), a.pantalla)
    reloj = pg.time.Clock()
    px_por_m = 150.0
    fuente = pg.font.SysFont(None, 22)
    while True:
        for ev in pg.event.get():
            if ventana.tecla(ev):
                continue
            if ev.type == pg.QUIT:
                lidar.detener()
                return
            if ev.type == pg.MOUSEWHEEL:
                px_por_m = max(20, min(800, px_por_m * (1.1 ** ev.y)))
        pan = ventana.sup
        W, H = pan.get_size()
        pan.fill((15, 15, 20))
        cx, cy = W // 2, H // 2
        for r in range(1, 8):
            pg.draw.circle(pan, (40, 40, 50), (cx, cy), int(r * px_por_m), 1)
        pg.draw.polygon(pan, (0, 180, 255), [(cx, cy - 12), (cx - 7, cy + 8), (cx + 7, cy + 8)])
        scan = lidar.ultimo_scan()
        if scan is not None:
            x, y = scan.xy()
            for xi, yi in zip(x, y):
                pan.set_at((int(cx - yi * px_por_m), int(cy - xi * px_por_m)), (255, 255, 255))
            st = lidar.estadisticas()
            pan.blit(fuente.render(f"{len(x)} pts  {scan.rpm:.0f} rpm  CRC malos: {st['crc_malos']}  "
                                   f"1 anillo = 1 m", True, (200, 200, 200)), (10, 10))
        pg.display.flip()
        reloj.tick(30)


if __name__ == "__main__":
    main()
