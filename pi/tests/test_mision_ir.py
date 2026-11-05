"""Misión SIN LiDAR (rama navegacion-ir), sin robot.   python3 -m tests.test_mision_ir [--rapido]

  1. Carril entre albercas: el centro libre más cercano al árbol, con margen y dentro de lo que se ve.
  2. Albercas con la cámara dibujada (vision/render.py): los bordes en y salen a ~2 cm de la verdad.
  3. Foto del árbol sin LiDAR: con la y del árbol errada hasta ±10 cm y la distancia ±5 cm, la búsqueda fina la
     encuentra (a pocos mm) y no clasifica mal ningún grano recogible.
  4. (sin --rapido) Una ronda completa simulada: firmware del Mega + IR + cámara dibujada + brazo simulado con la
     tolerancia real de la garra; partida en diagonal hacia el árbol, 2 albercas en el camino. Recoge, no pisa el
     perímetro ni el borde de una alberca y no las choca.
"""
import math
import sys
import time

import numpy as np

from cafebot.config import cargar
from cafebot.mision.mision_ir import carril
from cafebot.mundo import cancha
from cafebot.vision.camara_modelo import CamaraPinhole
from cafebot.vision.render import MundoCancha, dibujar
from cafebot.vision.vision_ir import albercas_en_imagen


def prueba_carril():
    vis = (-0.55, 0.50)
    assert abs(carril([], vis, 0.10, 0.195, -1, 1) - 0.10) < 0.006           # libre: derecho al árbol
    assert abs(carril([], vis, 0.40, 0.195, -1, 1) - 0.305) < 0.006          # sin salirse de lo que se ve
    assert carril([(-0.36, 0.36)], vis, 0.0, 0.195, -1, 1) is None           # no cabe al lado: no inventa
    assert carril([(-0.60, 0.52)], vis, 0.0, 0.195, -1, 1) is None           # todo azul a la vista
    y = carril([(0.05, 0.70)], vis, 0.30, 0.195, -1, 1)                      # alberca a la derecha del árbol
    assert y is not None and y + 0.195 < 0.05 and y > -0.17, y
    assert carril([(0.05, 0.70)], vis, 0.30, 0.195, 0.0, 1) is None          # y fuera del límite del perímetro
    print("  carril libre: con margen, dentro de lo visible, el más cercano al árbol")


def prueba_albercas(cfg):
    cc = cfg.camara
    cam = CamaraPinhole.desde_config(cc, cfg.mision_ir.inclinacion_albercas_deg)
    peor = 0.0
    for alb, y_real in (((2, 3), 1.80), ((2, 3), 1.084), ((2, 3), 0.70), ((1, 4, 5), 2.99)):
        esc = cancha.Escenario(3, 1, 5, alb, {})
        img = dibujar(MundoCancha(esc), (0.85, y_real, math.radians(1.0)), cam.con(dist=np.zeros(5)))
        azules, vis = albercas_en_imagen(img, cam, (0.85, 0.0, 0.0))
        verdad = []
        for a in sorted(alb):                       # contiguas = un solo tramo
            lo, hi = (a - 1) * 0.72 - y_real, a * 0.72 - y_real
            if verdad and abs(verdad[-1][1] - lo) < 1e-6:
                verdad[-1][1] = hi
            else:
                verdad.append([lo, hi])
        for v0, v1 in verdad:
            v0c, v1c = max(v0, vis[0]), min(v1, vis[1])
            if v0c >= v1c:
                continue
            tramo = next((t for t in azules if t[0] <= v1c and t[1] >= v0c), None)
            assert tramo is not None, (alb, y_real, azules, verdad)
            # agrandar es seguro (la pared que mira a la cámara se proyecta hacia ella); achicar, no
            if v0 > vis[0]:
                assert tramo[0] < v0 + 0.015, (alb, y_real, tramo, v0)
                peor = max(peor, v0 - tramo[0])
            if v1 < vis[1]:
                assert tramo[1] > v1 - 0.015, (alb, y_real, tramo, v1)
                peor = max(peor, tramo[1] - v1)
    assert peor < 0.06, peor
    print(f"  albercas vistas con la cámara dibujada: nunca más chicas; más anchas {100 * peor:.1f} cm como mucho")


def prueba_arbol(cfg):
    from cafebot.vision import clasificador as cl
    from cafebot.vision.vision_ir import VisionIRCamara
    cc = cfg.camara
    cc.fotos, cc.guardar_fotos = 1, False

    class CamFija:
        img = None

        def capturar(self):
            return self.img

        def enfocar(self, d):
            pass
    cf = CamFija()
    v = VisionIRCamara(cfg, camara=cf, clasificador=cl.Clasificador(), log=lambda t: None)
    yt = cancha.centros_arboles()[1]
    x_foto = cancha.ARBOL_X_FRENTE - cfg.mision.distancia_foto_m - cfg.geometria.largo / 2
    peor, malos = 0.0, 0
    for semilla, (ey, ex, ep) in enumerate(((0.0, 0.0, 0), (0.05, 0.04, 0), (-0.08, -0.05, 2), (0.10, 0.0, -2))):
        esc = cancha.Escenario(3, 1, 5, (), cancha.sorteo_granos(np.random.default_rng(semilla)))
        real = (x_foto + ex, yt - cc.y + ey, 0.0)
        cf.img = dibujar(MundoCancha(esc), real, CamaraPinhole.desde_config(cc, cc.inclinacion_arbol_deg + ep)
                         .con(dist=np.zeros(5)))
        tab, lect = v.arbol((cancha.ARBOL_X_FRENTE - x_foto, cc.y, 0.0))
        assert tab is not None, (ey, ex, ep)
        peor = max(peor, abs(tab[1] - (yt - real[1])))
        malos += sum(1 for k, (c, _) in lect.items()
                     if c in (cancha.MADURO, cancha.SOBREMADURO) and c != esc.granos.get((2,) + k, ("VACIO",))[0])
    assert peor < 0.01 and malos == 0, (peor, malos)
    print(f"  foto del árbol con la y errada hasta ±10 cm: la encuentra a {1000 * peor:.1f} mm; ningún grano "
          "recogible mal clasificado")


def prueba_ronda(cfg):
    from cafebot.mision import interfaces as I
    from cafebot.mision.mision_ir import MisionIR
    import tools.mision_ir as T
    esc, mega, verdad, vision, brazo, arbol = T.armar_sim(cfg, 7, 2, (1, 2), t_agarre=1.0, log=lambda t: None)
    m = MisionIR(cfg, mega, brazo, vision, I.Boton(), gyro=verdad, log=lambda t: None)
    try:
        m.iniciar()
        fin = time.monotonic() + 600
        while not m.terminada and time.monotonic() < fin:
            time.sleep(0.2)
        assert m.terminada, "la misión no terminó"
        pts = T.puntaje(brazo.depositos)
        print(f"  ronda: partida sector 2 -> árbol {arbol}, albercas 1 y 2 | {pts} puntos en {m.tiempo():.0f} s | "
              f"faltas {verdad.faltas or 'ninguna'}")
        for e in m.eventos:
            if "carril" in e or "MISIÓN" in e or "detenida" in e:
                print("     ", e)
        assert not verdad.faltas, verdad.faltas
        assert pts > 0, m.eventos[-5:]
    finally:
        verdad.detener()
        mega.ir_sim.detener()
        mega.cerrar()


def main():
    cfg = cargar()
    print("1) Carril entre albercas")
    prueba_carril()
    print("2) Albercas con la cámara")
    prueba_albercas(cfg)
    print("3) Foto del árbol sin LiDAR")
    prueba_arbol(cfg)
    if "--rapido" not in sys.argv:
        print("4) Ronda completa simulada")
        prueba_ronda(cargar())
    print("OK: misión sin LiDAR")


if __name__ == "__main__":
    main()
