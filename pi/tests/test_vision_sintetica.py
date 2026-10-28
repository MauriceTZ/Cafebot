"""Visión real (VisionCamara) con fotos SINTÉTICAS dibujadas con el mismo modelo de cámara: proyección de
los soportes, calibración de colores, clasificación de los 16 soportes (también con el robot corrido de lado
según el LiDAR) y color de una caja. Sin cámara.     python3 -m tests.test_vision_sintetica
"""
import os
import tempfile

import cv2
import numpy as np

from cafebot.config import cargar
from cafebot.mundo import cancha
from cafebot.vision import clasificador as cl
from cafebot.vision.proyeccion import ModeloCamara, tablero_frente
from cafebot.vision.vision import VisionCamara

BGR = {"ROJO": (40, 40, 200), "NARANJO": (30, 130, 240), "AMARILLO": (40, 220, 235), "VERDE": (60, 170, 60),
       "AZUL": (190, 90, 30), "NEGRO": (35, 35, 35)}
CAT = {"ROJO": cancha.MADURO, "NARANJO": cancha.MADURO, "AMARILLO": cancha.MADURO, "VERDE": cancha.VERDE,
       "AZUL": cancha.SOBREMADURO, "NEGRO": cancha.SOBREMADURO}


def foto_arbol(cfg, colores: dict, tablero, rng) -> np.ndarray:
    """Fondo café claro (el árbol) + un círculo por grano, con ruido de cámara."""
    cc = cfg.camara
    img = np.full((cc.alto_px, cc.ancho_px, 3), (120, 160, 190), np.uint8)
    pos = ModeloCamara(cc).posiciones_soportes(2, tablero)
    for nombre, (u, v, r) in pos.items():
        cv2.circle(img, (u, v), int(r * 1.25), (60, 90, 130), -1)           # soporte café
        if colores.get(nombre):
            cv2.circle(img, (u, v), r, BGR[colores[nombre]], -1)
    ruido = rng.normal(0, 6, img.shape)
    return np.clip(img + ruido, 0, 255).astype(np.uint8)


def main():
    cfg = cargar()
    rng = np.random.default_rng(1)
    tmp = tempfile.mkdtemp()
    cl.ARCHIVO_REFERENCIAS = os.path.join(tmp, "ref.json")
    cl.CARPETA_CAPTURAS = os.path.join(tmp, "capturas")
    nombres = [f"{f}-{i}" for f, n in (("alta", 5), ("media", 6), ("baja", 5)) for i in range(1, n + 1)]
    t0 = tablero_frente(cfg.mision.distancia_foto_m, cfg.geometria.largo, cfg.camara.y)  # cámara centrada

    # 1) Calibración: una foto por color (todos los soportes del mismo color) + árbol vacío
    clasif = cl.Clasificador()
    for clase in list(BGR) + ["VACIO"]:
        img = foto_arbol(cfg, {n: (None if clase == "VACIO" else clase) for n in nombres}, t0, rng)
        lab = cl.imagen_a_lab(img)
        for nombre, (x, y, r) in ModeloCamara(cfg.camara).posiciones_soportes(2, t0).items():
            clasif.agregar(clase, cl.color_zona(lab, x, y, r, clasif.config), nombre)
    print(f"Referencias: {len(clasif.muestras)} muestras")

    # 2) Un árbol sorteado como en la ronda, con el robot corrido 3 cm (el LiDAR lo mide)
    reales = {}
    for fila, cats in (("alta", ["VERDE", "ROJO", "NARANJO", "AMARILLO", "AZUL"]),
                       ("media", ["VERDE", "VERDE", "ROJO", "AMARILLO", "NEGRO", "AZUL"]),
                       ("baja", ["VERDE", "VERDE", "ROJO", "AZUL", None])):
        rng.shuffle(cats)
        for i, c in enumerate(cats):
            reales[f"{fila}-{i + 1}"] = c
    for lateral in (0.0, 0.03, -0.04):
        t = tablero_frente(cfg.mision.distancia_foto_m, cfg.geometria.largo, cfg.camara.y + lateral)
        fotos = [foto_arbol(cfg, reales, t, rng) for _ in range(3)]
        v = VisionCamara(cfg, camara=None, clasificador=clasif, log=lambda s: None)
        lect = v.clasificar_arbol(2, None, tablero=t, imagenes=fotos)
        bien = mal = fuera = 0
        for l in lect:
            nombre = f"{l.clave[1]}-{l.clave[2] + 1}"
            esperado = CAT.get(reales[nombre], "VACIO") if reales[nombre] else "VACIO"
            if l.categoria == "FUERA":
                fuera += 1
            elif l.categoria == esperado:
                bien += 1
            else:
                mal += 1
                print(f"   {nombre}: esperado {esperado}, dio {l.categoria}")
        print(f"Robot corrido {100 * lateral:+.0f} cm: {bien} bien, {mal} mal, {fuera} fuera de la foto")
        assert mal == 0 and bien + fuera == 16 and fuera <= 2
    # Si el LiDAR NO corrige (se usa el tablero sin corrimiento), con 4 cm de error falla: por eso se mide
    t_mal = tablero_frente(cfg.mision.distancia_foto_m, cfg.geometria.largo, cfg.camara.y - 0.04)
    fotos = [foto_arbol(cfg, reales, t_mal, rng) for _ in range(3)]
    lect = VisionCamara(cfg, clasificador=clasif, log=lambda s: None).clasificar_arbol(2, None, tablero=t0, imagenes=fotos)
    granos = (cancha.MADURO, cancha.SOBREMADURO, cancha.VERDE)
    peligrosos = sum(1 for l in lect if l.categoria in granos and
                     l.categoria != CAT.get(reales[f"{l.clave[1]}-{l.clave[2] + 1}"]))
    perdidos = sum(1 for l in lect if l.categoria in ("VACIO", "NO_SE", "FUERA") and reales[f"{l.clave[1]}-{l.clave[2] + 1}"])
    print(f"Sin la corrección del LiDAR (4 cm): {perdidos} granos perdidos (salen VACIO/NO_SE: no se tocan), "
          f"{peligrosos} con categoría equivocada")
    assert peligrosos == 0

    # 3) Color de una caja roja vista de frente con la cámara hacia abajo
    cc = cfg.camara
    for color, bgr in (("ROJA", (40, 40, 200)), ("AZUL", (190, 90, 30))):
        img = np.full((cc.alto_px, cc.ancho_px, 3), (235, 235, 235), np.uint8)   # lona blanca
        caja = cancha.rect_caja(3)
        pose = np.array([caja.cx + caja.ancho_x / 2 + 0.55 + cfg.geometria.largo / 2, caja.cy, np.pi])
        modelo = ModeloCamara(cc, cc.inclinacion_cajas_deg)
        from cafebot.navegacion.geometria import relativo
        pts = []
        for dx in (-1, 1):
            for dy in (-1, 1):
                for z in (0.0, 0.09):
                    p = relativo(pose, np.array([caja.cx + dx * caja.ancho_x / 2, caja.cy + dy * caja.ancho_y / 2, 0]))
                    q = modelo.proyectar((p[0], p[1], z))
                    if q:
                        pts.append((int(q[0]), int(q[1])))
        hull = cv2.convexHull(np.array(pts))
        cv2.fillConvexPoly(img, hull, bgr)
        res = VisionCamara(cfg, clasificador=clasif, log=lambda s: None).color_caja(3, pose, imagen=img)
        print(f"Caja {color}: la visión dice {res}")
        assert res == color
    print("OK: visión con fotos sintéticas")


if __name__ == "__main__":
    main()
