"""Visión REAL para la misión (implementa `mision.interfaces.Vision`).

Árbol: foto(s) con exposición y balance de blancos fijos (camara.py) → posición de cada soporte en la imagen
por geometría (proyeccion.py, con el tablero medido por el LiDAR) → color Lab de cada zona contra las
referencias calibradas en sitio (clasificador.py, de Pablo). Varias fotos: si un soporte no da la misma clase
en todas, queda NO_SE. Soportes que no caen dentro de la imagen → "FUERA" (la misión se corre de lado y
saca otra foto).

Cajas: mirando de frente a una caja (cámara inclinada hacia abajo), se cuentan píxeles rojos y azules dentro
del rectángulo donde debería verse la caja.
"""
from __future__ import annotations

import math
import os
import time

import numpy as np

from cafebot.mision.interfaces import LecturaSoporte, Vision
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import relativo
from cafebot.vision import clasificador as cl
from cafebot.vision.proyeccion import ModeloCamara, nombre_a_clave

CATEGORIA = {"ROJO": cancha.MADURO, "NARANJO": cancha.MADURO, "AMARILLO": cancha.MADURO,
             "AZUL": cancha.SOBREMADURO, "NEGRO": cancha.SOBREMADURO, "VERDE": cancha.VERDE,
             "VACIO": "VACIO", cl.NO_SE: "NO_SE"}
FUERA = "FUERA"


def tablero_desde_pose(arbol: int, pose_robot_cancha) -> tuple[float, float, float]:
    """Sin medida del LiDAR: la cara del tablero en el sistema del robot según la pose en la cancha."""
    r = relativo(np.asarray(pose_robot_cancha, float), np.array([cancha.ARBOL_X_FRENTE, cancha.centros_arboles()[arbol - 1], 0.0]))
    return float(r[0]), float(r[1]), float(-pose_robot_cancha[2])


class VisionCamara(Vision):
    def __init__(self, cfg, camara=None, clasificador=None, log=print):
        self.cfg, self.cc = cfg, cfg.camara
        self._cam = camara
        self.clasif = clasificador or cl.Clasificador.cargar(cl.ARCHIVO_REFERENCIAS)
        self.log = log
        if not self.clasif.muestras:
            self.log("[visión] ¡no hay referencias de color! (python3 -m tools.camara referencias ...): todo será NO_SE")
        self.ultima_imagen = None
        self.ultimas_posiciones = {}

    # ------------------------------------------------------------------ cámara
    @property
    def cam(self):
        if self._cam is None:
            from cafebot.vision.camara import Camara, cargar_ajustes
            aj = cargar_ajustes()
            self._cam = Camara((aj or {}).get("dist_m", 0.6))
            if aj:
                self._cam.aplicar(aj)
            else:
                self.log("[visión] sin ajustes de exposición guardados: se fijan ahora (python3 -m tools.camara exposicion)")
                self._cam.bloquear_auto()
        return self._cam

    def iniciar(self):
        _ = self.cam                      # abre la cámara y fija la exposición antes del botón

    def preparar_arbol(self):
        dist = self.cfg.geometria.largo / 2 + self.cfg.mision.distancia_foto_m - self.cc.x
        self.cam.enfocar(dist)

    def fotos(self, n: int | None = None) -> list:
        out = []
        for _ in range(n or self.cc.fotos):
            out.append(self.cam.capturar())
            time.sleep(0.15)
        return out

    def _guardar(self, img, nombre):
        if not self.cc.guardar_fotos:
            return None
        import cv2
        os.makedirs(cl.CARPETA_CAPTURAS, exist_ok=True)
        ruta = os.path.join(cl.CARPETA_CAPTURAS, time.strftime(f"%Y%m%d_%H%M%S_{nombre}.jpg"))
        cv2.imwrite(ruta, img)
        return ruta

    # ------------------------------------------------------------------ árbol
    def clasificar_arbol(self, arbol, pose_robot_cancha, tablero=None, imagenes=None):
        if tablero is None:
            tablero = tablero_desde_pose(arbol, pose_robot_cancha)
        modelo = ModeloCamara(self.cc)
        pos = modelo.posiciones_soportes(arbol, tablero)
        imgs = imagenes if imagenes is not None else self.fotos()
        res = self.clasif.clasificar_varias(imgs, pos) if pos else {}
        self.ultima_imagen, self.ultimas_posiciones = imgs[0], pos
        self._guardar(cl.dibujar(imgs[0], pos, res), f"arbol{arbol}")
        umbral = self.clasif.config["umbral_distancia"]
        out = []
        for s in cancha.soportes(arbol):
            nombre = f"{s.fila}-{s.col + 1}"
            if nombre not in res:
                out.append(LecturaSoporte(s.clave, FUERA, 0.0))
                continue
            r = res[nombre]
            cat = CATEGORIA.get(r.clase, "NO_SE")
            conf = 0.0 if cat == "NO_SE" else max(0.0, 1.0 - 0.3 * r.distancia / umbral)
            out.append(LecturaSoporte(s.clave, cat, conf))
        return out

    # ------------------------------------------------------------------ cajas
    def color_caja(self, zona: int, pose_robot_cancha, imagen=None) -> str | None:
        """Mirando hacia la caja de `zona` (cámara inclinada hacia abajo). 'ROJA', 'AZUL' o None."""
        import cv2
        modelo = ModeloCamara(self.cc, self.cc.inclinacion_cajas_deg)
        caja = cancha.rect_caja(zona)
        pose = np.asarray(pose_robot_cancha, float)
        esquinas = []
        for dx in (-1, 1):
            for dy in (-1, 1):
                for z in (0.0, 0.09):
                    p = relativo(pose, np.array([caja.cx + dx * caja.ancho_x / 2, caja.cy + dy * caja.ancho_y / 2, 0.0]))
                    esquinas.append((p[0], p[1], z))
        rect = modelo.rect_en_imagen(esquinas)
        img = imagen if imagen is not None else self.fotos(1)[0]
        if rect is None:
            self.log("[visión] la caja no cae dentro de la imagen")
            return None
        u0, v0, u1, v1 = rect
        hsv = cv2.cvtColor(img[v0:v1, u0:u1], cv2.COLOR_BGR2HSV)
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        saturado = (s > 90) & (v > 50)
        rojo = float(np.mean(saturado & ((h < 10) | (h > 170))))
        azul = float(np.mean(saturado & (h > 100) & (h < 130)))
        marca = img.copy()
        cv2.rectangle(marca, (u0, v0), (u1, v1), (0, 255, 255), 2)
        cv2.putText(marca, f"rojo {rojo:.0%} azul {azul:.0%}", (u0, max(20, v0 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        self._guardar(marca, f"caja{zona}")
        self.log(f"[visión] caja del sector {zona}: rojo {rojo:.0%}, azul {azul:.0%} del recuadro")
        if max(rojo, azul) < 0.08 or abs(rojo - azul) < 0.05:
            return None
        return "ROJA" if rojo > azul else "AZUL"

    def color_cajas(self, zonas_con_caja, pose_robot_cancha):
        return {}          # desde la partida la cámara (mirando a los árboles) no ve las cajas

    def cerrar(self):
        if self._cam is not None:
            self._cam.cerrar()
