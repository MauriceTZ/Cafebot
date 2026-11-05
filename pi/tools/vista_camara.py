"""Vista de la CÁMARA dentro de la interfaz de navegación (tools.navegacion), en una esquina del mapa.

  Simulación (--sim): lo que vería la cámara en la cancha simulada, desde la pose REAL del robot simulado
                      (cafebot/vision/render.py: polígonos con el mismo modelo de cámara que la visión).
  Robot:              la cámara de verdad, compartida con la misión (cafebot/vision/camara.CamaraCompartida:
                      la misión sigue sacando sus fotos; la vista muestra la última o pide una nueva).
La inclinación de la cámara es la del último `X 2` que aceptó el Mega (al encender, 80°: al frente).

Superposición (Shift+V): las líneas de la cancha (cian) y los 48 soportes donde DEBERÍAN verse según la pose que el
robot CREE tener en la cancha (I o la misión) y la calibración de la cámara. Cada soporte con el color de lo que
clasificó la misión (rojo maduro, azul sobremaduro, verde, amarillo no sé, gris vacío; blanco = sin leer). Si
calzan con la imagen, la localización y la cámara están bien; en la simulación, la diferencia es el error de la
localización (la imagen sale de la pose real).

    V        cámara chica -> grande -> oculta          Shift+V  superposición sí/no
"""
from __future__ import annotations

import threading
import time

import cv2
import numpy as np

from cafebot.vision import render
from cafebot.vision.camara_modelo import CamaraPinhole

TAMANOS = {"chica": 0.30, "grande": 0.50, "no": 0.0}      # fracción del ancho de la ventana
SIGUIENTE = {"chica": "grande", "grande": "no", "no": "chica"}
SERVO_INICIAL = 80.0                                       # AUX_INICIAL_DEG del firmware (cámara al frente, 5 nov)


class VistaCamara:
    def __init__(self, cfg, tamano: str = "chica", mega=None, mundo=None, pose_real_fn=None, camara=None):
        """Simulación: mundo (render.MundoCancha / MundoTaller) + pose_real_fn() -> (x, y, θ) en ese mundo.
        Robot: camara (CamaraCompartida)."""
        self.cc = cfg.camara
        self.mega = mega
        self.mundo, self.pose_real_fn = mundo, pose_real_fn
        self.camara = camara
        self.tamano = tamano if tamano in TAMANOS else "chica"
        self.superponer = True
        self.error = ""
        self._tam = (400, 225)            # el tamaño del cuadro que pide la interfaz
        self._cuadro = None               # (imagen BGR, CamaraPinhole de esa imagen, t)
        self._version = 0
        self._sup, self._sup_estado, self._sup_tam = None, None, None
        self._clave_render = None
        self.rect = None                  # dónde quedó dibujada (para no tomar sus clics como destino)
        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle, name="vista-camara", daemon=True)
        self._hilo.start()

    @property
    def simulada(self) -> bool:
        return self.mundo is not None

    @property
    def hay_cancha(self) -> bool:
        """La superposición es de la cancha: no en la pista de taller simulada."""
        return not isinstance(self.mundo, render.MundoTaller)

    def ciclar(self):
        self.tamano = SIGUIENTE[self.tamano]
        self.error = ""                    # y reintentar si había fallado

    def servo(self) -> float:
        s = self.mega.aux.get(2) if self.mega is not None else None
        return SERVO_INICIAL if s is None else float(s)

    def inclinacion(self) -> float:
        return self.cc.pitch_de_servo(self.servo())

    def detener(self):
        self._corriendo = False
        self._hilo.join(2.0)

    # ------------------------------------------------------------------ hilo: produce los cuadros
    def _bucle(self):
        t_error = 0.0
        while self._corriendo:
            # con error, la cámara real no se reintenta sola (cada intento de abrirla llena la terminal de
            # mensajes de libcamera): se reintenta con V
            if self.tamano == "no" or (self.error and (not self.simulada or time.monotonic() - t_error < 3.0)):
                time.sleep(0.1)
                continue
            t0 = time.monotonic()
            periodo = 0.12
            try:
                w, h = self._tam
                cam = CamaraPinhole.desde_config(self.cc, self.inclinacion()).escalada(w, h)
                if self.simulada:
                    cam = cam.con(dist=np.zeros(5))
                    pose = np.asarray(self.pose_real_fn(), float)
                    granos = getattr(getattr(self.mundo, "esc", None), "granos", {})
                    clave = (w, h, self.servo(), len(granos), tuple(np.round(pose, 4)))
                    if clave != self._clave_render:          # el robot quieto: no se vuelve a dibujar
                        self._cuadro = (render.dibujar(self.mundo, pose, cam), cam, time.monotonic())
                        self._clave_render = clave
                        self._version += 1
                else:
                    if self.camara is None:
                        raise RuntimeError("sin cámara")
                    foto = self.camara.reciente(0.25)
                    img = cv2.resize(foto, (w, h), interpolation=cv2.INTER_AREA)
                    self._cuadro = (img, cam, time.monotonic())
                    self._version += 1
                    periodo = 0.2
                self.error = ""
            except Exception as e:
                self.error = f"{type(e).__name__}: {e}"[:160]
                t_error = time.monotonic()
            time.sleep(max(0.01, periodo - (time.monotonic() - t0)))

    # ------------------------------------------------------------------ interfaz (hilo de pygame)
    def calcular_rect(self, W: int, H: int, y_min: int = 0):
        """Abajo a la derecha, con la forma de la imagen (16:9). None si está oculta."""
        import pygame as pg
        f = TAMANOS[self.tamano]
        if f <= 0:
            return None
        w = max(160, int(W * f))
        h = int(round(w * self.cc.alto_px / self.cc.ancho_px))
        alto_libre = H - y_min - 30
        if h > alto_libre:                                  # ventana baja: que quepa
            h = max(90, alto_libre)
            w = int(round(h * self.cc.ancho_px / self.cc.alto_px))
        return pg.Rect(W - w - 10, H - h - 10, w, h)

    def dibujar(self, win, fuente, pose_cancha=None, lecturas=None, y_min: int = 0):
        """Dibuja la vista (y la superposición, con la pose estimada en la cancha). Devuelve su rectángulo o None."""
        import pygame as pg
        W, H = win.get_size()
        r = self.rect = self.calcular_rect(W, H, y_min)
        if r is None:
            return None
        self._tam = (r.w, r.h)
        c = self._cuadro
        pc = None if pose_cancha is None else tuple(np.round(np.asarray(pose_cancha, float)[:3], 3))
        estado = (self._version, (r.w, r.h), self.superponer, pc, len(lecturas or {}))
        if c is not None and estado != self._sup_estado:
            img, cam, _ = c
            if img.shape[1] != r.w or img.shape[0] != r.h:   # recién cambió el tamaño: escalar este cuadro
                img = cv2.resize(img, (r.w, r.h), interpolation=cv2.INTER_AREA)
                cam = cam.escalada(r.w, r.h)
            if self.superponer and pose_cancha is not None and self.hay_cancha:
                img = img.copy()
                render.superponer_cancha(img, cam, pose_cancha, lecturas)
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            self._sup = pg.image.frombuffer(rgb.tobytes(), (r.w, r.h), "RGB")
            self._sup_estado, self._sup_tam = estado, (r.w, r.h)
        pg.draw.rect(win, (8, 10, 16), r.inflate(6, 26).move(0, -10))
        if self._sup is not None and self._sup_tam == (r.w, r.h):
            win.blit(self._sup, r)
        else:
            pg.draw.rect(win, (20, 22, 28), r)
        if self.error or c is None:
            txt = (self.error + ("  (V para reintentar)" if not self.simulada else "") if self.error
                   else "abriendo la cámara..." if not self.simulada else "dibujando...")
            win.blit(fuente.render(txt[: max(10, r.w // 8)], True, (255, 190, 90)), (r.x + 8, r.y + 8))
        pg.draw.rect(win, (90, 100, 120), r, 1)
        quien = "simulada" if self.simulada else "robot"
        vieja = c is not None and time.monotonic() - c[2] > 2.0 and not self.simulada
        sup = "" if not (self.superponer and self.hay_cancha) else (
            " | superposición" if pose_cancha is not None else " | superposición: falta ubicar (I o K)")
        titulo = (f"CÁMARA {quien}{' (sin cuadros nuevos)' if vieja else ''} | incl {self.inclinacion():+.0f}° "
                  f"(X 2 = {self.servo():.0f}){sup}")
        while len(titulo) > 8 and fuente.size(titulo)[0] > r.w:
            titulo = titulo[:-2] + "…"
        win.blit(fuente.render(titulo, True, (200, 210, 225)), (r.x, r.y - 18))
        return r
