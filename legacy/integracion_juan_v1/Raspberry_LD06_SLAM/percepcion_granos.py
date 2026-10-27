"""Percepción de granos (cámara + clasificador de color) para el robot con SLAM.

Envuelve los tres módulos calibrados de la carpeta vision/ (camara.py,
clasificador.py, detector.py) sin modificarlos y les agrega lo que el SLAM
necesita:

  * un HILO propio: el SLAM (10 Hz) nunca espera a la cámara (el detector tarda
    ~130 ms por cuadro en la Pi). Mientras la visión está INACTIVA no consume CPU;
    solo se activa al llegar al árbol (o con la tecla [V]);
  * un FILTRO DE ESTABILIDAD: un grano solo se "publica" como estable si la misma
    categoría se ve en N cuadros seguidos (regla del proyecto: si hay duda, NO se toca);
  * una POLÍTICA conservadora: MADURO / SOBREMADURO -> candidato a recoger,
    VERDE -> ignorar (-7 si se saca), NO_SE -> no tocar.

Suposición de montaje: la cámara mira hacia ADELANTE (mismo sentido que el ángulo 0 del
LD06). Si va montada en la pinza/brazo, el desvío dx/dy se expresa respecto del píxel
objetivo (por defecto el centro de la imagen; ver `objetivo_frac`).
"""
from __future__ import annotations

import math
import os
import sys
import threading
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np
from scipy.optimize import linear_sum_assignment

AQUI = os.path.dirname(os.path.abspath(__file__))
VISION_DIR = os.path.join(AQUI, "vision")
if VISION_DIR not in sys.path:
    sys.path.insert(0, VISION_DIR)

import cv2                      # noqa: E402
import clasificador as C        # noqa: E402
import detector as D            # noqa: E402
import camara as CAM            # noqa: E402  (picamera2 solo se importa al abrir la cámara)

# Rutas absolutas: los módulos de vision/ usan rutas relativas al directorio de trabajo.
CARPETA_CALIBRACION = os.path.join(VISION_DIR, "calibracion")
C.ARCHIVO_REFERENCIAS = os.path.join(CARPETA_CALIBRACION, "referencias.json")
D.ARCHIVO_CONFIG = os.path.join(CARPETA_CALIBRACION, "detector.json")
CAM.ARCHIVO_AJUSTES = os.path.join(CARPETA_CALIBRACION, "camara.json")
CARPETA_CAPTURAS = os.path.join(AQUI, "capturas")
C.CARPETA_CAPTURAS = CARPETA_CAPTURAS
CAM.CARPETA_CAPTURAS = CARPETA_CAPTURAS

# ---------------------------------------------------------------- política
ACCION_RECOGER = "RECOGER"
ACCION_IGNORAR_VERDE = "IGNORAR_VERDE"
ACCION_NO_TOCAR = "NO_TOCAR"
PUNTOS_CAJA = {"MADURO": 7, "SOBREMADURO": 5}      # reglamento LARC OPEN 2026


@dataclass
class Grano:
    """Un grano visto por la cámara (ya clasificado)."""
    track_id: int
    clase: str                  # ROJO, NARANJO, AMARILLO, AZUL, VERDE, NEGRO o NO_SE
    categoria: str              # MADURO, SOBREMADURO, VERDE, DESCONOCIDO
    cx: float                   # centro en píxeles
    cy: float
    diam_px: float
    dx_mm: float                # desvío respecto del píxel objetivo (+ = a la derecha)
    dy_mm: float                # (+ = abajo)
    dist_m: Optional[float]     # solo si se calibró focal_px (detector.py --calibrar-dist)
    dist_color: float           # distancia Lab a la referencia (menor = más seguro)
    votos: int                  # cuadros seguidos con la misma categoría
    estable: bool
    accion: str

    @property
    def recogible(self) -> bool:
        return self.estable and self.accion == ACCION_RECOGER

    def resumen(self) -> str:
        d = f" {self.dist_m * 100:.0f}cm" if self.dist_m else ""
        return (f"#{self.track_id} {self.clase}/{self.categoria} dx={self.dx_mm:+.0f}mm "
                f"dy={self.dy_mm:+.0f}mm{d} {'ESTABLE' if self.estable else f'votos={self.votos}'}")


@dataclass
class Snapshot:
    frame_id: int = 0
    t: float = 0.0
    granos: List[Grano] = field(default_factory=list)
    imagen: Optional[np.ndarray] = None     # BGR anotada (para mostrar en el HUD)
    ms: float = 0.0


def accion_para(categoria: str) -> str:
    if categoria in ("MADURO", "SOBREMADURO"):
        return ACCION_RECOGER
    if categoria == "VERDE":
        return ACCION_IGNORAR_VERDE
    return ACCION_NO_TOCAR


# ---------------------------------------------------------------- estabilidad
class _Track:
    _sig = 1

    def __init__(self, det, n):
        self.id = _Track._sig
        _Track._sig += 1
        self.cx, self.cy, self.diam = det.cx, det.cy, det.diam_px
        self.hist = deque(maxlen=n)         # (clase, categoria) de los últimos n cuadros
        self.perdidos = 0

    def actualizar(self, det):
        self.cx, self.cy, self.diam = det.cx, det.cy, det.diam_px
        self.hist.append((det.clase, C.CATEGORIA[det.clase]))
        self.perdidos = 0


class SeguidorGranos:
    """Asocia detecciones entre cuadros (por posición) y exige N cuadros seguidos con la
    misma categoría antes de declarar un grano 'estable'."""

    def __init__(self, n_estable=5, max_perdidos=3, asoc_frac=0.8):
        self.n = n_estable
        self.max_perdidos = max_perdidos
        self.asoc_frac = asoc_frac
        self.tracks: List[_Track] = []

    def reset(self):
        self.tracks = []

    def actualizar(self, dets) -> list:
        """dets: lista de detector.Deteccion. Devuelve [(det, track)] de las asociadas."""
        pares = []
        usadas_t, usadas_d = set(), set()
        if self.tracks and dets:
            costo = np.array([[math.hypot(t.cx - d.cx, t.cy - d.cy) for d in dets]
                              for t in self.tracks])
            for i, j in zip(*linear_sum_assignment(costo)):
                if costo[i, j] <= self.asoc_frac * max(self.tracks[i].diam, dets[j].diam_px):
                    self.tracks[i].actualizar(dets[j])
                    pares.append((dets[j], self.tracks[i]))
                    usadas_t.add(i)
                    usadas_d.add(j)
        for i, t in enumerate(self.tracks):
            if i not in usadas_t:
                t.perdidos += 1
        for j, d in enumerate(dets):
            if j not in usadas_d:
                t = _Track(d, self.n)
                t.actualizar(d)
                self.tracks.append(t)
                pares.append((d, t))
        self.tracks = [t for t in self.tracks if t.perdidos <= self.max_perdidos]
        return pares

    def evaluar(self, track: _Track):
        """(categoria_estable|None, clase_representativa, votos)."""
        votos = 0
        if track.hist:
            ult_cat = track.hist[-1][1]
            for _, cat in reversed(track.hist):
                if cat != ult_cat:
                    break
                votos += 1
        if len(track.hist) < self.n or votos < self.n:
            return None, track.hist[-1][0] if track.hist else C.NO_SE, votos
        cat = track.hist[-1][1]
        if cat in ("DESCONOCIDO", "VACIO"):
            return None, C.NO_SE, votos
        clase = Counter(c for c, k in track.hist if k == cat).most_common(1)[0][0]
        return cat, clase, votos


# ---------------------------------------------------------------- hilo de visión
class VisionWorker:
    def __init__(self, hz=5.0, imagen: Optional[str] = None, n_estable=5,
                 guardar_cada_s=0.0, diam_px_rango=None, objetivo_frac=(0.5, 0.5)):
        """imagen: ruta de una foto fija (pruebas sin cámara). Si es None se usa la Pi Camera.
        diam_px_rango: (min, max) opcional para descartar manchas que no miden ~42 mm."""
        self.period = 1.0 / max(hz, 0.5)
        self.imagen = imagen
        self.guardar_cada_s = guardar_cada_s
        self.diam_px_rango = diam_px_rango
        self.objetivo_frac = objetivo_frac
        self.cfg = D.cargar_config()
        self.clasif = C.Clasificador.cargar(C.ARCHIVO_REFERENCIAS)
        self.seguidor = SeguidorGranos(n_estable=n_estable)
        self.n_estable = n_estable
        self.status = "visión: inactiva"
        self.error = ""
        self.manual = False          # True si el operador la encendió con [V] (la misión no la apaga)
        self._active = False
        self._cam = None
        self._cam_failed = False
        self._lock = threading.Lock()
        self._snap = Snapshot()
        self._frame_id = 0
        self._last_save = 0.0
        self._run = True
        self._wake = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        if not self.clasif.muestras:
            self.status = "visión: SIN referencias de color (calibracion/referencias.json vacío)"

    # ---- control ----
    def start(self):
        self._thread.start()

    @property
    def active(self) -> bool:
        return self._active

    def set_active(self, on: bool):
        if on and not self._active:
            self._cam_failed = False          # permitir reintentar abrir la cámara
            self.seguidor.reset()
        self._active = bool(on)
        if on:
            self._wake.set()
        else:
            self.status = "visión: inactiva"

    def reset_tracks(self):
        self.seguidor.reset()

    def snapshot(self) -> Snapshot:
        with self._lock:
            return self._snap

    def stop(self):
        self._run = False
        self._active = False
        self._wake.set()
        self._thread.join(timeout=1.0)
        if self._cam is not None:
            try:
                self._cam.cerrar()
            except Exception:
                pass

    # ---- cámara ----
    def _abrir_camara(self):
        if self.imagen:
            self._img_fija = C.leer_imagen(self.imagen)
            return
        ajustes = CAM.cargar_ajustes()
        self._cam = CAM.Camara((ajustes or {}).get("dist_m", 0.25))
        if ajustes:
            self._cam.aplicar(ajustes)
        else:
            self._cam.bloquear_auto()

    def _capturar(self):
        if self.imagen:
            return self._img_fija.copy()
        return self._cam.capturar()

    # ---- bucle ----
    def _loop(self):
        while self._run:
            if not self._active:
                self._wake.wait(0.2)
                self._wake.clear()
                continue
            t0 = time.monotonic()
            try:
                if self._cam is None and not hasattr(self, "_img_fija"):
                    if self._cam_failed:
                        time.sleep(0.3)
                        continue
                    self.status = "visión: abriendo cámara..."
                    self._abrir_camara()
                img = self._capturar()
                self._procesar(img)
                self.error = ""
            except Exception as e:                      # cámara no disponible, etc.
                self.error = f"{type(e).__name__}: {e}"
                self.status = f"visión: ERROR {self.error[:80]}"
                self._cam_failed = True
                self._cam = None
                time.sleep(0.3)
                continue
            resto = self.period - (time.monotonic() - t0)
            if resto > 0:
                time.sleep(resto)

    def _procesar(self, img):
        t0 = time.perf_counter()
        alto, ancho = img.shape[:2]
        objetivo = (ancho * self.objetivo_frac[0], alto * self.objetivo_frac[1])
        dets = D.detectar(img, self.cfg, self.clasif)
        if self.diam_px_rango:
            lo, hi = self.diam_px_rango
            dets = [d for d in dets if lo <= d.diam_px <= hi]
        pares = self.seguidor.actualizar(dets)
        granos = []
        for d, tr in pares:
            cat, clase, votos = self.seguidor.evaluar(tr)
            estable = cat is not None
            categoria = cat if estable else C.CATEGORIA[d.clase]
            dx, dy = d.desvio_mm(objetivo)
            granos.append(Grano(
                track_id=tr.id, clase=clase if estable else d.clase, categoria=categoria,
                cx=d.cx, cy=d.cy, diam_px=d.diam_px, dx_mm=dx, dy_mm=dy,
                dist_m=d.distancia_m(self.cfg.get("focal_px")), dist_color=d.dist_clase,
                votos=votos, estable=estable,
                accion=accion_para(categoria) if estable else ACCION_NO_TOCAR))
        granos.sort(key=lambda g: -g.diam_px)           # el más grande (cercano) primero
        ms = (time.perf_counter() - t0) * 1000.0
        anotada = self._dibujar(img, granos, objetivo)
        self._frame_id += 1
        with self._lock:
            self._snap = Snapshot(self._frame_id, time.monotonic(), granos, anotada, ms)
        n_est = sum(1 for g in granos if g.estable)
        self.status = (f"visión: ON {ms:.0f} ms | {len(granos)} granos ({n_est} estables, "
                       f"{sum(1 for g in granos if g.recogible)} recogibles)")
        if self.guardar_cada_s > 0 and time.monotonic() - self._last_save >= self.guardar_cada_s:
            self._last_save = time.monotonic()
            os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
            cv2.imwrite(os.path.join(CARPETA_CAPTURAS, time.strftime("vivo_%Y%m%d_%H%M%S.jpg")), img)

    def _dibujar(self, img, granos, objetivo):
        salida = img.copy()
        ox, oy = int(objetivo[0]), int(objetivo[1])
        cv2.drawMarker(salida, (ox, oy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
        for g in granos:
            color = C.COLOR_DIBUJO.get(g.clase, (255, 0, 255))
            centro = (int(g.cx), int(g.cy))
            cv2.circle(salida, centro, int(g.diam_px / 2), color, 3 if g.estable else 1)
            etiqueta = f"#{g.track_id} {g.clase}" + (" OK" if g.estable else f" {g.votos}/{self.n_estable}")
            pos = (int(g.cx - g.diam_px / 2), max(20, int(g.cy - g.diam_px / 2) - 8))
            cv2.putText(salida, etiqueta, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
            cv2.putText(salida, etiqueta, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return salida
