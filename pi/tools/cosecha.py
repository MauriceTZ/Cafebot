"""Cosecha de PRUEBA frente a un árbol: cámara + LiDAR + brazo (+ la base para acercarse y alejarse).

Qué hace (detalles en docs/cosecha.md):
  1. Mide el pie del árbol con el LiDAR (x, y, ángulo) y saca fotos.
  2. Busca las pelotas (manchas de color redondas; el soporte tapa su parte de abajo), las asigna a los soportes
     del modelo del árbol y AJUSTA la pose del árbol y la de la cámara (fusión cámara + LiDAR).
  3. Proyección inversa: la posición 3D de cada grano en el robot (x adelante, y izquierda, z desde el suelo).
  4. Color -> categoría (maduro / sobremaduro / verde / vacío / no sé). Ante la duda, no se toca.
  5. El brazo agarra el grano elegido (o el más cercano) con su cinemática inversa y lo deja en la canasta de su
     categoría (posturas deposito_maduro / deposito_sobremaduro, grabadas con tools.brazo).
  Las posiciones se guardan en el marco del ÁRBOL: si la base se acerca (A) la cámara ya no ve bien el árbol, pero el
  LiDAR sí ve el pie y con eso se recalcula dónde quedó cada grano.

En la Raspberry (escritorio por VNC), con el robot frente al árbol, el frente a ~60 cm del pie:
    python3 -m tools.cosecha                          # todo: cámara, LiDAR, Mega y brazo
    python3 -m tools.cosecha --sin-brazo              # solo ver (no mueve el brazo)
    python3 -m tools.cosecha --sin-mega               # la base no se mueve (A, R y flechas no hacen nada)
    python3 -m tools.cosecha --arbol taller           # modelo del árbol de prueba (pi/arboles/taller.json)
    python3 -m tools.cosecha --arbol taller --crear-arbol   # crea ese archivo (con las medidas de competencia) para editar
    python3 -m tools.cosecha --imagen capturas/F.jpg --frente 0.60    # fotos guardadas, sin robot (prior a mano)
    python3 -m tools.cosecha --imagen F.jpg --frente 0.60 --texto     # sin ventana: imprime y guarda la imagen marcada
    python3 -m tools.cosecha --sim                    # todo simulado: árbol sintético, Mega, LiDAR y brazo falsos

Ratón: clic sobre un soporte = elegirlo.
Teclas:
  ESPACIO  LiDAR + fotos + análisis               L  vista en vivo (sin análisis) sí/no      S  guardar fotos + análisis
  1..7     enseñar el color del soporte elegido: 1 ROJO 2 NARANJO 3 AMARILLO 4 VERDE 5 AZUL 6 NEGRO 7 VACIO
  I / K    inclinación de la cámara ±0,5° (prueba)   C  calibrar la cámara con el árbol + LiDAR (se promedia, se guarda)
  O        soltar / sujetar el brazo                 M  marcar: la garra (movida a mano) está en el grano elegido
  N        calibrar la cámara con las marcas del brazo (≥ 4, en distintas filas y columnas) y guardar
  P        garra al punto "antes" del grano elegido (6 cm antes, sin agarrar)     V  brazo a la postura de espera
  G        agarrar el elegido (o, sin elegir, el seguro y alcanzable más cercano) y dejarlo en su canasta
  T        agarrar todos los seguros y alcanzables (fila alta primero)
  A        acercarse (frente del robot a holgura_agarre_m del pie)   R  volver a la distancia de la foto
  ↑ / ↓    avanzar / retroceder 2 cm      ← / →  cangrejo 2 cm a la izquierda / derecha (granos de los costados)
  ESC      parar (base y brazo, después del movimiento en curso)      Q  salir
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import pathlib
import sys
import tempfile
import threading
import time
import traceback

import cv2
import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, relativo
from cafebot.vision import clasificador as cl
from cafebot.vision import granos as G
from cafebot.vision.arbol import ModeloArbol, medir_pie
from cafebot.vision.camara_modelo import CamaraPinhole

TECLAS_CLASE = ("ROJO", "NARANJO", "AMARILLO", "VERDE", "AZUL", "NEGRO", "VACIO")   # teclas 1..7
COLOR_CAT = {"MADURO": (60, 70, 255), "SOBREMADURO": (255, 130, 60), "VERDE": (70, 210, 70),
             "VACIO": (150, 150, 150), "NO_SE": (0, 220, 255), "FUERA": (95, 95, 95)}            # BGR
CORTO = {"ROJO": "ROJ", "NARANJO": "NAR", "AMARILLO": "AMA", "VERDE": "VER", "AZUL": "AZU", "NEGRO": "NEG",
         "VACIO": "vac", cl.NO_SE: "?", "FUERA": "---"}


def _grados(dcam) -> str:
    return "Δincl {:+.1f}° Δyaw {:+.1f}° Δroll {:+.1f}°".format(*np.degrees(dcam))


def lab_zona(img, u, v, r, config):
    """Lab mediano (sin brillos) de un círculo de la imagen, o None si se sale."""
    h, w = img.shape[:2]
    rr, uu, vv = max(2, int(round(r))), int(round(u)), int(round(v))
    if not (rr <= uu < w - rr and rr <= vv < h - rr):
        return None
    lab = cl.imagen_a_lab(img[vv - rr:vv + rr + 1, uu - rr:uu + rr + 1])
    return cl.color_zona(lab, rr, rr, rr, dict(config, fraccion_radio=1.0))


# ---------------------------------------------------------------------------- simulación
class MundoSim:
    """Un árbol sintético FIJO en el mundo; el robot (Mega simulado) se mueve. La cámara 'real' tiene un error de
    montaje (inclinación, yaw, roll, posición) que la calibración tiene que encontrar."""

    def __init__(self, cfg, modelo: ModeloArbol, mega=None, frente=0.62, lateral=0.0, ang_deg=3.0, semilla=None,
                 colores=None, bus_falso=None, error_camara=True):
        from cafebot.vision.sintetica import CamaraSintetica, LidarPie
        rng = np.random.default_rng(semilla)
        self.modelo, self.mega, self.bus_falso = modelo, mega, bus_falso
        self.pie_mundo = np.array([cfg.geometria.largo / 2 + frente, lateral, math.radians(ang_deg)])
        if colores is None:
            clases = ["ROJO", "NARANJO", "AMARILLO", "VERDE", "AZUL", "NEGRO", None]
            colores = {ModeloArbol.nombre_de(c): clases[int(rng.integers(len(clases)))] for c, _, _ in modelo.soportes()}
        self.colores = colores
        c = CamaraPinhole.desde_config(cfg.camara)
        if error_camara:
            c = c.con(pitch=c.pitch + math.radians(rng.uniform(-2.5, 2.5)), yaw=c.yaw + math.radians(rng.uniform(-1.5, 1.5)),
                      roll=c.roll + math.radians(rng.uniform(-1.2, 1.2)), pos=c.pos + rng.normal(0, 0.004, 3))
        self.cam_real = c
        self.camara = CamaraSintetica(c, modelo, self.pose_pie, self.colores, semilla)
        self.lidar = LidarPie(modelo, self.pose_pie, rng=rng)
        self.errores_agarre = []

    def pose_robot(self) -> np.ndarray:
        if self.mega is None:
            return np.zeros(3)
        from cafebot.drivers.ir_simulado import pose_mega_extrapolada
        return pose_mega_extrapolada(self.mega)

    def pose_pie(self):
        return tuple(float(v) for v in relativo(self.pose_robot(), self.pie_mundo))

    def grano_real(self, clave) -> np.ndarray:
        return self.modelo.en_robot(self.pose_pie())[clave]

    def antes_de_agarrar(self, clave, p, brazo=None):
        """El bus falso 'tiene grano' si la garra va a menos de 12 mm del grano real (la garra abre 65 mm, el grano
        mide 42: sobra ~11 mm por lado)."""
        err = float(np.linalg.norm(self.grano_real(clave) - np.asarray(p)))
        self.errores_agarre.append(err)
        if self.bus_falso is not None:
            self.bus_falso.grano = self.colores.get(ModeloArbol.nombre_de(clave)) is not None and err < 0.012

    def quitar(self, clave):
        self.colores[ModeloArbol.nombre_de(clave)] = None


def posturas_sim(brazo):
    """Posturas para el brazo simulado (en el robot se enseñan a mano con tools.brazo leer + G)."""
    brazo.perfil.posturas.update({
        "deposito_maduro": {"base": -150.0, "hombro": -25.0, "codo1": 60.0, "codo2": 60.0, "muneca": 40.0},
        "deposito_sobremaduro": {"base": 135.0, "hombro": -25.0, "codo1": 60.0, "codo2": 60.0, "muneca": 40.0},
        "transporte": {"base": 0.0, "hombro": -10.0, "codo1": 70.0, "codo2": 80.0, "muneca": 20.0}})


# ---------------------------------------------------------------------------- la sesión (sin ventana)
class Cosecha:
    """Lógica de la herramienta (la usan la ventana y las pruebas)."""

    def __init__(self, cfg, modelo: ModeloArbol, camara, lidar=None, mega=None, brazo=None, mundo=None,
                 clasif: cl.Clasificador | None = None, prior_fijo=None, categorias=("MADURO", "SOBREMADURO"),
                 ruta_local=None, ruta_referencias=None, confianza_min=None, log=print):
        self.cfg, self.modelo, self.camara, self.lidar, self.mega, self.brazo, self.mundo = (
            cfg, modelo, camara, lidar, mega, brazo, mundo)
        self.clasif = clasif if clasif is not None else cl.Clasificador()
        self.prior_fijo = prior_fijo                # (pose, σ) cuando no hay LiDAR (fotos guardadas)
        self.categorias = tuple(categorias)
        self.confianza_min = cfg.mision.confianza_min if confianza_min is None else float(confianza_min)
        self.ruta_local = ruta_local                # None = pi/robot_local.json
        self.ruta_referencias = ruta_referencias or cl.ARCHIVO_REFERENCIAS
        self._log = log
        self.cam = CamaraPinhole.desde_config(cfg.camara)
        self.analisis: G.Analisis | None = None
        self.imgs: list = []
        self.pose_pie = None                        # pie del árbol en el robot (medido/ajustado; se sigue al mover)
        self.scan_xy = None
        self.medida = None
        self.sel = None                             # clave del soporte elegido
        self.marcas: list = []                      # calibración con el brazo: (p garra, (u, v), nombre, k, ancho img)
        self.calibs: list = []                      # (pitch, yaw, roll) de cada C
        self.tomados: set = set()
        self.alcance: dict = {}                     # clave -> el brazo lo alcanza (se recalcula al medir/mover)
        self.delta_lidar = (0.0, 0.0)               # árbol ajustado − medida del LiDAR (x, y): ver analizar()
        self.suelto = False
        self.ocupado = ""
        self.version = 0                            # cambia cuando hay algo nuevo que dibujar
        self._hilo = None
        self.mensajes = collections.deque(maxlen=14)

    def log(self, txt: str):
        self.mensajes.append(txt)
        self.version += 1
        self._log(txt)

    # ------------------------------------------------------------------ tareas en segundo plano
    def en_fondo(self, nombre: str, f, *args) -> bool:
        if self.ocupado:
            self.log(f"espera: todavía {self.ocupado}")
            return False
        self.ocupado = nombre

        def correr():
            try:
                f(*args)
            except RuntimeError as e:          # avisos esperables (no ve el árbol, demasiado cerca...): sin traza
                self.log(f"[{nombre}] {e}")
            except Exception as e:
                self.log(f"[{nombre}] error: {e}")
                traceback.print_exc()
            finally:
                self.ocupado = ""
                self.version += 1
        self._hilo = threading.Thread(target=correr, name=nombre, daemon=True)
        self._hilo.start()
        return True

    def esperar(self, timeout=None):
        if self._hilo is not None:
            self._hilo.join(timeout)

    # ------------------------------------------------------------------ medir
    def sig_cam(self):
        return (math.radians(0.6),) * 3 if self.cfg.camara.calibrada else None

    def medir_lidar(self):
        if self.lidar is None:
            return None
        s = self.lidar.esperar_scan(1.5) or self.lidar.ultimo_scan()
        if s is None:
            self.log("LiDAR: no llegan barridos")
            return None
        x, y = s.xy()
        self.scan_xy = np.column_stack((x, y))
        if self.pose_pie is not None:
            esperado = self.pose_pie[:2]
        elif self.prior_fijo is not None:              # --frente / --lateral: dónde buscarlo
            esperado = self.prior_fijo[0][:2]
        else:
            esperado = None
        # Sin saber dónde está, el árbol se busca desde 10 cm delante del LiDAR: lo que ve pegado a él suele ser el
        # piso (va muy bajo), un cable o una pieza del robot
        x_min = self.cfg.geometria.lidar_x + 0.10
        med = medir_pie(self.scan_xy, self.modelo, esperado, x_min=x_min)
        if med is None and esperado is not None:
            med = medir_pie(self.scan_xy, self.modelo, x_min=x_min)
        self.medida = med
        if med is None:
            delante = self.scan_xy[(self.scan_xy[:, 0] > 0.05) & (np.abs(self.scan_xy[:, 1]) < 0.4)]
            cerca = (f"; lo más cercano delante está a {100 * (delante[:, 0].min() - self.cfg.geometria.largo / 2):.0f} cm "
                     "del frente" if len(delante) else "; no ve nada delante")
            self.log(f"LiDAR: no encuentro el pie del árbol{cerca}")
        else:
            self.log(f"LiDAR: pie a {100 * (med.x_cara - self.cfg.geometria.largo / 2):.1f} cm del frente, "
                     f"{100 * med.y_centro:+.1f} cm de lado, {math.degrees(med.ang):+.1f}°, ancho {100 * med.ancho:.0f} cm "
                     f"({med.n} puntos)" + ("" if med.centro_medido else
                                            f" | el modelo dice {100 * self.modelo.pie_ancho_m:.0f} cm de ancho"))
        return med

    def prior(self):
        """(pose esperada del pie, σ, medida del LiDAR o None)."""
        med = self.medir_lidar()
        if med is not None:
            return med.pose, med.sigma, med
        if self.prior_fijo is not None:
            return self.prior_fijo[0], self.prior_fijo[1], None
        if self.pose_pie is not None:
            self.log("el LiDAR no ve el pie: uso la última pose del árbol")
            return self.pose_pie, (0.03, 0.03, math.radians(4.0)), None
        raise RuntimeError("el LiDAR no ve el pie del árbol (¿delante, a menos de 1,6 m?) y no se dio --frente")

    def analizar(self, fotos_nuevas: bool = True) -> G.Analisis:
        pose0, sig, med = self.prior()
        d = pose0[0] - self.modelo.grano_delante_m - self.cam.pos[0]
        if d < 0.30:
            if self.analisis is None:
                raise RuntimeError(
                    f"el pie del árbol quedaría a {100 * (pose0[0] - self.cfg.geometria.largo / 2):.0f} cm del frente "
                    f"(los granos a {100 * d:.0f} cm de la cámara): si el robot está a ~60 cm, lo que se midió no es el "
                    "árbol (¿el LiDAR ve el piso o una pieza del robot? mirar la vista desde arriba o "
                    "python3 -m tools.visor_lidar). Mientras: --frente 0.60 dice dónde buscarlo")
            raise RuntimeError(f"los granos están a {100 * d:.0f} cm de la cámara: demasiado cerca para la foto "
                               f"(R vuelve a {100 * self.cfg.mision.distancia_foto_m:.0f} cm); lo ya medido sigue valiendo")
        if fotos_nuevas or not self.imgs:
            self.imgs = [self.camara.capturar() for _ in range(max(1, int(self.cfg.camara.fotos)))]
        cc = self.cfg.camara
        t0 = time.monotonic()
        a = G.analizar(self.imgs, self.cam, self.modelo, pose0, sig, self.clasif if self.clasif.muestras else None,
                       med, cc.sat_min, cc.val_min, sig_cam=self.sig_cam())
        for k in self.tomados:
            if k in a.granos:
                a.granos[k].tomado = True
        # De cerca la foto ve solo una parte del árbol: lo que quedó fuera se mantiene como se vio antes (su
        # posición está en el marco del árbol, así que sigue valiendo con la pose nueva)
        heredados = 0
        if self.analisis is not None:
            for k, g in a.granos.items():
                v = self.analisis.granos.get(k)
                if g.categoria == "FUERA" and v is not None and v.categoria != "FUERA":
                    v.uv, v.r_px, v.det = g.uv, g.r_px, None
                    a.granos[k] = v
                    heredados += 1
        self.analisis, self.pose_pie = a, a.pose
        # Con la cámara calibrada CON EL BRAZO (M + N), la foto da las posiciones en el sistema del brazo: si el LiDAR
        # está corrido (lidar_x/y mal medidos), esa diferencia se sigue aplicando al volver a ubicar el árbol. Si no,
        # manda el LiDAR (con la cámara calibrada con el LiDAR, la diferencia sería solo ruido de la foto).
        if med is not None and "brazo" in (self.cfg.camara.calibrada or ""):
            self.delta_lidar = (a.pose[0] - med.x_cara, (a.pose[1] - med.y_centro) if med.centro_medido else 0.0)
        elif med is not None:
            self.delta_lidar = (0.0, 0.0)
        self._actualizar_alcance()
        cuenta = collections.Counter(g.categoria for g in a.granos.values())
        self.log(f"análisis ({time.monotonic() - t0:.1f} s): pie x {a.pose[0]:.3f} y {a.pose[1]:+.3f} "
                 f"{math.degrees(a.pose[2]):+.1f}° | {_grados(a.dcam)} | {a.n_asignadas} granos, error "
                 f"{a.error_px:.1f} px | " + " ".join(f"{k} {v}" for k, v in sorted(cuenta.items())))
        for m in a.mensajes:
            self.log("  " + m)
        if heredados:
            self.log(f"  {heredados} soporte(s) fuera de esta foto: quedan como se vieron en la anterior")
        matiz = [g for g in a.granos.values() if g.det is not None and g.det.por_matiz and g.categoria in self.categorias]
        if matiz and max(g.confianza for g in matiz) < self.confianza_min:
            self.log(f"  por matiz la confianza es {max(g.confianza for g in matiz):.1f} < {self.confianza_min:.1f}: no se "
                     "agarra nada hasta enseñar los colores (clic + 1..7) o bajar --confianza")
        return a

    # ------------------------------------------------------------------ granos
    def p_robot(self, g) -> np.ndarray:
        """Centro del grano en el robot AHORA (desde su posición en el árbol y la última pose del pie)."""
        return self.modelo.desde_arbol(g.q_arbol, self.pose_pie)

    def seguros(self) -> list:
        if self.analisis is None:
            return []
        return [g for g in self.analisis.seguros(self.categorias, self.confianza_min)
                if g.clave not in self.tomados]

    def _actualizar_alcance(self):
        if self.brazo is None or self.analisis is None:
            self.alcance = {}
            return
        self.alcance = {k: g.categoria != "FUERA" and self.brazo.alcanzable(self.p_robot(g))
                        for k, g in self.analisis.granos.items()}

    def alcanzable(self, g) -> bool:
        return bool(self.alcance.get(g.clave, False))

    def base_brazo(self) -> np.ndarray:
        cm = self.cfg.mision
        return self.brazo.base if self.brazo is not None else np.array([cm.brazo_x, cm.brazo_y, cm.brazo_z])

    def mas_cercano(self):
        c = [g for g in self.seguros() if self.alcanzable(g)]
        return min(c, key=lambda g: float(np.linalg.norm(self.p_robot(g) - self.base_brazo())), default=None)

    def elegir_en(self, u, v):
        """Elige el soporte cuya proyección queda más cerca del píxel (u, v) de la imagen analizada."""
        if self.analisis is None:
            return None
        mejor = None
        for k, g in self.analisis.granos.items():
            d = math.hypot(g.uv[0] - u, g.uv[1] - v) / max(g.r_px, 4.0)
            if d < 1.6 and (mejor is None or d < mejor[0]):
                mejor = (d, k)
        self.sel = mejor[1] if mejor else None
        self.version += 1
        return self.sel

    # ------------------------------------------------------------------ colores
    def ensenar(self, clase: str):
        a = self.analisis
        if a is None or self.sel is None:
            self.log("elegir un soporte (clic) en la foto analizada")
            return
        g = a.granos[self.sel]
        if g.categoria == "FUERA":
            self.log(f"{g.nombre} está fuera de la foto")
            return
        u, v, r = g.det.zona if g.det is not None and g.det.zona else G.zona_modelo(a.cam, self.modelo, a.pose, g.p,
                                                                                     g.uv, g.r_px)
        labs = [x for x in (lab_zona(img, u, v, r, self.clasif.config) for img in self.imgs) if x is not None]
        if not labs:
            self.log(f"{g.nombre}: la zona se sale de la imagen")
            return
        lab = tuple(float(x) for x in np.mean(labs, axis=0))
        self.clasif.agregar(clase, lab, f"cosecha {time.strftime('%m-%d %H:%M')} {g.nombre}")
        self.clasif.guardar(self.ruta_referencias)
        n = sum(1 for m in self.clasif.muestras if m["clase"] == clase)
        faltan = [c for c in cl.CLASES if not any(m["clase"] == c for m in self.clasif.muestras)]
        self.log(f"{g.nombre} = {clase} (Lab {lab[0]:.0f} {lab[1]:.0f} {lab[2]:.0f}): {n} muestra(s) de {clase}"
                 + (f" | faltan: {' '.join(faltan)} (mientras, salen NO_SE)" if faltan else ""))
        self.analizar(fotos_nuevas=False)

    def resumen_referencias(self) -> str:
        if not self.clasif.muestras:
            return "sin referencias: colores por MATIZ (provisorio)"
        c = collections.Counter(m["clase"] for m in self.clasif.muestras)
        return " ".join(f"{k[:3]} {c.get(k, 0)}" for k in cl.CLASES)

    # ------------------------------------------------------------------ calibrar la cámara
    def _guardar_camara(self, cam: CamaraPinhole, como: str, posicion: bool = False):
        self.cam = self.cam.con(pitch=cam.pitch, yaw=cam.yaw, roll=cam.roll, pos=np.array(cam.pos, float))
        vals = {"inclinacion_arbol_deg": round(math.degrees(cam.pitch), 2), "yaw_deg": round(math.degrees(cam.yaw), 2),
                "roll_deg": round(math.degrees(cam.roll), 2), "calibrada": time.strftime("%Y-%m-%d %H:%M ") + como}
        if posicion:
            vals.update(x=round(float(cam.pos[0]), 4), y=round(float(cam.pos[1]), 4), z=round(float(cam.pos[2]), 4))
        for k, v in vals.items():
            setattr(self.cfg.camara, k, v)
        ruta = guardar_local("camara", vals, self.ruta_local)
        self.log(f"cámara calibrada ({como}): incl {vals['inclinacion_arbol_deg']:+.2f}° yaw {vals['yaw_deg']:+.2f}° "
                 f"roll {vals['roll_deg']:+.2f}°" + (" lente en ({:.3f}, {:+.3f}, {:.3f})".format(*cam.pos) if posicion
                                                     else "") + f" -> {ruta}")

    def calibrar_con_arbol(self):
        """C: la cámara que mejor proyecta los granos detectados con el árbol donde lo midió el LiDAR (o --frente)."""
        a = self.analisis
        if a is None:
            self.log("primero analizar (ESPACIO)")
            return None
        ref = a.lidar.pose if a.lidar is not None else (self.prior_fijo[0] if self.prior_fijo else None)
        if ref is None:
            self.log("sin medida del pie (LiDAR o --frente) no hay con qué comparar")
            return None
        if a.lidar is not None and not a.lidar.centro_medido:
            self.log(f"el LiDAR ve {100 * a.lidar.ancho:.0f} cm de pie y el modelo dice {100 * self.modelo.pie_ancho_m:.0f}: "
                     "no midió el centro, así que no sirve de referencia (revisar pie_ancho_m del árbol)")
            return None
        asig = {d.clave: i for i, d in enumerate(a.detecciones) if d.clave is not None}
        if len(asig) < 3:
            self.log(f"hacen falta ≥ 3 granos detectados (hay {len(asig)})")
            return None
        base = self.cam.escalada(a.cam.ancho, a.cam.alto).con(pitch=a.cam.pitch)   # parte de la inclinación hallada
        cam, rms = G.calibrar_camara(a.detecciones, asig, self.modelo, ref, base)
        self.calibs.append((cam.pitch, cam.yaw, cam.roll))
        p, y, r = np.mean(self.calibs, axis=0)
        self._guardar_camara(self.cam.con(pitch=p, yaw=y, roll=r), f"árbol+LiDAR, {len(self.calibs)} foto(s), "
                                                                    f"{rms:.1f} px")
        if len(self.calibs) == 1:
            self.log("  mejor: repetir (ESPACIO + C) en 3-4 posiciones del robot; se promedian")
        return self.cam, rms

    def marcar(self):
        """M: la garra (movida a mano, brazo suelto) está en el grano elegido: su posición por la cinemática directa
        y dónde se vio ese grano en la foto son un par para calibrar la cámara RESPECTO DEL BRAZO."""
        if self.brazo is None or self.analisis is None or self.sel is None:
            self.log("marcar: analizar con el brazo fuera de la foto, elegir (clic) un grano detectado, llevar la "
                     "garra a ese grano (O suelta el brazo) y apretar M")
            return
        g = self.analisis.granos[self.sel]
        if g.det is None:
            self.log(f"{g.nombre}: no tiene pelota detectada en la foto")
            return
        p, _ = self.brazo.directa_robot(self.brazo.bus.angulos(self.brazo.IDS))
        self.marcas = [m for m in self.marcas if m[2] != g.nombre] + [
            (p, (g.det.u, g.det.v), g.nombre, G.incertidumbre_relativa(g.det), self.analisis.cam.ancho)]
        self.log(f"marca {len(self.marcas)}: {g.nombre}, garra en ({p[0]:.3f}, {p[1]:+.3f}, {p[2]:.3f}); la foto lo "
                 f"ponía a {1000 * np.linalg.norm(p - self.p_robot(g)):.0f} mm de ahí")

    def calibrar_con_brazo(self):
        """N: pose COMPLETA de la cámara (también la posición del lente) con las marcas del brazo."""
        if len(self.marcas) < 4:
            self.log(f"hacen falta ≥ 4 marcas en distintas filas y columnas (hay {len(self.marcas)})")
            return None
        P = np.array([m[0] for m in self.marcas])
        obs = np.array([np.asarray(m[1]) * self.cam.ancho / m[4] for m in self.marcas])
        cam, rms = G.calibrar_extrinsecos(P, obs, self.cam, [m[3] for m in self.marcas], mover_pos=True, sig_pos=0.03)
        self._guardar_camara(cam, f"brazo, {len(self.marcas)} marcas, {rms:.1f} px", posicion=True)
        return cam, rms

    def inclinar(self, d_deg: float):
        self.cam = self.cam.con(pitch=self.cam.pitch + math.radians(d_deg))
        self.log(f"inclinación de prueba {math.degrees(self.cam.pitch):.1f}° (C la calibra y la guarda)")

    # ------------------------------------------------------------------ brazo
    def _brazo_ok(self) -> bool:
        if self.brazo is None:
            self.log("sin brazo (--sin-brazo)")
            return False
        if self.suelto:
            self.log("el brazo está SUELTO: O para sujetarlo primero")
            return False
        self.brazo.parar.clear()
        return True

    def agarrar(self, clave=None) -> bool:
        if not self._brazo_ok() or self.analisis is None:
            return False
        g = self.analisis.granos.get(clave) if clave is not None else self.mas_cercano()
        if g is None:
            self.log("no hay granos seguros al alcance del brazo" + ("" if self.seguros() else " (ni seguros)")
                     + ": ¿acercarse con A?")
            return False
        if g.categoria not in self.categorias or g.confianza < self.confianza_min or g.tomado:
            self.log(f"{g.nombre}: {g.categoria} {g.clase} {g.confianza:.2f} -> no se toca")
            return False
        p = self.p_robot(g)
        if self.mundo is not None:
            self.mundo.antes_de_agarrar(g.clave, p)
        destino = "deposito_maduro" if g.categoria == cancha.MADURO else "deposito_sobremaduro"
        self.log(f"agarrando {g.nombre} ({g.clase}) en ({p[0]:.3f}, {p[1]:+.3f}, {p[2]:.3f}) -> {destino}")
        ok = self.brazo.recoger(p, g.categoria)
        if ok:
            self.tomados.add(g.clave)
            g.tomado = True
            if self.mundo is not None:
                self.mundo.quitar(g.clave)
            self.log(f"{g.nombre}: listo ({destino})")
        else:
            self.log(f"{g.nombre}: no se pudo ({self.brazo.ultimo_motivo})")
        self.version += 1
        return ok

    def agarrar_todos(self) -> int:
        orden = list(self.cfg.mision.orden_filas)
        n = 0
        for g in sorted(self.seguros(), key=lambda g: (orden.index(g.clave[0]) if g.clave[0] in orden else 9,
                                                       abs(self.p_robot(g)[1]))):
            if self.brazo is not None and self.brazo.parar.is_set():
                break
            if not self.alcanzable(g):
                self.log(f"{g.nombre}: fuera del alcance del brazo")
                continue
            n += bool(self.agarrar(g.clave))
        self.log(f"agarrados {n}")
        return n

    def antes(self) -> bool:
        if not self._brazo_ok() or self.analisis is None or self.sel is None:
            if self.sel is None:
                self.log("elegir un grano (clic)")
            return False
        g = self.analisis.granos[self.sel]
        plan = self.brazo.plan_agarre(self.p_robot(g))
        if plan is None:
            self.log(f"{g.nombre}: fuera del alcance del brazo")
            return False
        self.brazo.garra(abrir=True, esperar=False)
        ok = self.brazo._mover(plan[0][1], self.cfg.brazo.t_mov_s, "antes")
        self.log(f"{g.nombre}: garra {100 * self.cfg.brazo.aproximacion_m:.0f} cm antes del grano: ¿apunta a él? "
                 "(V vuelve)" if ok else f"{g.nombre}: {self.brazo.ultimo_motivo}")
        return ok

    def espera(self):
        if self._brazo_ok():
            self.brazo.ir_plegado(self.cfg.brazo.postura_espera)
            self.log("brazo en la postura de espera")

    def soltar(self):
        if self.brazo is None:
            self.log("sin brazo")
            return
        self.suelto = not self.suelto
        self.brazo.bus.torque(not self.suelto, self.brazo.IDS)
        self.log("brazo SUELTO (sostenlo: se cae). Llévalo a mano y M marca" if self.suelto else "brazo sujeto")

    # ------------------------------------------------------------------ base
    def _pose_mega(self) -> np.ndarray:
        from cafebot.drivers.ir_simulado import pose_mega_extrapolada
        time.sleep(0.15)
        return pose_mega_extrapolada(self.mega)

    def mover_base(self, mm: float, vel: float = 80.0, cangrejo_deg: float = 0.0) -> bool:
        """Avanza (o, con cangrejo_deg = ±90, se corre de lado: + = izquierda) y vuelve a ubicar el árbol: primero
        con la odometría y después con el LiDAR."""
        if self.mega is None:
            self.log("sin Mega (--sin-mega)")
            return False
        p0 = self._pose_mega()
        t = 10 + 3 * abs(mm) / max(vel, 20.0)
        r = (self.mega.cangrejo(cangrejo_deg, mm, vel, timeout=t) if cangrejo_deg else
             self.mega.avanzar(mm, vel, timeout=t))
        p1 = self._pose_mega()
        if self.pose_pie is not None:                     # por odometría...
            self.pose_pie = tuple(float(v) for v in relativo(p1, componer(p0, np.array(self.pose_pie))))
        med = self.medir_lidar()                          # ... y corregido con el LiDAR (con lo que la foto le
        if med is not None and self.pose_pie is not None:  # corrigió en la última medida)
            dx, dy = self.delta_lidar
            y = self.pose_pie[1]
            if med.centro_medido and abs(med.y_centro + dy - y) < 0.05:
                y = med.y_centro + dy
            elif med.centro_medido:
                self.log(f"el LiDAR pone el pie {100 * (med.y_centro + dy - y):+.0f} cm de lado respecto de la "
                         "odometría: me quedo con la odometría (¿otro objeto? ESPACIO vuelve a medir todo)")
            self.pose_pie = (med.x_cara + dx, y, med.ang)
        self._actualizar_alcance()
        frente = "" if self.pose_pie is None else f" | pie a {self.pose_pie[0] - self.cfg.geometria.largo / 2:.3f} m del frente"
        que = "base" if not cangrejo_deg else ("a la izquierda" if cangrejo_deg > 0 else "a la derecha")
        self.log(f"{que} {mm:+.0f} mm: {'ok' if r.ok else r.motivo}{frente}"
                 + ("" if med is not None else " (el LiDAR no vio el pie: solo odometría)"))
        return r.ok

    def acercar(self):
        if self.pose_pie is None:
            self.log("primero medir (ESPACIO)")
            return
        d = self.pose_pie[0] - self.cfg.geometria.largo / 2 - self.cfg.mision.holgura_agarre_m
        if d < 0.005:
            self.log("ya está a la distancia de agarre")
            return
        self.mover_base(min(d, 0.8) * 1000, 70)

    def alejar(self):
        if self.pose_pie is None:
            self.log("primero medir (ESPACIO)")
            return
        d = self.pose_pie[0] - self.cfg.geometria.largo / 2 - self.cfg.mision.distancia_foto_m
        self.mover_base(max(d, -0.8) * 1000, 80)

    def parar(self):
        if self.mega is not None:
            try:
                self.mega.parar()
            except Exception:
                pass
        if self.brazo is not None:
            self.brazo.parar.set()
        self.log("ESC: parar")

    # ------------------------------------------------------------------ guardar
    def resumen(self) -> dict:
        a = self.analisis
        d = {"camara": {"pitch_deg": math.degrees(self.cam.pitch), "yaw_deg": math.degrees(self.cam.yaw),
                        "roll_deg": math.degrees(self.cam.roll), "pos": [float(x) for x in self.cam.pos]},
             "arbol": self.modelo.nombre}
        if a is None:
            return d
        d.update(pie=[float(x) for x in a.pose], dcam_deg=[float(x) for x in np.degrees(a.dcam)],
                 error_px=float(a.error_px), mensajes=a.mensajes,
                 lidar=None if a.lidar is None else [float(x) for x in a.lidar.pose],
                 granos={g.nombre: {"clase": g.clase, "categoria": g.categoria, "confianza": round(g.confianza, 3),
                                    "p": [round(float(x), 4) for x in self.p_robot(g)],
                                    "uv": [round(float(x), 1) for x in g.uv], "tomado": g.tomado,
                                    "detectado": g.det is not None} for g in a.granos.values()})
        return d

    def guardar(self) -> str:
        os.makedirs(cl.CARPETA_CAPTURAS, exist_ok=True)
        base = os.path.join(cl.CARPETA_CAPTURAS, time.strftime("%Y%m%d_%H%M%S_cosecha"))
        for i, img in enumerate(self.imgs):
            cv2.imwrite(f"{base}_{i + 1}.jpg", img)
        if self.analisis is not None and self.imgs:
            cv2.imwrite(base + "_marcada.jpg", dibujar(self.imgs[0], self))
        with open(base + ".json", "w", encoding="utf-8") as f:
            json.dump(self.resumen(), f, indent=2, ensure_ascii=False)
        self.log(f"guardado: {base}_*.jpg / .json")
        return base


# ---------------------------------------------------------------------------- dibujo (OpenCV, BGR)
def dibujar(img, ses: Cosecha, proyectar=True):
    out = img.copy()
    a = ses.analisis
    if a is None:
        return out
    k = img.shape[1] / a.cam.ancho
    for clave, g in a.granos.items():
        if g.uv is None or not proyectar:
            continue
        u, v, r = g.uv[0] * k, g.uv[1] * k, max(3.0, g.r_px * k)
        col = COLOR_CAT.get(g.categoria, (255, 255, 255))
        cv2.circle(out, (int(u), int(v)), int(r), col, 1 if g.categoria == "FUERA" else 2, cv2.LINE_AA)
        if g.categoria == "FUERA":
            continue
        txt = f"{g.nombre} {CORTO.get(g.clase, g.clase[:3])}"
        if g.categoria in ses.categorias:
            txt += f" {g.confianza:.1f}" + ("" if ses.brazo is None or ses.alcanzable(g) else " lejos")
        if g.tomado:
            cv2.line(out, (int(u - r), int(v - r)), (int(u + r), int(v + r)), col, 2)
            cv2.line(out, (int(u - r), int(v + r)), (int(u + r), int(v - r)), col, 2)
        p = (int(u - r), int(v + r + 16))
        cv2.putText(out, txt, p, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(out, txt, p, cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
    for d in a.detecciones:
        c = (int(d.u * k), int(d.v * k))
        cv2.circle(out, c, int(d.r * k), (255, 255, 255), 1, cv2.LINE_AA)
        if d.zona:
            cv2.circle(out, (int(d.zona[0] * k), int(d.zona[1] * k)), max(1, int(d.zona[2] * k)), (255, 255, 255), 1)
        if d.clave is None:
            cv2.drawMarker(out, c, (255, 0, 255), cv2.MARKER_TILTED_CROSS, int(d.r * k), 2)
    if ses.sel is not None and ses.sel in a.granos:
        g = a.granos[ses.sel]
        cv2.circle(out, (int(g.uv[0] * k), int(g.uv[1] * k)), int(max(6, g.r_px * k) + 7), (0, 255, 255), 3, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------- ventana (pygame)
def ventana(ses: Cosecha, titulo: str, en_vivo_posible: bool):
    import pygame as pg
    pg.init()
    win = pg.display.set_mode((1360, 800), pg.RESIZABLE)
    pg.display.set_caption(titulo)
    fuente = pg.font.SysFont("dejavusansmono,monospace", 13)
    negrita = pg.font.SysFont("dejavusans,sans", 16, bold=True)
    reloj = pg.time.Clock()
    vivo, ultimo_vivo, img_vivo = False, 0.0, None
    sup_img, version_dibujada = None, -1
    while True:
        W, H = win.get_size()
        iw = int(W * 0.68)
        ih = int(iw * 9 / 16)
        for ev in pg.event.get():
            if ev.type == pg.QUIT:
                return
            if ev.type == pg.MOUSEBUTTONDOWN and ev.button == 1 and ev.pos[0] < iw and ev.pos[1] < ih and ses.imgs:
                h0, w0 = ses.imgs[0].shape[:2]
                if ses.analisis is not None:
                    k = ses.analisis.cam.ancho / w0
                    g = ses.elegir_en(ev.pos[0] * w0 / iw * k, ev.pos[1] * h0 / ih * k)
                    if g is not None:
                        gr = ses.analisis.granos[g]
                        p = ses.p_robot(gr)
                        ses.log(f"elegido {gr.nombre}: {gr.clase} ({gr.categoria} {gr.confianza:.2f}) en "
                                f"({p[0]:.3f}, {p[1]:+.3f}, {p[2]:.3f})" + ("" if ses.brazo is None else
                                ", alcanzable" if ses.alcanzable(gr) else ", FUERA de alcance"))
            if ev.type != pg.KEYDOWN:
                continue
            k = ev.key
            if k == pg.K_q:
                ses.parar()
                return
            if k == pg.K_ESCAPE:
                ses.parar()
            elif k == pg.K_SPACE:
                vivo = False
                ses.en_fondo("análisis", ses.analizar)
            elif k == pg.K_l and en_vivo_posible:
                vivo = not vivo
                ses.log("vista en vivo " + ("ON (sin análisis; ESPACIO analiza)" if vivo else "OFF"))
            elif k == pg.K_s:
                ses.guardar()
            elif pg.K_1 <= k <= pg.K_7:
                ses.en_fondo("enseñar color", ses.ensenar, TECLAS_CLASE[k - pg.K_1])
            elif k in (pg.K_i, pg.K_k):
                ses.inclinar(0.5 if k == pg.K_i else -0.5)
                if ses.imgs:
                    ses.en_fondo("análisis", ses.analizar, False)
            elif k == pg.K_c:
                ses.en_fondo("calibrar con el árbol", ses.calibrar_con_arbol)
            elif k == pg.K_o:
                ses.soltar()
            elif k == pg.K_m:
                ses.marcar()
            elif k == pg.K_n:
                ses.en_fondo("calibrar con el brazo", ses.calibrar_con_brazo)
            elif k == pg.K_p:
                ses.en_fondo("garra antes", ses.antes)
            elif k == pg.K_v:
                ses.en_fondo("postura de espera", ses.espera)
            elif k == pg.K_g:
                ses.en_fondo("agarrar", ses.agarrar, ses.sel)
            elif k == pg.K_t:
                ses.en_fondo("agarrar todos", ses.agarrar_todos)
            elif k == pg.K_a:
                ses.en_fondo("acercarse", ses.acercar)
            elif k == pg.K_r:
                ses.en_fondo("alejarse", ses.alejar)
            elif k in (pg.K_UP, pg.K_DOWN):
                ses.en_fondo("mover", ses.mover_base, 20.0 if k == pg.K_UP else -20.0, 60.0)
            elif k in (pg.K_LEFT, pg.K_RIGHT):
                ses.en_fondo("cangrejo", ses.mover_base, 20.0, 50.0, 90.0 if k == pg.K_LEFT else -90.0)

        # imagen
        if vivo and not ses.ocupado and time.monotonic() - ultimo_vivo > 0.3:
            img_vivo, ultimo_vivo = ses.camara.capturar(), time.monotonic()
            version_dibujada = -1
        if version_dibujada != ses.version or sup_img is None or sup_img.get_width() != iw:
            if vivo and img_vivo is not None:
                base = dibujar(img_vivo, ses) if ses.analisis is not None else img_vivo
            elif ses.imgs:
                base = dibujar(ses.imgs[0], ses)
            else:
                base = np.zeros((ih, iw, 3), np.uint8)
            rgb = cv2.cvtColor(cv2.resize(base, (iw, ih), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2RGB)
            sup_img = pg.image.frombuffer(rgb.tobytes(), (iw, ih), "RGB")
            version_dibujada = ses.version
        win.fill((14, 16, 22))
        win.blit(sup_img, (0, 0))
        _panel(win, ses, fuente, negrita, (iw + 10, 8, W - iw - 18, H - 16))
        _planta(win, ses, fuente, (iw + 10, H - min(300, H // 2 - 40) - 8, W - iw - 18, min(300, H // 2 - 40)))
        y = ih + 8
        for m in list(ses.mensajes)[-((H - ih - 16) // 17):]:
            while len(m) > 4 and fuente.size(m)[0] > iw - 12:
                m = m[:-2] + "…"
            win.blit(fuente.render(m, True, (200, 205, 215)), (8, y))
            y += 17
        pg.display.flip()
        reloj.tick(20)


def _panel(win, ses: Cosecha, fuente, negrita, rect):
    import pygame as pg
    x, y, w, h = rect
    cc, a = ses.cfg.camara, ses.analisis

    def linea(t, color=(210, 215, 225), f=None):
        nonlocal y
        f = f or fuente
        while len(t) > 4 and f.size(t)[0] > w:
            t = t[:-2] + "…"
        win.blit(f.render(t, True, color), (x, y))
        y += 18 if f is negrita else 17

    linea(f"COSECHA  árbol: {ses.modelo.nombre}", (255, 230, 120), negrita)
    linea("cámara: incl {:.1f}° yaw {:+.1f}° roll {:+.1f}°".format(*np.degrees([ses.cam.pitch, ses.cam.yaw, ses.cam.roll])))
    linea("  " + (f"calibrada {cc.calibrada}" if cc.calibrada else "SIN CALIBRAR (C, o M + N con el brazo)"),
          (150, 220, 150) if cc.calibrada else (255, 190, 90))
    m = ses.medida
    linea("LiDAR: " + ("-" if ses.lidar is None else "no ve el pie" if m is None else
                       f"pie x {m.x_cara:.3f} y {m.y_centro:+.3f} {math.degrees(m.ang):+.1f}° ancho {m.ancho:.2f}"))
    if a is not None:
        linea(f"árbol: x {a.pose[0]:.3f} y {a.pose[1]:+.3f} {math.degrees(a.pose[2]):+.1f}° | {a.n_asignadas} granos "
              f"{a.error_px:.1f} px")
        linea("  " + _grados(a.dcam))
        seg = ses.seguros()
        linea(f"seguros {len(seg)}, alcanzables {sum(ses.alcanzable(g) for g in seg)}, tomados {len(ses.tomados)}")
    if ses.sel is not None and a is not None:
        g = a.granos[ses.sel]
        p = ses.p_robot(g)
        linea(f"elegido {g.nombre}: {g.clase} {g.categoria} {g.confianza:.2f}", (0, 230, 255))
        linea(f"  ({p[0]:.3f}, {p[1]:+.3f}, {p[2]:.3f}) m" + ("" if ses.brazo is None else
                                                             "  alcanzable" if ses.alcanzable(g) else "  FUERA de alcance"))
    linea("brazo: " + ("-" if ses.brazo is None else ("SUELTO | " if ses.suelto else "") + ses.brazo.estado()))
    if ses.marcas:
        linea(f"marcas para calibrar con el brazo: {len(ses.marcas)} (N con ≥ 4)")
    linea("colores: " + ses.resumen_referencias())
    if ses.ocupado:
        linea(f"... {ses.ocupado}", (255, 200, 80))
    y += 6
    for t in ("ESPACIO analizar · L en vivo · S guardar · Q salir",
              "clic elegir · 1-7 color: ROJ NAR AMA VER AZU NEG VAC",
              "I/K inclinación · C calibrar con el árbol",
              "O soltar brazo · M marcar · N calibrar con el brazo",
              "P antes · G agarrar · T todos · V espera · ESC parar",
              "A acercarse · R alejarse · ↑↓ 2 cm · ←→ cangrejo"):
        linea(t, (140, 150, 170))


def _planta(win, ses: Cosecha, fuente, rect):
    """Vista desde arriba: robot, base del brazo, LiDAR, cara del pie y granos (color = categoría)."""
    import pygame as pg
    x0, y0, w, h = rect
    pg.draw.rect(win, (22, 26, 36), rect)
    esc = min(w / 1.0, h / 1.25)

    def P(px, py):
        return int(x0 + w / 2 - py * esc), int(y0 + h - 8 - (px + 0.2) * esc)

    g = ses.cfg.geometria
    L, A = g.largo / 2, g.ancho / 2
    pg.draw.polygon(win, (90, 110, 150), [P(-L, -A), P(L, -A), P(L, A), P(-L, A)], 2)
    if ses.scan_xy is not None:
        for px, py in ses.scan_xy[::2]:
            if -0.2 < px < 1.05 and abs(py) < 0.5:
                win.fill((120, 200, 255), (*P(px, py), 2, 2))
    if ses.pose_pie is not None:
        x, y, ang = ses.pose_pie
        u, t = ModeloArbol.ejes(ang)
        e = [np.array([x, y]) + s * ses.modelo.pie_ancho_m / 2 * t for s in (1, -1)]
        pg.draw.line(win, (230, 210, 90), P(*e[0]), P(*e[1]), 3)
    b = ses.base_brazo()
    pg.draw.circle(win, (230, 230, 90), P(b[0], b[1]), 5)
    if ses.analisis is not None:
        for k, gr in ses.analisis.granos.items():
            if gr.categoria == "FUERA":
                continue
            p = ses.p_robot(gr)
            col = COLOR_CAT.get(gr.categoria, (255, 255, 255))[::-1]
            r = {"alta": 6, "media": 5, "baja": 4}.get(k[0], 4)        # más grande = más alto
            pg.draw.circle(win, col, P(p[0], p[1]), r, 0 if ses.alcanzable(gr) else 1)
    win.blit(fuente.render("desde arriba (relleno = alcanzable)", True, (140, 150, 170)), (x0 + 6, y0 + 4))


# ---------------------------------------------------------------------------- armar según el modo
def _prior_de_args(cfg, a, ruta_imagen=None):
    """Pose del pie a mano (--frente/--lateral/--angulo) o la guardada junto a la foto."""
    sig = (0.02, 0.02, math.radians(3.0))
    if a.frente is not None:
        return (cfg.geometria.largo / 2 + a.frente, a.lateral or 0.0, math.radians(a.angulo or 0.0)), sig
    if ruta_imagen:
        base = os.path.splitext(ruta_imagen)[0]
        for cand in (base + ".json", base.rsplit("_", 1)[0] + ".json"):
            if os.path.exists(cand):
                d = json.load(open(cand, encoding="utf-8"))
                if d.get("pie"):
                    return tuple(d["pie"]), sig
                if d.get("frente") is not None:      # fotos de tools.camara
                    return (cfg.geometria.largo / 2 + d["frente"], d.get("lateral", 0.0), 0.0), sig
    return None


class _CamaraReducida:
    """Fotos guardadas (p. ej. a 4608×2592) llevadas al tamaño de trabajo."""

    def __init__(self, camara, ancho, alto):
        self.c, self.tam = camara, (ancho, alto)

    def capturar(self):
        img = self.c.capturar()
        if img.shape[1] > self.tam[0] and abs(img.shape[1] / img.shape[0] - self.tam[0] / self.tam[1]) < 0.01:
            img = cv2.resize(img, self.tam, interpolation=cv2.INTER_AREA)
        return img

    def cerrar(self):
        self.c.cerrar()


def preparar_brazo(brazo, cfg, log):
    brazo.conectar()
    ok, motivo = brazo.bus.puede_trabajar()
    if not ok and brazo.bus.recuperar_errores():
        ok, motivo = brazo.bus.puede_trabajar()
    if not ok:
        log(f"brazo: {motivo}")
        return False
    brazo.bus.torque(True)
    brazo.garra(abrir=True, esperar=False)
    if cfg.brazo.postura_espera in brazo.posturas:
        brazo.ir_plegado(cfg.brazo.postura_espera)
    faltan = [p for p in ("deposito_maduro", "deposito_sobremaduro") if p not in brazo.posturas]
    if faltan:
        log(f"brazo: faltan las posturas {', '.join(faltan)} (python3 -m tools.brazo leer, tecla G): sin ellas no "
            "agarra esa categoría (P sí funciona)")
    if "transporte" not in brazo.posturas:
        log("brazo: sin postura 'transporte' (opcional): va directo a las canastas")
    return True


def armar(cfg, a, log):
    """(sesión, cosas que cerrar al salir, puede vista en vivo)."""
    try:
        modelo = ModeloArbol.cargar(a.arbol or cfg.camara.arbol)
    except ValueError as e:
        raise SystemExit(str(e))
    clasif = cl.Clasificador() if a.sin_referencias else cl.Clasificador.cargar(cl.ARCHIVO_REFERENCIAS)
    categorias = ("MADURO",) if a.solo_maduros else ("MADURO", "SOBREMADURO")
    cerrar = []
    if a.sim:
        from tests import bus_dxl_falso
        from tests.sim_serial import SerialSimulado, compilar
        memoria = bus_dxl_falso.instalar()
        from cafebot.arm import brazo as mod_brazo
        from cafebot.arm.brazo import BrazoXL430
        from cafebot.drivers.mega import Mega
        compilar()
        mega = Mega("sim")
        mega.conectar(serial_obj=SerialSimulado())
        mega.telemetria(20)
        cerrar.append(mega.cerrar)
        mundo = MundoSim(cfg, modelo, mega, a.frente if a.frente is not None else 0.62, a.lateral or 0.0,
                         3.0 if a.angulo is None else a.angulo, a.semilla, bus_falso=memoria)
        tmp = pathlib.Path(tempfile.mkdtemp(prefix="cosecha_sim_"))
        mod_brazo.RAIZ_PI = tmp                          # el perfil del brazo simulado no pisa el de verdad
        brazo = None
        if not a.sin_brazo:
            brazo = BrazoXL430(cfg, "falso", log=log)
            posturas_sim(brazo)
            preparar_brazo(brazo, cfg, log)
        cfg.camara.calibrada = ""
        ses = Cosecha(cfg, modelo, mundo.camara, mundo.lidar, mega, brazo, mundo, clasif, categorias=categorias,
                      ruta_local=str(tmp / "robot_local.json"), ruta_referencias=str(tmp / "referencias.json"),
                      confianza_min=a.confianza, log=log)
        log("SIMULACIÓN: árbol sintético a {:.2f} m, cámara 'real' con error de montaje (incl {:+.1f}° yaw {:+.1f}° "
            "roll {:+.1f}°): C la calibra".format(mundo.pie_mundo[0] - cfg.geometria.largo / 2,
                                                   *np.degrees([mundo.cam_real.pitch - ses.cam.pitch,
                                                                mundo.cam_real.yaw - ses.cam.yaw,
                                                                mundo.cam_real.roll - ses.cam.roll])))
        return ses, cerrar, True
    if a.imagen:
        from cafebot.vision.camara import CamaraArchivo
        prior = _prior_de_args(cfg, a, a.imagen[0])
        if prior is None:
            raise SystemExit("¿Dónde estaba el árbol? --frente (m del frente del robot a la cara del pie) "
                             "[--lateral m a la izquierda] [--angulo °]")
        camara = _CamaraReducida(CamaraArchivo(a.imagen), cfg.camara.ancho_px, cfg.camara.alto_px)
        cfg.camara.fotos = len(a.imagen)
        return Cosecha(cfg, modelo, camara, clasif=clasif, prior_fijo=prior, categorias=categorias,
                       confianza_min=a.confianza, log=log), [], False
    # robot real
    from cafebot.drivers.ld06 import LD06
    from cafebot.vision.camara import Camara, cargar_ajustes
    camara = Camara(cfg.camara.foco_m)
    cerrar.append(camara.cerrar)
    aj = cargar_ajustes()
    if aj:
        camara.aplicar(aj)
    else:
        log("[AVISO] sin ajustes de exposición: se fijan ahora (mejor: python3 -m tools.camara exposicion)")
        camara.bloquear_auto()
    lidar = None
    try:
        lidar = LD06(a.lidar or cfg.puertos.lidar, cfg.geometria.config_ld06())
        lidar.iniciar()
        cerrar.append(lidar.detener)
        if lidar.esperar_scan(2.5) is None:
            e = lidar.estadisticas()
            log(f"[AVISO] el LiDAR ({lidar.puerto}) no entrega vueltas: {e['ok']} paquetes buenos, {e['crc_malos']} con "
                "error. ¿Gira? ¿Es ese el puerto del LD06? (ls -l /dev/cafebot-* ; python3 -m tools.visor_lidar)")
    except Exception as e:
        lidar = None
        log(f"[AVISO] sin LiDAR ({e}): el árbol sale de --frente/--lateral")
    mega = None
    if not a.sin_mega:
        from cafebot.drivers.mega import Mega
        try:
            mega = Mega(a.mega or cfg.puertos.mega)
            mega.conectar()
            mega.telemetria(20)
            cerrar.append(mega.cerrar)
        except Exception as e:
            mega = None
            log(f"[AVISO] sin Mega ({e}): la base no se mueve")
    brazo = None
    if not a.sin_brazo:
        from cafebot.arm.brazo import BrazoXL430, PerfilBrazo
        try:
            perfil = PerfilBrazo.cargar(a.perfil) if a.perfil else None
            brazo = BrazoXL430(cfg, a.u2d2, log=log, perfil=perfil)
            if preparar_brazo(brazo, cfg, log):
                cerrar.append(brazo.cerrar)
            else:
                brazo = None
        except Exception as e:
            brazo = None
            log(f"[AVISO] sin brazo ({e})")
    prior = _prior_de_args(cfg, a)
    if lidar is None and prior is None:
        raise SystemExit("sin LiDAR hay que decir dónde está el árbol: --frente (m) [--lateral m] [--angulo °]")
    return Cosecha(cfg, modelo, camara, lidar, mega, brazo, None, clasif, prior_fijo=prior, categorias=categorias,
                   confianza_min=a.confianza, log=log), cerrar, True


def imprimir(ses: Cosecha):
    a = ses.analisis
    print(f"\nPie del árbol: x {a.pose[0]:.3f} y {a.pose[1]:+.3f} ángulo {math.degrees(a.pose[2]):+.1f}° | "
          f"{_grados(a.dcam)} | {a.n_asignadas} granos, error {a.error_px:.1f} px")
    for fila in ("alta", "media", "baja"):
        celdas = []
        for k, g in sorted(a.granos.items()):
            if k[0] == fila:
                celdas.append(f"{g.nombre:8s}{CORTO.get(g.clase, g.clase[:3]):4s}{g.confianza:4.2f}")
        print("  " + " | ".join(celdas))
    print("Granos seguros (para el brazo, en el robot):")
    for g in ses.seguros():
        p = ses.p_robot(g)
        print(f"  {g.nombre:8s} {g.clase:9s} {g.categoria:12s} ({p[0]:.3f}, {p[1]:+.3f}, {p[2]:.3f}) m")
    for m in a.mensajes:
        print("  ", m)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="todo simulado")
    ap.add_argument("--semilla", type=int, default=None, help="(sim) colores y error de la cámara")
    ap.add_argument("--imagen", nargs="+", default=None, help="fotos guardadas (sin robot)")
    ap.add_argument("--frente", type=float, default=None, help="m del frente del robot a la cara del pie del árbol")
    ap.add_argument("--lateral", type=float, default=None, help="m que el centro del pie está a la IZQUIERDA")
    ap.add_argument("--angulo", type=float, default=None, help="° del árbol (normal de la cara; + = girado a la izq.)")
    ap.add_argument("--arbol", default=None, help="modelo del árbol: competencia o pi/arboles/<nombre>.json")
    ap.add_argument("--crear-arbol", action="store_true", help="crea pi/arboles/<--arbol>.json para editar")
    ap.add_argument("--inclinacion", type=float, default=None, help="° de inclinación de la cámara (sin calibrar)")
    ap.add_argument("--perfil", default=None, help="perfil del brazo (pi/brazos/<perfil>.json)")
    ap.add_argument("--sin-brazo", action="store_true")
    ap.add_argument("--sin-mega", action="store_true")
    ap.add_argument("--solo-maduros", action="store_true", help="no agarrar sobremaduros")
    ap.add_argument("--sin-referencias", action="store_true", help="colores por matiz aunque haya referencias")
    ap.add_argument("--confianza", type=float, default=None,
                    help="confianza mínima para agarrar (por defecto la de la misión, 0,7; por matiz sale 0,6)")
    ap.add_argument("--mega", default=None)
    ap.add_argument("--lidar", default=None)
    ap.add_argument("--u2d2", default=None)
    ap.add_argument("--texto", action="store_true", help="sin ventana: analiza, imprime y guarda la imagen marcada")
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    cfg = cargar()
    if a.crear_arbol:
        if not a.arbol or a.arbol == "competencia":
            raise SystemExit("--crear-arbol necesita --arbol <nombre> (p. ej. taller)")
        ruta = ModeloArbol(nombre=a.arbol).guardar(a.arbol)
        print(f"Creado {ruta} con las medidas de competencia. Medir en el árbol de prueba y editar:\n"
              "  z_cara        altura (m) de la CARA DE ARRIBA de los soportes de cada fila\n"
              "  columnas_y    posición lateral (m, + = izquierda vista desde el robot) de cada soporte, por fila\n"
              "  copa_y_m      corrimiento lateral de todos los soportes respecto del centro del pie\n"
              "  grano_delante_m  cuánto está el CENTRO de los granos delante de la cara que ve el LiDAR (a 5 cm del suelo)\n"
              "  pie_ancho_m   ancho de lo que ve el LiDAR a esa altura\n"
              "  soporte_lado_m / soporte_espesor_m / centro_sobre_cara_m  la placa de cada soporte\n"
              f"Después: python3 -m tools.cosecha --arbol {a.arbol}")
        return
    if a.inclinacion is not None:
        cfg.camara.inclinacion_arbol_deg = a.inclinacion
        cfg.camara.calibrada = ""
    ses, cerrar, vivo = armar(cfg, a, print)
    try:
        if a.texto:
            ses.analizar()
            imprimir(ses)
            ruta = ses.guardar()
            print(f"Imagen marcada: {ruta}_marcada.jpg")
            return
        ventana(ses, "Cafebot: cosecha" + (" (SIMULACIÓN)" if a.sim else " (fotos)" if a.imagen else ""), vivo)
    finally:
        ses.esperar(5.0)
        for f in reversed(cerrar):
            try:
                f()
            except Exception:
                pass


if __name__ == "__main__":
    main()
