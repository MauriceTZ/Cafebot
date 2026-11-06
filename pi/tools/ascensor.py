"""Cosecha por PISOS con el robot de verdad: el brazo como ASCENSOR. Para probar y ajustar a mano (docs/ascensor.md).

La cámara solo dice QUÉ hay (los 3 pisos, qué soportes tienen pelota y su color: cafebot/vision/pisos.py). DÓNDE:
la línea negra del árbol (IR delanteros) da la distancia y el rumbo, el LiDAR el lado, y las alturas de los pisos
son las del reglamento (config.ascensor, se ajustan con las teclas). Para cada pelota: el robot se aleja de la línea
lo de ese piso, se corre en cangrejo hasta tenerla DELANTE del brazo, el brazo SUBE al piso con la garra abierta,
entra derecho, cierra, saca la pelota y la deja en el robot.

    python3 -m tools.ascensor                          # Mega + LiDAR + brazo + cámara
    python3 -m tools.ascensor --sin-lidar --dist 0.55  # sin LiDAR: el robot CENTRADO y derecho frente al pie,
                                                       # el frente a 0,55 m (huincha)
    python3 -m tools.ascensor --sin-brazo              # probar foto + acercamiento + cangrejos
    python3 -m tools.ascensor --auto --boton enter     # la ronda completa sola (como la tecla r)

TECLAS (en la terminal, también por SSH; la foto se ve en la pantalla HDMI):
  f  FOTO: los 3 pisos, las pelotas y sus colores (queda en pi/capturas/: *_pisos.jpg anotada y la original)
  a  ACERCARSE: (LiDAR) se cuadra y se centra frente al pie, va a la distancia de la foto, saca la foto si no hay,
     y avanza hasta que los IR delanteros tocan la línea del árbol (se endereza con ella)
  1 2 3  piso BAJA / MEDIA / ALTA            ← →  columna (0 = la de más a la izquierda en la foto)
  p  PISO: se aleja de la línea lo de ese piso, cangrejo hasta tener la columna delante del brazo y el brazo SUBE
     al piso con la garra abierta, unos cm antes de la pelota (aproximación)
  e  ENTRAR hasta la pelota     c  CERRAR la garra (dice si tomó algo)     s  SACAR (sube y retrocede)
  d  DEPOSITAR en el robot (maduro / sobremaduro según la foto) y el brazo a esperar
  g  AGARRE COMPLETO de la columna elegida (p + e + c + s + d)
  r  RONDA: foto, acercarse y todas las pelotas en orden (maduros del medio, maduros de abajo, sobremaduros...)
  ↑ ↓  altura del piso ±5 mm    i k  hacia el árbol / hacia atrás ±5 mm    j l  izquierda / derecha ±5 mm
  + -  aproximación ±1 cm       (si el brazo está en el piso o en la pelota, se vuelve a mover con el ajuste)
  t  GUARDAR los ajustes en robot_local.json    h  brazo a esperar    v  estado    ?  ayuda
  ESPACIO  PARAR todo           q  salir
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import threading
import time
import traceback

import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, relativo, transformar

FILAS_TECLA = {"1": "baja", "2": "media", "3": "alta"}
COL_Y = {}                                   # fila -> {col: lado respecto del centro del árbol (+ = izquierda)}
_yt = cancha.centros_arboles()[1]
for _s in cancha.soportes(2):
    COL_Y.setdefault(_s.fila, {})[_s.col] = float(_s.y - _yt)


FRENTE_MIN_M = 0.06    # buscando la línea, nunca acercar el frente a menos de esto de la cara del árbol (con los IR
                       # sobre la línea queda a ~8 cm; más cerca, el cabezal de la cámara toca los soportes de abajo)


class Cancelado(Exception):
    pass


class Ascensor:
    def __init__(self, cfg, mega, lidar=None, brazo=None, cam=None, pantalla=None, dist_inicial=None, log=print):
        self.cfg, self.ca, self.cc = cfg, cfg.ascensor, cfg.camara
        self.mega, self.lidar, self.brazo, self.cam, self.pantalla = mega, lidar, brazo, cam, pantalla
        self.dist_inicial = dist_inicial
        self.log = log
        self.medio = cfg.geometria.largo / 2
        self.sx = 0.5 * (cfg.ir.posiciones[0][0] + cfg.ir.posiciones[1][0])       # x de los IR delanteros
        self.arbol_odo = None        # cara del árbol (centro, normal hacia el árbol) en el marco de la odometría
        self.x_linea = None          # x de la cara (en el robot) con los IR delanteros sobre la línea
        self.pisos = None            # última foto analizada (vision/pisos.py)
        self.img = None              # última imagen para la pantalla
        self.fila, self.col = "media", 2
        self.plan, self.p, self.paso = None, None, None     # paso del plan del brazo donde está (0 antes, 2 grano)
        self.hechos = set()
        self.recogidos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0}
        self.cancelar = threading.Event()
        self.hilo = None

    # ------------------------------------------------------------------ básico
    def revisar(self):
        if self.cancelar.is_set():
            raise Cancelado()

    def _ok(self, r, que):
        if not r.ok:
            raise RuntimeError(f"{que}: el Mega dijo {r.motivo}")

    def aj(self, fila, que):
        return float(getattr(self.ca, f"{que}_{fila}_m"))

    def poner(self, fila, que, v):
        setattr(self.ca, f"{que}_{fila}_m", round(float(v), 4))

    def pose_odo(self):
        x, y, th = self.mega.pose()
        return np.array([x / 1000.0, y / 1000.0, th])

    def arbol_rel(self):
        """(x de la cara, y del centro, ángulo de su normal) en el sistema del robot AHORA."""
        return relativo(self.pose_odo(), self.arbol_odo)

    def fijar_arbol(self, x_cara, y_centro, ang=0.0):
        self.arbol_odo = componer(self.pose_odo(), np.array([x_cara, y_centro, ang]))

    def punto(self, fila, col):
        """Centro de la pelota (fila, col) en el robot (x adelante, y izquierda, z desde el suelo), con los ajustes."""
        rel = self.arbol_rel()
        loc = np.array([[-self.ca.grano_delante_m + self.aj(fila, "dx"), COL_Y[fila][col] + self.aj(fila, "dy")]])
        xy = transformar(loc, rel)[0]
        return np.array([xy[0], xy[1], self.aj(fila, "z")])

    def pelota_en(self, fila, col):
        if self.pisos is None:
            return None
        return next((p for p in self.pisos.pelotas if p.clave == (fila, col)), None)

    # ------------------------------------------------------------------ movimientos de la base
    def plegar(self):
        """Antes de mover la base: el brazo recogido (nunca moverse con la garra metida en el árbol)."""
        if self.brazo is not None and self.paso is not None:
            self.brazo.parar.clear()
            self.brazo.ir_plegado(self.cfg.brazo.postura_espera)
            self.paso = None

    def avanzar(self, m, que):
        if abs(m) < 0.003:
            return
        self.revisar()
        self.plegar()
        vel = self.ca.vel_mm_s * (0.6 if abs(m) < 0.10 else 1.0)
        self._ok(self.mega.avanzar(1000 * m, vel, timeout=1000 * abs(m) / vel + 8), que)
        time.sleep(0.2)

    def cangrejo(self, dy, que):
        if abs(dy) < 0.003:
            return
        self.revisar()
        self.plegar()
        vel = self.ca.vel_mm_s
        self._ok(self.mega.cangrejo(90 if dy > 0 else -90, 1000 * abs(dy), vel, timeout=1000 * abs(dy) / vel + 10), que)
        time.sleep(0.3)

    def girar(self, rad, que):
        if abs(rad) < math.radians(0.4):
            return
        self.revisar()
        self.plegar()
        self._ok(self.mega.girar(math.degrees(rad), timeout=10), que)
        time.sleep(0.3)

    # ------------------------------------------------------------------ sensores
    def medir_lidar(self, esperado=None):
        """Pie del árbol con el LiDAR: (x de la cara, y del centro o None si no se ve entero, ángulo de su normal,
        ancho visto). Primero la distancia con los puntos del CENTRO (±12 cm de donde se espera el árbol: en una
        cancha chica las paredes de los lados quedan fuera); después los bordes de la cara a esa distancia."""
        if self.lidar is None:
            return None
        from cafebot.vision.arbol import cara_recta
        x_lo, x_hi, y0 = (0.12, 1.3, 0.0) if esperado is None else (esperado[0] - 0.15, esperado[0] + 0.15, esperado[1])
        for _ in range(3):
            s = self.lidar.esperar_scan(1.0)
            if s is None:
                continue
            x, y = s.xy()
            central = (x > x_lo) & (x < x_hi) & (np.abs(y - y0) < 0.12)
            if central.sum() < 5:
                continue
            xf = float(np.percentile(x[central], 30))
            cerca = (np.abs(x - xf) < 0.04) & (np.abs(y - y0) < 0.30)
            xc, yc = cara_recta(x[cerca], y[cerca], banda=0.03)
            if len(xc) < 5:
                continue
            ancho = float(np.ptp(yc))
            ang = -math.atan(np.polyfit(yc, xc, 1)[0]) if ancho > 0.10 else 0.0
            # la base mide 45 cm: más ancho = se pegó con otra cosa (una pared); menos = no se ve entera
            ok = abs(ancho - cancha.ARBOL_ANCHO_PIE) < 0.05
            centro = 0.5 * (float(yc.min()) + float(yc.max())) if ok else None
            return float(np.median(xc)), centro, ang, ancho
        return None

    def tocar_linea(self, max_m):
        """Avanza hasta que DI y DD tocan la línea y se endereza con ella. Distancia centro -> línea, o None."""
        from cafebot.navegacion.bordes import Bordes
        if max_m < 0.01:
            self.log(f"  (la línea debería estar a {100 * max_m:.1f} cm: no avanzo)")
            return None
        self.revisar()
        self.plegar()
        b = Bordes(self.mega, self.cfg, log=self.log)
        try:
            med = b.medir_adelante(1000 * max_m, vel=80 if max_m > 0.10 else 50)
        finally:
            b.cerrar()
        if med is None:
            return None
        self.log(f"  línea: a {100 * med.dist_m:.1f} cm del centro, torcido {math.degrees(med.angulo):+.1f}°")
        self.girar(-med.angulo, "enderezar con la línea")
        return med.dist_m

    def retocar_linea(self):
        rel = self.arbol_rel()
        d = rel[0] - self.ca.linea_a_cara_m - self.sx          # de los IR delanteros a la línea
        if d < 0.025:
            self.avanzar(-(0.045 - d), "atrás antes de la línea")
            d = 0.045
        rel = self.arbol_rel()
        dist = self.tocar_linea(min(d + 0.06, rel[0] - self.medio - FRENTE_MIN_M))
        if dist is None:
            self.log("  no vi la línea con los IR delanteros (¿calibrados?): sigo con la odometría")
            return False
        y = self.arbol_rel()[1]
        self.x_linea = dist + self.ca.linea_a_cara_m
        self.fijar_arbol(self.x_linea, y, 0.0)
        return True

    def lidar_lado(self, max_dif=0.06):
        """Corrige el lado del árbol con el LiDAR (más fino que la odometría después de un cangrejo)."""
        if self.lidar is None or self.arbol_odo is None:
            return
        rel = self.arbol_rel()
        m = self.medir_lidar(rel[:2])
        if m is None or m[1] is None:
            self.log("  LiDAR: no veo el pie entero: el lado queda por odometría")
            return
        dif = m[1] - rel[1]
        if abs(dif) > max_dif:
            self.log(f"  LiDAR: el pie {100 * dif:+.1f} cm de lo esperado: demasiado, no le creo")
            return
        self.arbol_odo = componer(self.pose_odo(), np.array([rel[0], m[1], rel[2]]))
        self.log(f"  LiDAR: el árbol estaba {100 * dif:+.1f} cm de lo que decía la odometría")

    # ------------------------------------------------------------------ foto
    def foto(self):
        import cv2
        from cafebot.vision import pisos
        from cafebot.vision.clasificador import CARPETA_CAPTURAS
        if self.cam is None:
            raise RuntimeError("sin cámara")
        servo = self.cc.servo_de_pitch(self.cc.inclinacion_arbol_deg)
        self.mega.comando(f"X 2 {servo}")
        time.sleep(0.8)
        img = self.cam.capturar()
        res = pisos.analizar(img, self.cc.excluir_h, self.cc.sat_min, self.cc.val_min, negros=self.ca.negros)
        anotada = pisos.dibujar(img, res)
        os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
        base = os.path.join(CARPETA_CAPTURAS, time.strftime("%Y%m%d_%H%M%S"))
        cv2.imwrite(base + "_foto.jpg", img)
        cv2.imwrite(base + "_pisos.jpg", anotada)
        self.img = anotada
        self.pisos = res if res.ok else None
        self.log(f"FOTO ({os.path.basename(base)}_pisos.jpg): pisos {list(res.soportes) or '-'}, "
                 f"{len(res.pelotas)} pelotas, paso {res.paso_px:.0f} px, foto torcida "
                 f"{math.degrees(math.atan(res.inclinacion)):+.1f}°; el árbol está "
                 f"{100 * res.lateral_camara_m(img.shape[1]):+.1f} cm a la izquierda del eje de la cámara")
        for m in res.mensajes:
            self.log(f"  {m}")
        self.log(res.tabla())
        if not res.ok:
            self.log("  ¡no le creo a esta foto! (no encontré el piso del medio)")

    # ------------------------------------------------------------------ acercarse
    def acercarse(self):
        """Cuadrarse y centrarse frente al pie (LiDAR), foto a la distancia de la foto y avanzar hasta la línea."""
        cam_y = self.cc.y
        m = self.medir_lidar(self.arbol_rel()[:2] if self.arbol_odo is not None else None)
        if m is not None:
            x_cara, yc, ang, ancho = m
            self.log(f"LiDAR: pie a {100 * (x_cara - self.medio):.1f} cm del frente, "
                     + (f"{100 * yc:+.1f} cm de lado" if yc is not None else f"el lado no (veo {100 * ancho:.0f} cm)")
                     + f", girado {math.degrees(ang):+.1f}°")
            if yc is None:
                yc = self.arbol_rel()[1] if self.arbol_odo is not None else 0.0
            self.fijar_arbol(x_cara, yc, ang)          # al girar, la odometría lo lleva
            self.girar(ang, "cuadrarse con el pie")
            if abs(ang) > math.radians(1.5):
                m2 = self.medir_lidar(self.arbol_rel()[:2])
                if m2 is not None and m2[1] is not None:
                    self.fijar_arbol(m2[0], m2[1], m2[2])
        elif self.arbol_odo is None:
            if self.dist_inicial is None:
                raise RuntimeError("el LiDAR no ve el pie y no sé a qué distancia está (--dist)")
            self.log(f"sin LiDAR: supongo el robot centrado y derecho, con el frente a {100 * self.dist_inicial:.0f} "
                     "cm del pie")
            self.fijar_arbol(self.dist_inicial + self.medio, 0.0, 0.0)
        # foto: la cámara frente al centro del árbol, a la distancia de la foto
        if self.pisos is None and self.cam is not None:
            rel = self.arbol_rel()
            self.cangrejo(rel[1] - cam_y, "centrar la cámara")
            rel = self.arbol_rel()
            self.avanzar(rel[0] - self.medio - self.ca.dist_foto_m, "a la distancia de la foto")
            self.foto()
        # línea
        rel = self.arbol_rel()
        d = rel[0] - self.ca.linea_a_cara_m - self.sx
        if d < 0.025:
            self.avanzar(-(0.045 - d), "atrás antes de la línea")
            d = 0.045
        rel = self.arbol_rel()
        dist = self.tocar_linea(min(d + 0.06, rel[0] - self.medio - FRENTE_MIN_M))
        if dist is None:
            self.log("NO vi la línea del árbol con los IR delanteros: la distancia queda por LiDAR/odometría "
                     "(¿IR calibrados en esta cancha?)")
            self.x_linea = self.arbol_rel()[0] if self.x_linea is None else self.x_linea
            return False
        y = self.arbol_rel()[1]
        self.x_linea = dist + self.ca.linea_a_cara_m
        self.fijar_arbol(self.x_linea, y, 0.0)
        self.log(f"en la línea: la cara del árbol a {100 * (self.x_linea - self.medio):.1f} cm del frente")
        self.lidar_lado()
        return True

    # ------------------------------------------------------------------ el ascensor
    def alinear(self, fila, col):
        """El robot a la distancia de ese piso y con la pelota DELANTE del brazo (cangrejo). Devuelve el punto."""
        if self.arbol_odo is None:
            raise RuntimeError("no sé dónde está el árbol: primero a (acercarse)")
        if self.x_linea is not None:
            self.avanzar(self.arbol_rel()[0] - (self.x_linea + self.aj(fila, "retroceso")), f"distancia del piso {fila}")
        p = self.punto(fila, col)
        dy = p[1] - self.cfg.mision.brazo_y
        if abs(dy) > self.ca.cangrejo_min_m:
            self.log(f"  cangrejo {100 * dy:+.1f} cm hasta {fila}-{col}")
            self.cangrejo(dy, f"cangrejo hasta {fila}-{col}")
            if self.ca.linea_tras_cangrejo and self.x_linea is not None and abs(dy) > 0.03:
                if self.retocar_linea():
                    self.avanzar(self.arbol_rel()[0] - (self.x_linea + self.aj(fila, "retroceso")),
                                 f"distancia del piso {fila}")
            self.lidar_lado()
        return self.punto(fila, col)

    def _plan(self, p):
        plan = self.brazo.plan_agarre(p)
        if plan is None:
            raise RuntimeError(f"el brazo no alcanza ({100 * p[0]:.1f}, {100 * p[1]:+.1f}, {100 * p[2]:.1f}) cm: probar "
                               f"otro retroceso del piso o la altura")
        self.plan, self.p = plan, p
        return plan

    def ir_paso(self, i):
        self.revisar()
        nombre, obj, t = self.plan[i]
        self.brazo.parar.clear()
        if not self.brazo._mover(obj, t, nombre):
            raise RuntimeError(f"brazo ({nombre}): {self.brazo.ultimo_motivo}")
        self.paso = i

    def sin_brazo(self):
        if self.brazo is None:
            raise RuntimeError("sin brazo (--sin-brazo o no conectó)")

    def paso_piso(self):
        p = self.alinear(self.fila, self.col)
        self.log(f"pelota {self.fila}-{self.col} en el robot: x {100 * p[0]:.1f}  y {100 * p[1]:+.1f}  z "
                 f"{100 * p[2]:.1f} cm")
        if self.brazo is None:
            self.log("  (sin brazo: solo me alineé)")
            return
        self._plan(p)
        self.brazo.garra(abrir=True)
        self.ir_paso(0)
        self.log(f"  brazo en el piso {self.fila}, garra abierta, {100 * self.cfg.brazo.aproximacion_m:.0f} cm antes "
                 "de la pelota")

    def paso_entrar(self):
        self.sin_brazo()
        if self.plan is None or self.paso is None:
            raise RuntimeError("primero p (el brazo al piso)")
        self.ir_paso(1)
        self.ir_paso(2)
        self.log("  garra en la pelota")

    def paso_cerrar(self):
        self.sin_brazo()
        self.brazo.parar.clear()
        ang = self.brazo.garra(abrir=False)
        vacia = ang > self.cfg.brazo.garra_vacia_deg
        self.log(f"  garra cerrada en {ang:.1f}° (" + ("VACÍA: cerró del todo" if vacia else "¡con algo adentro!") +
                 f"; vacía si pasa de {self.cfg.brazo.garra_vacia_deg:.0f}°)")

    def paso_sacar(self):
        self.sin_brazo()
        if self.plan is None or len(self.plan) < 5:
            raise RuntimeError("primero p")
        self.ir_paso(3)
        self.ir_paso(4)

    def paso_depositar(self):
        self.sin_brazo()
        p = self.pelota_en(self.fila, self.col)
        cat = p.categoria if p is not None and p.categoria else cancha.MADURO
        destino = "deposito_maduro" if cat == cancha.MADURO else "deposito_sobremaduro"
        self.brazo.parar.clear()
        if not self.brazo.ir_plegado(destino):
            self.log(f"  {self.brazo.ultimo_motivo} (yendo al depósito)")
        self.brazo.garra(abrir=True)
        time.sleep(0.3)
        self.brazo.ir_plegado(self.cfg.brazo.postura_espera)
        self.paso = None
        self.log(f"  depositado como {cat}")

    def agarrar(self, fila, col, cat):
        """La secuencia completa (brazo.recoger: abrir, antes, entrar, cerrar, ¿tomó?, subir, retirar, depósito)."""
        p = self.alinear(fila, col)
        self.log(f"{fila}-{col} ({cat}): x {100 * p[0]:.1f}  y {100 * p[1]:+.1f}  z {100 * p[2]:.1f} cm")
        if self.brazo is None:
            self.log("  (sin brazo: solo me alineé)")
            return False
        if not self.brazo.alcanzable(p):
            self.log("  el brazo no alcanza: probar otro retroceso del piso o la altura")
            return False
        self.revisar()
        self.brazo.parar.clear()
        ok = self.brazo.recoger(p, cat)
        self.paso = None
        if ok:
            self.recogidos[cat] = self.recogidos.get(cat, 0) + 1
            self.log(f"  ¡en el robot! {self.brazo.estado()}")
        else:
            self.log(f"  no salió: {self.brazo.ultimo_motivo}")
        return ok

    def siguiente(self):
        if self.pisos is None:
            return None
        for fila, cat in self.ca.prioridad:
            if fila not in self.ca.pisos:
                continue
            cand = [p for p in self.pisos.pelotas
                    if p.fila == fila and p.categoria == cat and p.clave not in self.hechos]
            if cand:
                p = min(cand, key=lambda q: abs(self.punto(fila, q.col)[1] - self.cfg.mision.brazo_y))
                return fila, p.col, cat
        return None

    def ronda(self, max_granos=0):
        t0 = time.monotonic()
        if self.arbol_odo is None or self.x_linea is None:
            self.acercarse()
        if self.pisos is None:
            raise RuntimeError("sin una foto que le crea no sé qué recoger (f)")
        n = 0
        while True:
            self.revisar()
            obj = self.siguiente()
            if obj is None:
                break
            fila, col, cat = obj
            self.fila, self.col = fila, col
            ok = self.agarrar(fila, col, cat)
            self.hechos.add((fila, col))
            n += ok
            if max_granos and n >= max_granos:
                break
        self.log(f"RONDA: {n} pelota(s) en el robot en {time.monotonic() - t0:.0f} s {self.recogidos}")

    # ------------------------------------------------------------------ ajustes en vivo
    def ajustar(self, que, delta):
        if que == "aprox":
            self.cfg.brazo.aproximacion_m = round(max(0.02, self.cfg.brazo.aproximacion_m + delta), 3)
            self.log(f"aproximación {100 * self.cfg.brazo.aproximacion_m:.0f} cm")
        else:
            self.poner(self.fila, que, self.aj(self.fila, que) + delta)
            self.log(f"{self.fila}: z {100 * self.aj(self.fila, 'z'):.1f}  dx {100 * self.aj(self.fila, 'dx'):+.1f}  "
                     f"dy {100 * self.aj(self.fila, 'dy'):+.1f} cm")
        if self.brazo is not None and self.paso in (0, 2) and self.arbol_odo is not None:
            i = self.paso
            self._plan(self.punto(self.fila, self.col))
            self.ir_paso(i)

    def guardar(self):
        claves = {f"{q}_{f}_m": self.aj(f, q) for f in ("baja", "media", "alta") for q in ("z", "dx", "dy", "retroceso")}
        claves.update(linea_a_cara_m=self.ca.linea_a_cara_m, grano_delante_m=self.ca.grano_delante_m)
        ruta = guardar_local("ascensor", claves)
        guardar_local("brazo", {"aproximacion_m": self.cfg.brazo.aproximacion_m})
        self.log(f"guardado en {ruta} (sección ascensor + brazo.aproximacion_m). Subirlo al repo.")

    def estado(self):
        t = [f"piso {self.fila} col {self.col}"]
        p = self.pelota_en(self.fila, self.col)
        t.append(f"({p.color} -> {p.categoria})" if p is not None else "(sin pelota en la foto)")
        if self.arbol_odo is not None:
            rel = self.arbol_rel()
            t.append(f"| árbol: cara a {100 * (rel[0] - self.medio):.1f} cm del frente, {100 * rel[1]:+.1f} cm de lado, "
                     f"{math.degrees(rel[2]):+.1f}°")
            q = self.punto(self.fila, self.col)
            t.append(f"| pelota x {100 * q[0]:.1f} y {100 * q[1]:+.1f} z {100 * q[2]:.1f} cm")
        else:
            t.append("| árbol: no sé dónde (a)")
        t.append(f"| línea {'sí' if self.x_linea is not None else 'no'} | brazo "
                 f"{['en el piso', 'entrando', 'en la pelota', 'arriba', 'retirado'][self.paso] if self.paso is not None else 'esperando'}")
        self.log(" ".join(t))

    # ------------------------------------------------------------------ hilo de trabajo
    def lanzar(self, nombre, fn, *args):
        if self.hilo is not None and self.hilo.is_alive():
            self.log("(ocupado: ESPACIO para parar)")
            return

        def correr():
            try:
                fn(*args)
            except Cancelado:
                self.log(f"{nombre}: PARADO")
            except Exception as e:
                self.log(f"{nombre}: ERROR {e}")
                if not isinstance(e, RuntimeError):
                    traceback.print_exc()
            finally:
                try:
                    self.estado()
                except Exception:
                    pass
        self.cancelar.clear()
        self.hilo = threading.Thread(target=correr, daemon=True)
        self.hilo.start()

    def parar(self):
        self.cancelar.set()
        try:
            self.mega.parar()
        except Exception:
            pass
        if self.brazo is not None:
            self.brazo.detener()
        self.log("PARAR")

    def ocupado(self):
        return self.hilo is not None and self.hilo.is_alive()


def bucle(asc: Ascensor):
    import termios
    import tty
    from tools.cabezal import leer_tecla
    print(__doc__.split("TECLAS")[1].split('"""')[0])
    asc.estado()
    viejo = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    t_pant = 0.0
    try:
        while True:
            if asc.pantalla is not None and asc.img is not None and time.monotonic() - t_pant > 0.3:
                from tools.cabezal import mostrar
                t_pant = time.monotonic()
                if mostrar(asc.pantalla, asc.img):
                    asc.img = None
            k = leer_tecla(0.1)
            if not k:
                continue
            if k == " ":
                asc.parar()
                continue
            if k in ("q", "Q"):
                if asc.ocupado():
                    asc.parar()
                    asc.hilo.join(5)
                break
            if asc.ocupado():
                print("(ocupado: ESPACIO para parar)")
                continue
            if k in FILAS_TECLA:
                asc.fila = FILAS_TECLA[k]
                asc.col = min(asc.col, len(COL_Y[asc.fila]) - 1)
                asc.estado()
            elif k in ("izq", "der"):
                asc.col = max(0, min(len(COL_Y[asc.fila]) - 1, asc.col + (1 if k == "der" else -1)))
                asc.estado()
            elif k in ("arriba", "abajo"):
                asc.lanzar("ajuste", asc.ajustar, "z", 0.005 if k == "arriba" else -0.005)
            elif k in ("i", "k"):
                asc.lanzar("ajuste", asc.ajustar, "dx", 0.005 if k == "i" else -0.005)
            elif k in ("j", "l"):
                asc.lanzar("ajuste", asc.ajustar, "dy", 0.005 if k == "j" else -0.005)
            elif k in ("+", "-"):
                asc.lanzar("ajuste", asc.ajustar, "aprox", 0.01 if k == "+" else -0.01)
            elif k == "f":
                asc.lanzar("foto", asc.foto)
            elif k == "a":
                asc.lanzar("acercarse", asc.acercarse)
            elif k == "p":
                asc.lanzar("piso", asc.paso_piso)
            elif k == "e":
                asc.lanzar("entrar", asc.paso_entrar)
            elif k == "c":
                asc.lanzar("cerrar", asc.paso_cerrar)
            elif k == "s":
                asc.lanzar("sacar", asc.paso_sacar)
            elif k == "d":
                asc.lanzar("depositar", asc.paso_depositar)
            elif k == "g":
                p = asc.pelota_en(asc.fila, asc.col)
                cat = p.categoria if p is not None and p.categoria else cancha.MADURO
                asc.lanzar("agarre", asc.agarrar, asc.fila, asc.col, cat)
            elif k == "r":
                asc.lanzar("ronda", asc.ronda)
            elif k == "h":
                if asc.brazo is not None:
                    def h():
                        asc.brazo.parar.clear()
                        asc.brazo.garra(abrir=True)
                        asc.brazo.ir_plegado(asc.cfg.brazo.postura_espera)
                        asc.paso = None
                    asc.lanzar("esperar", h)
            elif k == "t":
                asc.guardar()
            elif k == "v":
                asc.estado()
            elif k == "?":
                print(__doc__.split("TECLAS")[1].split('"""')[0])
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None)
    ap.add_argument("--sin-lidar", action="store_true")
    ap.add_argument("--sin-brazo", action="store_true")
    ap.add_argument("--sin-camara", action="store_true")
    ap.add_argument("--dist", type=float, default=None,
                    help="frente del robot -> pie del árbol al partir (m), si no hay LiDAR (robot centrado y derecho)")
    ap.add_argument("--auto", action="store_true", help="la ronda completa sola al apretar el botón")
    ap.add_argument("--boton", choices=("enter", "gpio"), default="enter")
    ap.add_argument("--max-granos", type=int, default=0)
    from tools.pantalla import agregar_opcion
    agregar_opcion(ap)
    a = ap.parse_args()
    cfg = cargar()

    from cafebot.drivers.mega import Mega
    from cafebot.drivers.puertos import resolver
    mega = Mega(resolver(a.mega or cfg.puertos.mega, "mega"))
    print("Conectando con el Mega...")
    mega.conectar()
    mega.telemetria(20)
    if cfg.ir.umbrales is not None or cfg.ir.histeresis is not None:
        mega.ir_aplicar(cfg.ir.umbrales, cfg.ir.histeresis)
    mega.comando(f"X 2 {cfg.camara.servo_pitch_centro}")

    lidar = None
    if not a.sin_lidar:
        try:
            from cafebot.drivers.ld06 import LD06
            lidar = LD06(resolver(cfg.puertos.lidar, "lidar"), cfg.geometria.config_ld06())
            lidar.iniciar()
            if lidar.esperar_scan(3.0) is None:
                print("[LiDAR] no llegan barridos: sigo SIN LiDAR (¿alimentación?)")
                lidar.detener()
                lidar = None
        except Exception as e:
            print(f"[LiDAR] {e}: sigo SIN LiDAR")
            lidar = None

    brazo = depositos = None
    if not a.sin_brazo:
        from tools.mision import armar_reales
        brazo, _, depositos = armar_reales(cfg, True, "nula", mega)
        if not brazo.preparar():
            print(f"[brazo] no se puede usar: {brazo.ultimo_motivo or 'ver arriba'} — sigo SIN brazo")
            brazo = None
        else:
            depositos.preparar()             # compuertas cerradas y con torque (que no se caigan los granos)

    cam = pantalla = None
    if not a.sin_camara:
        try:
            from cafebot.vision.camara import abrir_para_mision
            cam = abrir_para_mision()
            try:
                cam.enfocar(cfg.ascensor.dist_foto_m + cfg.geometria.largo / 2 - cfg.camara.x)
            except Exception:
                pass
        except Exception as e:
            print(f"[cámara] {e}: sigo SIN cámara")
            cam = None
    if cam is not None:
        try:
            from tools.pantalla import Pantalla
            pantalla = Pantalla("Ascensor: pisos y pelotas", (960, 540), a.pantalla)
        except Exception as e:
            print(f"(sin ventana: {e}) — las fotos quedan en pi/capturas/")

    asc = Ascensor(cfg, mega, lidar, brazo, cam, pantalla, a.dist)
    print(f"LiDAR {'sí' if lidar else 'NO'} | brazo {'sí' if brazo else 'NO'} | cámara {'sí' if cam else 'NO'} | "
          f"pisos {cfg.ascensor.pisos} | aproximación {100 * cfg.brazo.aproximacion_m:.0f} cm")
    try:
        if a.auto:
            from cafebot.mision import interfaces as I
            boton = I.BotonGPIO(cfg.mision.pin_boton) if a.boton == "gpio" else I.BotonEnter()
            boton.esperar()
            asc.lanzar("ronda", asc.ronda, a.max_granos)
            try:
                while asc.ocupado():
                    time.sleep(0.2)
            except KeyboardInterrupt:
                asc.parar()
                asc.hilo.join(5)
        else:
            bucle(asc)
    finally:
        try:
            mega.parar()
        except Exception:
            pass
        if lidar is not None:
            lidar.detener()
        if cam is not None:
            cam.cerrar()
        mega.cerrar()
        print(f"Fin. En el robot: {asc.recogidos}. El brazo y las compuertas quedan con torque.")


if __name__ == "__main__":
    main()
