"""Pruebas sin hardware de la visión y de la misión de cosecha.
  A) VisionWorker real sobre una imagen sintética (colores tomados de la calibración real).
  B) MisionCosecha con árbol simulado: orden de recogida, verdes/NO_SE intocables, reintentos,
     sin brazo, aborto y fin de ronda."""
import _stubs  # noqa: F401
import json
import os
import tempfile
import time
import types

import cv2
import numpy as np

import percepcion_granos as P
from brazo_interfaz import BrazoNoIntegrado, BrazoSimulado
from mision_cosecha import MisionCosecha, ConfigMision, Estado

AQUI = os.path.dirname(os.path.abspath(__file__))


def color_bgr_de_referencia(clase):
    """Color BGR que, convertido a Lab, da el promedio calibrado de esa clase."""
    datos = json.load(open(P.C.ARCHIVO_REFERENCIAS))
    labs = np.array([m["lab"] for m in datos["muestras"] if m["clase"] == clase], np.float32)
    lab = labs.mean(axis=0).reshape(1, 1, 3).astype(np.float32)
    bgr = cv2.cvtColor(lab, cv2.COLOR_Lab2BGR)[0, 0]
    return tuple(int(round(v * 255)) for v in np.clip(bgr, 0, 1))


def imagen_arbol(frutos):
    """Fondo café claro con esferas sombreadas. frutos: [(clase, cx, cy)]."""
    img = np.full((864, 1536, 3), (70, 110, 150), np.uint8)               # café (BGR)
    cv2.rectangle(img, (600, 0), (940, 864), (45, 80, 110), -1)           # tronco más oscuro
    for clase, cx, cy in frutos:
        col = color_bgr_de_referencia(clase)
        cv2.circle(img, (cx, cy), 55, col, -1, cv2.LINE_AA)
    return cv2.GaussianBlur(img, (5, 5), 0)


def prueba_percepcion():
    frutos = [("ROJO", 300, 200), ("VERDE", 500, 420), ("AZUL", 1100, 250),
              ("AMARILLO", 1250, 600), ("NARANJO", 250, 650)]
    ruta = os.path.join(tempfile.mkdtemp(), "arbol.jpg")
    cv2.imwrite(ruta, imagen_arbol(frutos), [cv2.IMWRITE_JPEG_QUALITY, 98])
    w = P.VisionWorker(hz=30, imagen=ruta, n_estable=3)
    w.start()
    assert w.snapshot().frame_id == 0 and not w.active
    time.sleep(0.4)
    assert w.snapshot().frame_id == 0, "inactiva no debe procesar"
    w.set_active(True)
    t0 = time.time()
    while time.time() - t0 < 10 and sum(g.estable for g in w.snapshot().granos) < 5:
        time.sleep(0.1)
    snap = w.snapshot()
    w.stop()
    print(f"[A] {len(snap.granos)} granos en {snap.ms:.0f} ms/cuadro (frame {snap.frame_id})")
    for g in snap.granos:
        print("    ", g.resumen(), "->", g.accion)
    por_clase = {g.clase: g for g in snap.granos}
    esperado = {"ROJO": "MADURO", "NARANJO": "MADURO", "AMARILLO": "MADURO",
                "VERDE": "VERDE", "AZUL": "SOBREMADURO"}
    for clase, cat in esperado.items():
        assert clase in por_clase, f"no se detectó {clase}"
        assert por_clase[clase].categoria == cat and por_clase[clase].estable
    assert por_clase["VERDE"].accion == P.ACCION_IGNORAR_VERDE
    assert por_clase["ROJO"].accion == P.ACCION_RECOGER and por_clase["AZUL"].accion == P.ACCION_RECOGER
    assert len(snap.granos) == 5, "no debe ver el café ni el tronco como granos"
    assert snap.imagen is not None and snap.imagen.shape[:2] == (864, 1536)


# ------------------------------------------------------------------ misión
class Ejecutor:
    """RouteExecutor falso: 'avanza' unos cuadros y completa."""
    def __init__(self, resultado="ruta completada", pasos=5):
        self.active, self.status, self.resultado, self.pasos, self._n = False, "inactivo", resultado, pasos, 0

    def start(self):
        self.active, self.status, self._n = True, "seguimiento de ruta iniciado", 0

    def tick(self):
        if self.active:
            self._n += 1
            if self._n >= self.pasos:
                self.active, self.status = False, self.resultado

    def stop(self):
        self.active, self.status = False, "parada solicitada"


class VisionFalsa:
    """Árbol simulado: cada snapshot trae los granos que quedan; ya estables, salvo los 'dudosos'."""
    def __init__(self, granos):
        self.arbol = list(granos)
        self.frame = 0
        self.active, self.manual, self.error, self.status = False, False, "", "falsa"
        self.resets = 0

    def set_active(self, on):
        self.active = on

    def reset_tracks(self):
        self.resets += 1

    def snapshot(self):
        self.frame += 1
        return P.Snapshot(self.frame, time.monotonic(), list(self.arbol), None, 1.0)

    def quitar(self, g):
        self.arbol = [x for x in self.arbol if x is not g]


def grano(tid, clase, cat, cx, cy, estable=True):
    accion = P.accion_para(cat) if estable else P.ACCION_NO_TOCAR
    return P.Grano(tid, clase, cat, cx, cy, 80.0, (cx - 768) * 0.5, (cy - 432) * 0.5, 0.32, 5.0,
                   5 if estable else 2, estable, accion)


class BrazoQueSacaGranos(BrazoSimulado):
    """Brazo simulado que además quita el grano del árbol falso al terminar OK."""
    def __init__(self, vision, **kw):
        super().__init__(**kw)
        self.vision = vision

    def estado(self):
        antes = self._res
        r = super().estado()
        if r == "OK" and antes != "OK":
            self.vision.quitar(self._grano)
        return r


def correr(m, ejecutor, max_s=15):
    t0 = time.time()
    while m.activa and time.time() - t0 < max_s:
        ejecutor.tick()
        m.update()
        time.sleep(0.005)
    return m


CFG = dict(t_estabilizar_s=0.05, t_escaneo_max_s=0.3, t_espera_rescan_s=0.05, cuadros_min_rescan=1)


def armar(arbol, brazo_factory, resultado="ruta completada", **cfg):
    vision = VisionFalsa(arbol)
    ejecutor = Ejecutor(resultado)
    brazo = brazo_factory(vision)
    lm = types.SimpleNamespace(name="ARBOL_1", cls="ARBOL")

    def approach(lm_):
        ejecutor.start()
        return True, "aproximación iniciada"

    m = MisionCosecha(vision, brazo, ejecutor, approach, ConfigMision(**{**CFG, **cfg}))
    ok, msg = m.iniciar(lm)
    assert ok, msg
    return m, vision, ejecutor, brazo


def prueba_mision():
    arbol = [grano(1, "VERDE", "VERDE", 400, 300), grano(2, "ROJO", "MADURO", 700, 400),
             grano(3, "AZUL", "SOBREMADURO", 800, 300), grano(4, "NO_SE", "DESCONOCIDO", 900, 500, estable=False),
             grano(5, "AMARILLO", "MADURO", 600, 500), grano(6, "NARANJO", "MADURO", 1000, 600, estable=False)]
    # A) flujo completo
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoQueSacaGranos(v, duracion_s=0.05))
    assert m.estado is Estado.APROXIMANDO
    correr(m, ej)
    print("[B] flujo completo:", m.estado.value, "|", m.status_text())
    for linea in m.bitacora:
        print("      ", linea)
    assert m.estado is Estado.FIN
    assert m.recogidos == {"MADURO": 2, "SOBREMADURO": 1}, m.recogidos     # el naranjo dudoso NO se toca
    restantes = {g.track_id for g in vision.arbol}
    assert restantes == {1, 4, 6}, "solo deben quedar verde, NO_SE y el dudoso"
    assert m.puntaje_estimado == 19
    assert vision.active is False, "la visión debe apagarse al terminar"

    # B) prioridad: maduros antes que sobremaduros
    orden = []
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoQueSacaGranos(v, duracion_s=0.02))
    original = brazo.iniciar_recogida
    brazo.iniciar_recogida = lambda g: (orden.append(g.categoria), original(g))[1]
    correr(m, ej)
    assert orden[0] == "MADURO" and orden[-1] == "SOBREMADURO", orden

    # C) reintento y salto: el brazo falla siempre con el primer grano
    class BrazoFalla(BrazoQueSacaGranos):
        def estado(self):
            if self._res == "OCUPADO" and time.monotonic() >= self._t_fin:
                self._res = "FALLO"
            return self._res
    solo = [grano(2, "ROJO", "MADURO", 700, 400)]
    m, vision, ej, brazo = armar(solo, lambda v: BrazoFalla(v, duracion_s=0.02))
    correr(m, ej)
    print("[C] brazo que siempre falla:", m.estado.value, "| fallidos =", m.fallidos, "| intentos =", brazo._intentos)
    assert m.estado is Estado.FIN and m.fallidos == 1 and brazo._intentos == 2 and sum(m.recogidos.values()) == 0

    # D) sin brazo: informa y no mueve nada
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoNoIntegrado())
    correr(m, ej)
    print("[D] sin brazo:", m.bitacora[-1])
    assert m.estado is Estado.FIN and "BRAZO NO INTEGRADO" in m.bitacora[-1] and sum(m.recogidos.values()) == 0

    # E) la aproximación falla / se interrumpe
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoNoIntegrado(), resultado="parada solicitada")
    correr(m, ej)
    print("[E] aproximación interrumpida:", m.estado.value, "|", m.bitacora[-1])
    assert m.estado is Estado.ABORTADA and not vision.active

    # F) abortar a mitad de la recogida
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoQueSacaGranos(v, duracion_s=5.0))
    t0 = time.time()
    while m.estado is not Estado.RECOGIENDO and time.time() - t0 < 5:
        ej.tick(); m.update(); time.sleep(0.005)
    assert m.estado is Estado.RECOGIENDO
    m.abortar("ESC")
    assert m.estado is Estado.ABORTADA and brazo.estado() == "LIBRE"

    # G) fin de ronda
    m, vision, ej, brazo = armar(arbol, lambda v: BrazoQueSacaGranos(v, duracion_s=0.02),
                                 deadline_ronda=time.monotonic() + 0.1)
    correr(m, ej)
    assert m.estado is Estado.FIN and "tiempo" in m.bitacora[-1]

    # H) cámara sin imágenes -> aborta (no se queda esperando para siempre)
    class VisionMuda(VisionFalsa):
        def snapshot(self):
            return P.Snapshot(0, 0.0, [], None, 0.0)
    ej = Ejecutor()
    vision = VisionMuda([])
    m = MisionCosecha(vision, BrazoSimulado(), ej, lambda lm: (ej.start(), (True, "ok"))[1],
                      ConfigMision(**{**CFG, "t_escaneo_max_s": 0.05}))
    m.cfg.t_escaneo_max_s = -5.99      # fuerza el umbral de 'sin cuadros' (t + 6 s) a ~0
    m.iniciar(types.SimpleNamespace(name="ARBOL_1", cls="ARBOL"))
    correr(m, ej, 5)
    assert m.estado is Estado.ABORTADA and "sin imágenes" in m.bitacora[-1], (m.estado, list(m.bitacora))

    # I) rechaza landmarks que no son ARBOL
    ej = Ejecutor()
    m = MisionCosecha(VisionFalsa([]), BrazoSimulado(), ej, lambda lm: (True, "ok"), ConfigMision())
    assert m.iniciar(types.SimpleNamespace(name="OBJ_1", cls="OBSTACULO"))[0] is False
    assert m.iniciar(None)[0] is False


if __name__ == "__main__":
    prueba_percepcion()
    prueba_mision()
    print("OK test_vision_mision")
