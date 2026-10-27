"""Prueba de humo del programa COMPLETO en lazo cerrado y sin hardware:
LD06 simulado (sala + árbol + chasis propio) + Mega simulado (cinemática diferencial) + cámara
con foto fija + brazo simulado. Se inyectan las teclas [TAB] y [K] y se verifica que el robot
(1) no se detecta a sí mismo, (2) reconoce el árbol, (3) llega solo, (4) ve y clasifica los granos
y (5) recoge. Usa pygame/pyserial falsos si no están instalados (en el PC de desarrollo)."""
import _stubs  # noqa: F401
import math
import os
import sys
import tempfile
import time
import types

import cv2
import numpy as np

import ld06_slam_nav_rpi_imu_integrado as M
from sim_lidar import ray_scan
from test_vision_mision import imagen_arbol
import brazo_interfaz

ARBOL = ((0.0, 1.5), 0.08)
CHASIS_REAL = (0.12, 0.16)


class Mundo:
    pose = np.array([0.0, 0.0, 0.0])        # verdad: x, y, theta (theta=0 mira a +Y)
    v = 0.0
    w = 0.0
    postura = 0
    t_cmd = 0.0
    ticks = np.zeros(4)                      # acumulados M1..M4


class FakeMega:
    ENC = 800.0 / (math.pi * 0.053)          # ticks por metro

    def __init__(self, *a, **k):
        self.error = ""
        self.last = time.monotonic()
        self.seq = 0

    def send(self, cmd):
        p = cmd.split()
        if p[0] == "V":
            Mundo.v, Mundo.w = float(p[1]) / 1000.0, float(p[2])
            Mundo.postura = 1 if (abs(Mundo.v) < 1e-9 and abs(Mundo.w) > 1e-9) else 0
            Mundo.t_cmd = time.monotonic()
        elif p[0] in ("S", "C"):
            Mundo.v = Mundo.w = 0.0
            Mundo.postura = 0
        return True

    def poll(self, hz=25.0):
        now = time.monotonic()
        dt = min(now - self.last, 0.1)
        self.last = now
        if now - Mundo.t_cmd > 0.5:          # watchdog del firmware
            Mundo.v = Mundo.w = 0.0
        th = Mundo.pose[2]
        Mundo.pose += np.array([-Mundo.v * dt * math.sin(th), Mundo.v * dt * math.cos(th), Mundo.w * dt])
        if Mundo.postura:
            d = 0.5 * Mundo.w * dt * math.hypot(0.217, 0.223) * self.ENC
            Mundo.ticks += np.array([d, -d, d, -d])
        else:
            Mundo.ticks += Mundo.v * dt * self.ENC

    def get_state(self):
        self.seq += 1
        t = tuple(int(round(x)) for x in Mundo.ticks)
        return t + (Mundo.postura,), self.seq, []

    def close(self):
        pass


class FakeScanner:
    def __init__(self, port=None, baudrate=None, footprint=None):
        self.fp = footprint
        self.id = 0
        self.last = 0.0
        self.scan = None
        self.rng = np.random.default_rng(3)
        self.filtrados = 0
        self.en_huella_total = 0

    def get_latest_scan_t(self):
        now = time.monotonic()
        if now - self.last >= 0.1:
            self.last = now
            ang, dist = ray_scan(tuple(Mundo.pose), arboles=(ARBOL,), chasis=CHASIS_REAL, rng=self.rng)
            a = np.radians(ang)
            d = dist / 1000.0
            ok = (d >= 0.15) & (d <= 12.0)
            x, y = np.sin(a) * d, np.cos(a) * d
            if self.fp is not None:
                propio = np.array([self.fp.contains(px, py) for px, py in zip(x, y)])
                self.filtrados += int((ok & propio).sum())
                ok &= ~propio
            else:
                self.en_huella_total += 1
            self.scan = np.column_stack((a[ok], d[ok], np.full(ok.sum(), 200.0)))
            self.id += 1
        return self.id, (self.scan.copy() if self.scan is not None else None), self.last

    def status_text(self):
        return "LIDAR SIMULADO"

    def close(self):
        pass


def correr(con_filtro=True, segundos=150):
    Mundo.pose = np.array([0.0, 0.0, 0.0])
    Mundo.v = Mundo.w = 0.0
    Mundo.ticks = np.zeros(4)
    Mundo.t_cmd = time.monotonic()
    foto = os.path.join(tempfile.mkdtemp(), "arbol.jpg")
    cv2.imwrite(foto, imagen_arbol([("ROJO", 400, 300), ("VERDE", 800, 400), ("AZUL", 1100, 300)]),
                [cv2.IMWRITE_JPEG_QUALITY, 98])

    espia = types.SimpleNamespace(mision=None, slam=None, estados=[], scanner=None, t0=time.time(), fase=0)

    class SlamEspia(M.LidarSLAM2D):
        def __init__(s, *a, **k):
            super().__init__(*a, **k)
            espia.slam = s

    class MisionEspia(M.MisionCosecha):
        def __init__(s, *a, **k):
            super().__init__(*a, **k)
            espia.mision = s

        def iniciar(s, lm):
            r = super().iniciar(lm)
            print(f"   [K] iniciar({getattr(lm, 'name', None)}) -> {r}")
            return r

        def update(s):
            super().update()
            if not espia.estados or espia.estados[-1] != s.estado:
                espia.estados.append(s.estado)
                print(f"   t={time.time() - espia.t0:5.1f}s  estado -> {s.estado.value}  | pose real="
                      f"({Mundo.pose[0]:+.2f},{Mundo.pose[1]:+.2f},{math.degrees(Mundo.pose[2]):+.0f} grados)")

    class ScannerEspia(FakeScanner):
        def __init__(s, *a, **k):
            super().__init__(*a, **k)
            espia.scanner = s

    class EjecutorRapido(M.RouteExecutor):
        def __init__(s, *a, **k):
            super().__init__(*a, **k)
            s.max_v, s.max_w = 250.0, 1.0       # solo para acelerar la prueba

    M.CIRCLE_RMS_MIN = 0.03      # tolerancia de ajuste de círculo ajustada al ruido del SLAM simulado (ver comentario en el script)
    M.LD06Scanner, M.MegaSerialLink = ScannerEspia, FakeMega
    M.LidarSLAM2D, M.MisionCosecha, M.RouteExecutor = SlamEspia, MisionEspia, EjecutorRapido
    M.BrazoSimulado = lambda: brazo_interfaz.BrazoSimulado(duracion_s=0.3)

    pg = M.pg
    K = lambda nombre: types.SimpleNamespace(type=pg.KEYDOWN, key=getattr(pg, nombre))

    def eventos():
        t = time.time() - espia.t0
        m = espia.mision
        # Calentamiento: el operador empuja un poco el robot para que el mapa se rellene (adelante, atrás
        # y un giro), como se hace hoy antes de planificar. Esto sustituye a la futura ronda de exploración.
        if t < 9.0 and espia.fase == 0:
            Mundo.t_cmd = time.monotonic()
            if t < 3.0:
                Mundo.v, Mundo.w, Mundo.postura = 0.12, 0.0, 0
            elif t < 6.0:
                Mundo.v, Mundo.w, Mundo.postura = -0.12, 0.0, 0
            elif t < 8.5:
                Mundo.v, Mundo.w, Mundo.postura = 0.0, 0.3, 1
            else:
                Mundo.v, Mundo.w, Mundo.postura = 0.0, 0.0, 0
            return []
        if espia.fase == 0 and t > 12:
            espia.fase = 1
            return [K("K_TAB")]
        if espia.fase == 1 and t > 13:
            espia.fase = 2
            return [K("K_k")]
        if espia.fase == 2 and m is not None and (sum(m.recogidos.values()) >= 1 or not m.activa):
            espia.fase = 3
            return [K("K_ESCAPE")]
        if espia.fase == 3 or t > segundos:
            return [types.SimpleNamespace(type=pg.QUIT)]
        return []

    pg.event.get = eventos
    argv = ["x", "--port", "fake", "--mega-port", "fake", "--no-imu", "--vision", "--vision-imagen", foto,
            "--vision-hz", "20", "--vision-n", "3", "--brazo-simulado"]
    if not con_filtro:
        argv.append("--no-self-filter")
    sys.argv = argv
    M.main()
    return espia


def main():
    print("== CON filtro de autodetección ==")
    e = correr(True)
    m, slam, sc = e.mision, e.slam, e.scanner
    nombres = [s.name for s in e.estados]
    print("   estados:", " -> ".join(nombres))
    print("   bitácora:")
    for l in m.bitacora:
        print("     ", l)
    print(f"   puntos propios descartados por el filtro: {sc.filtrados}")
    ys, xs = np.nonzero(slam.static_mask())
    wx, wy = (xs - slam.center_cell + 0.5) * slam.res, (ys - slam.center_cell + 0.5) * slam.res
    cerca_inicio = int(np.sum(np.hypot(wx, wy) < 0.19))
    print(f"   celdas estáticas del mapa a <19 cm del punto de partida (deben ser 0): {cerca_inicio}")
    dist_arbol = math.hypot(Mundo.pose[0] - ARBOL[0][0], Mundo.pose[1] - ARBOL[0][1])
    print(f"   distancia final real al centro del árbol: {dist_arbol:.2f} m")
    assert {"APROXIMANDO", "ESTABILIZANDO", "ESCANEANDO", "RECOGIENDO"} <= set(nombres), nombres
    assert sum(m.recogidos.values()) >= 1
    assert cerca_inicio == 0 and sc.filtrados > 0
    assert 0.35 < dist_arbol < 0.9
    print("OK test_smoke_main")


if __name__ == "__main__":
    main()
