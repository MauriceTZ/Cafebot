"""Misión SIN LiDAR (rama navegacion-ir, docs/mision_ir.md): IR + odometría (+ IMU) + cámara.

En el robot (la ronda de verdad: queda esperando el botón):
    python3 -m tools.mision_ir --brazo --imu --boton gpio
    python3 -m tools.mision_ir --brazo --imu              # ENTER en vez del botón (pruebas)
Simulada (sin robot: firmware del Mega + IR + cámara dibujada + brazo simulado con el alcance del de verdad):
    python3 -m tools.mision_ir --sim --semilla 3
    python3 -m tools.mision_ir --sim --zona 2 --albercas 1,2,4
    python3 -m tools.mision_ir --sim --brazo             # ronda simulada con el BRAZO REAL haciendo los agarres
Partida: dentro de su sector, el robot MIRANDO DIRECTO al centro del árbol más cercano (lo apunta el equipo).
"""
from __future__ import annotations

import argparse
import math
import signal
import threading
import time

import numpy as np

from cafebot.config import cargar
from cafebot.mision import interfaces as I
from cafebot.mision.mision_ir import MisionIR
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, relativo


# ====================================================================== mundo simulado
class VerdadSim:
    """La pose REAL del robot simulado: integra lo que dice el Mega con error de calibración (escala) y una deriva
    de rumbo que los encoders no ven (el desvío del robot de verdad: ~6 cm por metro). Vigila que las ruedas no
    pisen el perímetro ni el borde de una alberca y que el robot no la empuje. Da también la IMU simulada."""

    RUEDAS = ((0.1115, 0.1085), (0.1115, -0.1085), (-0.1115, 0.1085), (-0.1115, -0.1085))

    def __init__(self, mega, pose_inicial, albercas, largo=0.30, ancho=0.23, escala_dist=1.03, escala_giro=1.02,
                 deriva_rad_por_m=0.12, imu_signo=1, semilla=None):
        self.mega, self.real = mega, np.asarray(pose_inicial, float)
        self.albercas = [cancha.rect_alberca(a) for a in albercas]
        self.medio = (largo / 2, ancho / 2)
        self.esc_d, self.esc_g, self.deriva = escala_dist, escala_giro, deriva_rad_por_m
        self.imu_signo = imu_signo
        self.rng = np.random.default_rng(semilla)
        self._prev = None
        self.pausa = False                           # durante un P (fijar_pose) no se integra nada
        self._th0 = float(self.real[2])
        self.faltas: list[str] = []
        self._lock = threading.Lock()
        self._corriendo = True
        threading.Thread(target=self._bucle, name="verdad-sim", daemon=True).start()

    def _pose_mega(self):
        return np.array([self.mega.tele.x / 1000, self.mega.tele.y / 1000, self.mega.tele.th])

    def _avanzar(self, pm):
        if self._prev is None or self.pausa:
            return self.real.copy()
        d = relativo(self._prev, pm)
        if math.hypot(d[0], d[1]) >= 0.25:          # P (fijar pose) en el Mega: no es movimiento
            return self.real.copy()
        dx, dy = d[0] / self.esc_d, d[1] / self.esc_d
        dth = d[2] / self.esc_g + self.deriva * abs(dx)
        return componer(self.real, np.array([dx, dy, dth]))

    def pose_real_ahora(self, pose_mega=None):
        with self._lock:
            return self._avanzar(np.asarray(pose_mega, float) if pose_mega is not None else self._pose_mega())

    def _bucle(self):
        while self._corriendo:
            pm = self._pose_mega()
            with self._lock:
                if not self.pausa:
                    self.real = self._avanzar(pm)
                    self._prev = pm
            self._vigilar()
            time.sleep(0.01)

    def _vigilar(self):
        x, y, th = self.real
        c, s = math.cos(th), math.sin(th)
        L = cancha.LINEA
        for px, py in self.RUEDAS:
            wx, wy = x + c * px - s * py, y + s * px + c * py
            if wx < L or wx > cancha.LARGO_X - L or wy < L or wy > cancha.LARGO_Y - L:
                self._falta("una rueda pisó el PERÍMETRO")
            for r in self.albercas:
                if r.contiene(np.array(wx), np.array(wy), margen=L):
                    self._falta("una rueda pisó el borde de una ALBERCA")
        hx, hy = self.medio
        for r in self.albercas:
            for fx in (-hx, 0.0, hx):
                for fy in (-hy, 0.0, hy):
                    if r.contiene(np.array(x + c * fx - s * fy), np.array(y + s * fx + c * fy)):
                        self._falta("el robot CHOCÓ una alberca")

    def _falta(self, t):
        if not self.faltas or self.faltas[-1] != t:
            self.faltas.append(t)

    @property
    def yaw(self):                                   # IMU simulada (Giroscopio.yaw)
        return self.imu_signo * (float(self.real[2]) - self._th0) + float(self.rng.normal(0, 0.002))

    def detener(self):
        self._corriendo = False


class CamaraRender:
    """La cámara del robot simulado: dibuja la cancha (vision/render.py) desde la pose REAL, con la inclinación del
    último X 2 que aceptó el Mega. Misma interfaz que la cámara de verdad (capturar / enfocar / cerrar)."""

    def __init__(self, cfg, mundo, pose_real_fn, mega, error_pitch_deg=0.0):
        from cafebot.vision.camara_modelo import CamaraPinhole
        self.cc, self.mundo, self.pose_real_fn, self.mega = cfg.camara, mundo, pose_real_fn, mega
        self._Cam = CamaraPinhole
        self.error_pitch = error_pitch_deg

    def capturar(self):
        from cafebot.vision.render import dibujar
        servo = self.mega.aux.get(2, self.cc.servo_pitch_centro)
        cam = self._Cam.desde_config(self.cc, self.cc.pitch_de_servo(servo) + self.error_pitch)
        return dibujar(self.mundo, tuple(self.pose_real_fn()), cam.con(dist=np.zeros(5)))

    def enfocar(self, d):
        pass

    def cerrar(self):
        pass


class BrazoSimIR(I.BrazoSimulado):
    """El brazo simulado con la tolerancia REAL de la garra (abre 65 mm, pelota de 42: ±11 mm por eje) en vez de la
    esfera de 25 mm del simulador general; anota el error de cada agarre."""

    def __init__(self, *a, tol_m=0.011, **k):
        super().__init__(*a, **k)
        self.tol, self.errores = tol_m, []

    def recoger(self, p_robot, categoria):
        time.sleep(self.t)
        if not self.alcanzable(p_robot):
            self.ultimo_motivo = "fuera de alcance"
            return False
        sop, error = self._soporte_real(p_robot)
        x, y, th = self.pose_robot_fn()
        c, s = math.cos(th), math.sin(th)
        p = np.asarray(p_robot, float)
        pw = np.array([x + c * p[0] - s * p[1], y + s * p[0] + c * p[1], p[2]])
        e = pw - np.array([sop.x, sop.y, sop.z])
        self.errores.append(e)
        if np.max(np.abs(e)) > self.tol or sop.clave not in self.esc.granos:
            self.ultimo_motivo = ("la garra cerró en el aire (error x/y/z " +
                                 "/".join(f"{1000 * v:+.0f}" for v in e) + " mm)")
            return False
        cat, _ = self.esc.granos.pop(sop.clave)
        self.depositos[categoria if categoria in self.depositos else cat].append(cat)
        return True


def albercas_sorteo(rng, n_max=3):
    n = int(rng.integers(0, n_max + 1))
    return tuple(sorted(int(a) for a in rng.choice(np.arange(1, 6), n, replace=False)))


MAX_PUNTERIA_DEG = 18.0     # docs/mision_ir.md: con más, los IR delanteros pisan las divisiones del beneficiadero
CORRIMIENTO_MAX_M = 0.20    # cuánto se puede correr el robot de lado dentro de su sector (72 cm, el robot 23)


def elegir_arbol(alb, zona, x0=0.25, medio_ancho=0.195):
    """Lo que hace el operador (ve las albercas antes de la ronda): entre los árboles que se pueden mirar desde el
    sector con <= MAX_PUNTERIA_DEG (corriéndose de lado dentro de él), el del paso libre más derecho (el carril
    entre albercas más cercano a su centro en y; hasta 15 cm de lado da igual) y, entre esos, el más cercano.
    Devuelve (árbol 1..3, y de la partida)."""
    ys = cancha.centros_arboles()
    yc = cancha.centros_zonas_beneficiadero()[zona - 1]
    centros = np.arange(0.20, cancha.LARGO_Y - 0.20 + 1e-9, 0.01)
    libres = [y for y in centros if all(y + medio_ancho < (a - 1) * 0.72 or y - medio_ancho > a * 0.72 for a in alb)]
    cand = []
    for i in range(3):
        # corrido hacia el árbol, pero que la rueda delantera de ese lado llegue al borde del beneficiadero sin pisar
        # la división del sector (si la pisa, el IR la toma por el borde)
        d = ys[i] - yc
        s = 0.0
        for s in np.arange(min(abs(d), CORRIMIENTO_MAX_M), -1e-9, -0.01):
            lat = ((cancha.BENEF_X - cancha.LINEA) - x0) * abs(d - math.copysign(s, d)) / (cancha.ARBOL_X_FRENTE - x0)
            if s + lat + 0.115 + 0.03 <= cancha.SECTOR_BENEF_Y / 2:
                break
        y0 = yc + math.copysign(float(s), d)
        if math.degrees(abs(math.atan2(ys[i] - y0, cancha.ARBOL_X_FRENTE - x0))) <= MAX_PUNTERIA_DEG:
            paso = min((abs(y - ys[i]) for y in libres), default=9.0)
            cand.append(((round(max(0.0, paso - 0.15), 1), abs(ys[i] - yc)), i + 1, y0))
    return min(cand)[1:]


def armar_sim(cfg, semilla, zona=None, albercas=None, t_agarre=4.0, error_punteria_deg=1.5, log=print):
    """Mega simulado + IR + cámara dibujada + brazo simulado. La partida: en su sector, mirando al árbol que
    elegiría el operador (elegir_arbol)
    (con un error de puntería)."""
    from cafebot.arm.brazo import BrazoXL430
    from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
    from cafebot.drivers.mega import Mega
    from cafebot.vision import clasificador as cl
    from cafebot.vision.render import MundoCancha
    from cafebot.vision.vision_ir import VisionIRCamara
    from tests.sim_serial import SerialSimulado, compilar
    rng = np.random.default_rng(semilla)
    zona = zona or int(rng.integers(1, 6))
    alb = albercas if albercas is not None else albercas_sorteo(rng)
    esc = cancha.Escenario(zona, 1, 2, alb, cancha.sorteo_granos(rng))
    ys = cancha.centros_arboles()
    x0 = 0.25 + rng.uniform(-0.03, 0.03)
    arbol, y0 = elegir_arbol(alb, zona, x0)
    y0 += rng.uniform(-0.04, 0.04)
    th0 = math.atan2(ys[arbol - 1] - y0, cancha.ARBOL_X_FRENTE - x0) + math.radians(rng.normal(0, error_punteria_deg))
    compilar()
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(50)
    # escala 1 %: las ruedas se calibraron (52,6 mm, 28 oct); la deriva de rumbo: el desvío de ~6 cm por metro
    verdad = VerdadSim(mega, (x0, y0, th0), alb, escala_dist=1.01, escala_giro=1.01,
                       imu_signo=int(rng.choice([-1, 1])), semilla=semilla)
    fijar = mega.fijar_pose

    def fijar_pose(*args):                          # P mueve la pose del Mega de golpe: no es movimiento del robot
        verdad.pausa = True
        try:
            r = fijar(*args)
            time.sleep(0.12)                         # que llegue la telemetría con la pose nueva
        finally:
            with verdad._lock:
                verdad._prev = None
            verdad.pausa = False
        return r
    mega.fijar_pose = fijar_pose
    mega.ir_sim = IRSimulado(mega, cfg.ir.posiciones, cancha.lineas_cancha(),
                             lambda: verdad.pose_real_ahora(pose_mega_extrapolada(mega)), semilla=semilla).iniciar()
    mundo = MundoCancha(esc)
    cam = CamaraRender(cfg, mundo, lambda: verdad.pose_real_ahora(), mega)
    cfg.camara.fotos, cfg.camara.guardar_fotos = 1, False
    vision = VisionIRCamara(cfg, camara=cam, clasificador=cl.Clasificador(), log=log)    # colores por matiz
    c = cfg.mision
    brazo = BrazoSimIR(esc, lambda: tuple(verdad.pose_real_ahora()), (c.brazo_x, c.brazo_y, c.brazo_z),
                       t_agarre=t_agarre, p_falla=0.0, rng=rng, alcanzable_fn=BrazoXL430(cfg, log=lambda t: None).alcanzable)
    log(f"Cancha: partida en el sector {zona} (x {x0:.2f}, y {y0:.2f}) mirando al árbol {arbol} "
        f"({math.degrees(th0):+.1f}°), albercas {alb or 'ninguna'}")
    return esc, mega, verdad, vision, brazo, arbol


def puntaje(depositos) -> int:
    """Granos que quedaron en el robot: +3 maduro, +1 sobremaduro, −5 un verde (no debería pasar)."""
    return sum({cancha.MADURO: 3, cancha.SOBREMADURO: 1, cancha.VERDE: -5}.get(g, 0)
               for lista in depositos.values() for g in lista)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--semilla", type=int, default=None)
    ap.add_argument("--zona", type=int, default=None, help="(sim) sector de partida 1..5")
    ap.add_argument("--albercas", default=None, help="(sim) posiciones, p. ej. 1,3,4 (vacío = ninguna)")
    ap.add_argument("--t-agarre", type=float, default=4.0, help="(sim) segundos por agarre")
    ap.add_argument("--brazo", action="store_true", help="el brazo real (con --sim: hace los agarres en el aire)")
    ap.add_argument("--arbol", type=int, default=None, choices=(0, 1, 2, 3),
                    help="árbol al que apunta la partida (1 = el de y chica); permite rodear albercas más lejos")
    ap.add_argument("--imu", action="store_true", help="usar la IMU para el rumbo (en --sim siempre)")
    ap.add_argument("--boton", choices=("enter", "gpio", "ya"), default=None)
    a = ap.parse_args()
    cfg = cargar()
    if a.arbol is not None:
        cfg.mision_ir.arbol = a.arbol
    gyro = None
    verdad = None
    if a.sim:
        alb = None if a.albercas is None else tuple(int(v) for v in a.albercas.split(",") if v.strip())
        esc, mega, verdad, vision, brazo, arbol = armar_sim(cfg, a.semilla, a.zona, alb, a.t_agarre)
        if a.arbol is None:
            print(f"(sim) sin --arbol: la misión no sabe que es el árbol {arbol}")
        gyro = verdad
        if a.brazo:
            from tools.mision import brazo_hibrido
            brazo = brazo_hibrido(cfg, brazo)
    else:
        from cafebot.drivers.mega import Mega
        from cafebot.vision.vision_ir import VisionIRCamara
        mega = Mega(cfg.puertos.mega)
        print("Conectando con el Mega...")
        mega.conectar()
        mega.telemetria(20)
        if cfg.ir.umbrales is not None or cfg.ir.histeresis is not None:
            mega.ir_aplicar(cfg.ir.umbrales, cfg.ir.histeresis)
        print(f"Cámara al frente: X 2 {cfg.camara.servo_pitch_centro} -> {mega.comando(f'X 2 {cfg.camara.servo_pitch_centro}')}")
        if a.imu:
            from cafebot.drivers.imu import Giroscopio
            try:
                gyro = Giroscopio(cfg.puertos.imu_bus, cfg.puertos.imu_dir)
                print(f"IMU: {gyro.modelo} (calibrando el sesgo: NO mover el robot)")
            except Exception as e:
                print(f"IMU no disponible ({e}): el rumbo queda solo con la odometría y las líneas")
        vision = VisionIRCamara(cfg)
        if a.brazo:
            from tools.mision import armar_reales
            brazo = armar_reales(cfg, True, "nula", mega)[0]
        else:
            brazo = I.BrazoNulo()
    vision.iniciar()
    boton = {"enter": I.BotonEnter, "ya": I.Boton}.get(a.boton or ("ya" if a.sim else "enter"))
    boton = I.BotonGPIO(cfg.mision.pin_boton) if a.boton == "gpio" else boton()
    if gyro is not None and hasattr(gyro, "listo"):
        fin = time.monotonic() + 5
        while not gyro.listo and time.monotonic() < fin:       # el sesgo se calibra con el robot quieto
            time.sleep(0.1)
    m = MisionIR(cfg, mega, brazo, vision, boton, gyro=gyro)
    signal.signal(signal.SIGINT, lambda *_: m.detener())
    m.iniciar()
    while not m.terminada:
        time.sleep(0.2)
    if a.sim:
        print(f"PUNTAJE (granos en el robot): {puntaje(brazo.sim.depositos if a.brazo else brazo.depositos)} | "
              f"{m.tiempo():.0f} s | faltas: {verdad.faltas or 'ninguna'}")
        errs = getattr(brazo, "errores", [])
        if errs:
            e = 1000 * np.abs(np.array(errs))
            print(f"error de los agarres (mm, máx por eje x/y/z): {e.max(0).round(1)} | media {e.mean(0).round(1)}")
        verdad.detener()
        mega.ir_sim.detener()
    mega.cerrar()
    try:
        vision.cam.cerrar()
    except Exception:
        pass
    print("FIN: la misión terminó y el programa se cierra.")


if __name__ == "__main__":
    main()
