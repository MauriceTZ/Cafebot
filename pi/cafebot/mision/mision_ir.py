"""Misión SIN LiDAR (rama navegacion-ir, docs/mision_ir.md): IR + odometría (+ IMU) para moverse, la cámara para
las albercas y el árbol. Los granos quedan en el robot (+3 maduro / +1 sobremaduro): no va a las canastas.

Marco de la misión ("local"): x = la x de la cancha (exacta cada vez que los IR delanteros tocan una línea
transversal), y = 0 donde el robot cruzó el borde del beneficiadero (no se sabe en qué sector partió), θ = 0
mirando a los árboles. Es el marco del Mega: se le corrige la pose (P) en cada línea.

1. Partida: el robot mira DIRECTO a un árbol (lo apunta el equipo, desde cualquier punto del sector). Avanza recto
   hasta tocar el borde del beneficiadero con los dos IR delanteros: x y el ángulo con que viene → dónde queda el
   árbol (en la recta que estaba mirando). Gira hasta mirar derecho (θ = 0).
2. Albercas: la cámara mira el piso de la franja (x 1,55–1,75) y busca el azul. Se elige el carril más cercano
   al árbol con margen a cada lado; si no se ve uno seguro, se corre de lado y vuelve a mirar.
3. Cruza recto (tramos cortos, rumbo corregido con la IMU) hasta la posición de la foto, con la cámara centrada en
   el árbol. La foto clasifica los soportes y afina la y del árbol.
4. Toca la línea del cultivo con los IR delanteros (x y rumbo exactos) y se acerca a la distancia de agarre.
5. Recoge por prioridad (config.mision_ir.prioridad: fila media primero, maduros primero), con el brazo alineado
   con una estación lateral para cada soporte; tras cada cangrejo vuelve a tocar la línea.
"""
from __future__ import annotations

import math
import threading
import time
import traceback

import numpy as np

from cafebot.mundo import cancha
from cafebot.navegacion.bordes import Bordes

X_BENEF = cancha.BENEF_X - cancha.LINEA        # 0,731: borde de la línea del beneficiadero del lado de la partida
X_CULTIVO = cancha.CULTIVO_X0                   # 2,550: borde de la línea del cultivo del lado de la zona intermedia
# Para no salir de la cancha sin saber el sector: lejos del perímetro (y) lo que dé el árbol más pegado a un costado
MARGEN_ARBOL_M = 0.45
# con el árbol conocido (mision_ir.arbol), la y absoluta del centro del robot cruzando las albercas: a 25 cm del
# perímetro (medio robot 11,5 + 8 de margen + la línea)
Y_SEGURA = (0.25, cancha.LARGO_Y - 0.25)


class Abortar(RuntimeError):
    pass


def envolver(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def carril(azules, visible, y_obj: float, medio_ancho: float, y_min: float, y_max: float, paso: float = 0.01):
    """Centro de carril (y) más cercano a y_obj tal que [y - medio_ancho, y + medio_ancho] quede DENTRO de lo visible
    y no toque ningún intervalo azul, y que y quede en [y_min, y_max]. None si no hay."""
    lo, hi = max(visible[0] + medio_ancho, y_min), min(visible[1] - medio_ancho, y_max)
    if lo > hi:
        return None
    ys = np.arange(lo, hi + 1e-9, paso)
    libres = [y for y in ys if all(y + medio_ancho < a or y - medio_ancho > b for a, b in azules)]
    if not libres:
        return None
    return float(min(libres, key=lambda y: abs(y - y_obj)))


class MisionIR:
    def __init__(self, cfg, mega, brazo, vision, boton, gyro=None, log=print):
        self.cfg, self.c, self.ci = cfg, cfg.mision, cfg.mision_ir
        self.mega, self.brazo, self.vision, self.boton, self.gyro = mega, brazo, vision, boton, gyro
        self._log = log
        self.eventos: list[str] = []
        self.medio = cfg.geometria.largo / 2
        self.sensor_x = float(cfg.ir.posiciones[0][0])
        self.y_arbol = None
        self.recogidos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0}
        self.lecturas = {}
        self.t0 = None
        self.terminada = False
        self.activa = False
        self._cancelar = threading.Event()
        self._hilo = None
        self._imu_signo = int(self.ci.imu_signo)
        self._corr_y = 0.0                # corrimiento lateral del agarre que funcionó
        self._imu_ref = None              # (yaw de la IMU, θ local) en el último momento conocido
        self.bordes = None

    # ------------------------------------------------------------------ utilidades
    def log(self, t: str):
        s = f"[{self.tiempo():5.1f} s] {t}" if self.t0 else t
        self.eventos.append(s)
        self._log(s)

    def tiempo(self) -> float:
        return time.monotonic() - self.t0 if self.t0 else 0.0

    def queda(self) -> float:
        return self.c.duracion_s - self.tiempo()

    def pose(self):
        x, y, th = self.mega.pose()
        return x / 1000.0, y / 1000.0, th

    def _revisar(self):
        if self._cancelar.is_set():
            raise Abortar("cancelada")
        if self.queda() < 2:
            raise Abortar("se acabó el tiempo")

    def _ok(self, r, que):
        self._revisar()
        if not r.ok:
            raise Abortar(f"{que}: {r.motivo}")

    def _fijar(self, x, y, th):
        self.mega.fijar_pose(1000 * x, 1000 * y, math.degrees(th))
        yaw = self._yaw()
        self._imu_ref = (yaw, th) if yaw is not None else None

    # ------------------------------------------------------------------ rumbo (IMU)
    def _yaw(self):
        """Yaw de la IMU (rad) o None si no hay, no responde o se está calibrando (entonces manda la odometría)."""
        if self.gyro is None:
            return None
        leer = getattr(self.gyro, "leer_yaw", None)
        return leer() if leer is not None else self.gyro.yaw

    def _rumbo_imu(self):
        yaw = self._yaw()
        if yaw is None or self._imu_ref is None or self._imu_signo == 0:
            return None
        return envolver(self._imu_ref[1] + self._imu_signo * (yaw - self._imu_ref[0]))

    def _girar(self, grados: float, que: str):
        """Giro en el lugar. El primero que pasa de 4° le dice a la misión el signo de la IMU (si no lo sabía)."""
        if abs(grados) < 0.3:
            return
        y0 = self._yaw()
        th0 = self.pose()[2]
        self._ok(self.mega.girar(grados, timeout=15), que)
        y1 = self._yaw()
        if y0 is not None and y1 is not None and self._imu_signo == 0 and abs(grados) > 4:
            d_imu, d_odo = y1 - y0, envolver(self.pose()[2] - th0)
            if abs(d_imu) > math.radians(2):
                self._imu_signo = 1 if d_imu * d_odo > 0 else -1
                self.log(f"IMU: signo {self._imu_signo:+d} (giró {math.degrees(d_imu):+.1f}° y la odometría dice "
                         f"{math.degrees(d_odo):+.1f}°)")

    def _corregir_rumbo(self, objetivo: float = 0.0):
        """Con la IMU (si se sabe su signo) se corrige el rumbo del Mega y se gira a `objetivo`."""
        x, y, th = self.pose()
        imu = self._rumbo_imu()
        if imu is not None and abs(envolver(imu - th)) > math.radians(0.3):
            self.mega.fijar_pose(1000 * x, 1000 * y, math.degrees(imu))
            th = imu
        err = envolver(objetivo - th)
        if abs(err) > math.radians(0.8):
            self._girar(math.degrees(err), "corregir el rumbo")

    def _avanzar_hasta_x(self, x_obj: float, vel: float, que: str, tramo: float | None = None):
        tramo = tramo or self.ci.tramo_m
        for _ in range(60):
            self._revisar()
            falta = x_obj - self.pose()[0]
            if abs(falta) < 0.01:
                return
            d = math.copysign(min(abs(falta), tramo), falta)
            self._ok(self.mega.avanzar(1000 * d, vel, timeout=30), que)
            self._corregir_rumbo(0.0)

    def _cangrejo(self, dy: float, que: str):
        if abs(dy) < 0.008:
            return
        self._ok(self.mega.cangrejo(90 if dy > 0 else -90, 1000 * abs(dy), 120, timeout=30), que)
        self._corregir_rumbo(0.0)

    def _linea(self, x_linea: float, max_mm: float, extra_mm: float, que: str):
        """Avanzar hasta que los dos IR delanteros toquen la línea transversal cuyo borde cercano está en x_linea:
        la x y el ángulo del robot salen exactos. Devuelve el ángulo con que venía (rad, + = girado a la izquierda)."""
        self._revisar()
        med = self.bordes.medir_adelante(max_mm, vel=self.ci.vel_linea_mm_s, extra_mm=extra_mm)
        if med is None:
            raise Abortar(f"{que}: no encontré la línea")
        x, y, _ = self.pose()
        self._fijar(x_linea - med.dist_m, y, med.angulo)
        self.log(f"{que}: x = {x_linea - med.dist_m:.3f} m, ángulo {math.degrees(med.angulo):+.1f}°")
        return med.angulo

    # ------------------------------------------------------------------ ciclo
    def iniciar(self):
        self.activa = True
        self._hilo = threading.Thread(target=self._correr, name="mision-ir", daemon=True)
        self._hilo.start()

    def detener(self):
        self._cancelar.set()
        try:
            self.mega.parar()
        except Exception:
            pass
        self.brazo.detener()

    def esperar(self, timeout=None):
        if self._hilo:
            self._hilo.join(timeout)

    def _correr(self):
        try:
            self.boton.esperar()
            self.t0 = time.monotonic()
            self.log("¡PARTIDA! misión sin LiDAR (IR + odometría + cámara)")
            self.bordes = Bordes(self.mega, self.cfg, log=lambda t: None)
            self._fijar(0.0, 0.0, 0.0)
            self._ronda()
            self.log(f"MISIÓN TERMINADA: en el robot {self.recogidos[cancha.MADURO]} maduros y "
                     f"{self.recogidos[cancha.SOBREMADURO]} sobremaduros")
        except Abortar as e:
            self.log(f"misión detenida: {e} | en el robot {self.recogidos}")
        except Exception as e:
            self.log(f"ERROR: {e}")
            traceback.print_exc()
        finally:
            try:
                self.mega.parar()
                self.brazo.descansar()
            except Exception:
                pass
            if self.bordes is not None:
                self.bordes.cerrar()
            self.activa = False
            self.terminada = True

    def _ronda(self):
        if not self.brazo.preparar():
            self.log("el brazo no está disponible: solo navego")
        # 1) recto hasta el borde del beneficiadero: ahí se sabe x, el ángulo de partida y dónde está el árbol
        th0 = self._linea(X_BENEF, 1200, max(self.ci.extra_linea_mm, 150), "borde del beneficiadero")
        x = self.pose()[0]
        self.y_arbol = (cancha.ARBOL_X_FRENTE - x) * math.tan(th0)
        self.log(f"el árbol queda {100 * self.y_arbol:+.1f} cm de lado (la partida lo miraba con {math.degrees(th0):+.1f}°)")
        self._girar(-math.degrees(th0), "enderezarse")
        if self._yaw() is not None and self._imu_signo == 0:      # el signo de la IMU: con un giro de prueba de 6°
            self._girar(6.0, "giro de prueba de la IMU")
            self._girar(-6.0, "giro de prueba de la IMU")
        self._corregir_rumbo(0.0)
        # 2) albercas
        self._avanzar_hasta_x(self.ci.x_vista_albercas, self.ci.vel_mm_s, "hasta mirar las albercas")
        self._elegir_carril()
        # 3) cruzar y foto
        x_foto = cancha.ARBOL_X_FRENTE - self.c.distancia_foto_m - self.medio
        self._avanzar_hasta_x(x_foto, self.ci.vel_mm_s, "cruzar la zona intermedia")
        self._cangrejo(self.y_arbol - self.cfg.camara.y - self.pose()[1], "centrar la cámara en el árbol")
        objetivos = self._foto()
        if not objetivos:
            self.log("no hay granos para recoger en este árbol")
            return
        # 4) línea del cultivo y distancia de agarre. Desde la foto en tramos CORTOS con el rumbo corregido por la
        # IMU: el desvío lateral que no ven los encoders crece con el cuadrado del tramo (es lo que más pesa en la y
        # del agarre), hasta 8 cm antes de la línea, y ahí se busca la línea
        self._avanzar_hasta_x(X_CULTIVO - self.sensor_x - 0.08, self.ci.vel_linea_mm_s, "hacia la línea del cultivo",
                              tramo=self.ci.tramo_fino_m)
        self._acercar(250)
        # 5) recoger
        self._recoger_todos(objetivos)

    # ------------------------------------------------------------------ albercas
    def _elegir_carril(self):
        """Mirar la franja de albercas y quedar en un carril libre (lo más cerca posible del árbol)."""
        medio_ancho = self.cfg.geometria.ancho / 2 + self.ci.margen_alberca_m
        y_min, y_max = self.y_arbol - MARGEN_ARBOL_M, self.y_arbol + MARGEN_ARBOL_M
        intentos = 5
        if 1 <= self.ci.arbol <= 3:        # se sabe cuál es: el límite es el perímetro, no la distancia al árbol
            y_abs = float(cancha.centros_arboles()[self.ci.arbol - 1])
            y_min = self.y_arbol + Y_SEGURA[0] - y_abs
            y_max = self.y_arbol + Y_SEGURA[1] - y_abs
            intentos = 12
        sentido = 0
        for intento in range(intentos):
            self._revisar()
            self.mega.comando(f"X 2 {self.cfg.camara.servo_de_pitch(self.ci.inclinacion_albercas_deg)}")
            time.sleep(0.5)
            vista = self.vision.albercas(self.pose())
            if vista is None:
                raise Abortar("la cámara no pudo mirar las albercas: no cruzo a ciegas")
            azules, visible = vista
            txt = ", ".join(f"{100 * a:+.0f}..{100 * b:+.0f}" for a, b in azules) or "ninguna"
            self.log(f"albercas (cm, en y): {txt} | se ve de {100 * visible[0]:+.0f} a {100 * visible[1]:+.0f}")
            y = carril(azules, visible, self.y_arbol, medio_ancho, y_min, y_max)
            if y is not None:
                self.log(f"carril libre en y = {100 * y:+.1f} cm")
                self._cangrejo(y - self.pose()[1], "al carril libre")
                return
            # nada seguro a la vista: correrse (hacia el árbol primero; si ya se fue para un lado, seguir)
            y0 = self.pose()[1]
            if sentido == 0:
                sentido = 1 if self.y_arbol >= y0 else -1
            destino = y0 + sentido * self.ci.paso_busqueda_m
            if not y_min <= destino <= y_max:
                sentido = -sentido
                destino = y0 + sentido * self.ci.paso_busqueda_m
                if not y_min <= destino <= y_max:
                    break
            self.log(f"no veo un carril seguro: me corro {100 * (destino - y0):+.0f} cm y vuelvo a mirar")
            self._cangrejo(destino - y0, "buscar carril")
        raise Abortar("no encontré un carril libre entre las albercas (sin salir de la zona segura)")

    # ------------------------------------------------------------------ árbol
    def _tablero_robot(self):
        """(x de la cara, y del centro del pie, ángulo) del árbol en el sistema del robot, según lo que se sabe."""
        x, y, th = self.pose()
        dx, dy = cancha.ARBOL_X_FRENTE - x, self.y_arbol - y
        c, s = math.cos(-th), math.sin(-th)
        return float(c * dx - s * dy), float(s * dx + c * dy), float(-th)

    def _foto(self) -> list:
        """Foto del árbol: corrige la y del árbol con lo que ve y devuelve [(fila, col, categoría)] en orden."""
        self.mega.comando(f"X 2 {self.cfg.camara.servo_de_pitch(self.cfg.camara.inclinacion_arbol_deg)}")
        self.vision.preparar_arbol()
        time.sleep(0.4)
        prior = self._tablero_robot()
        tablero, lect = self.vision.arbol(prior)
        if tablero is None:                      # otra foto (un reflejo, alguien pasando delante...)
            self.log("no le creí a la foto del árbol: saco otra")
            time.sleep(0.5)
            tablero, lect = self.vision.arbol(prior)
        if tablero is not None:
            dy = tablero[1] - prior[1]
            if abs(dy) <= self.ci.max_correccion_lateral_m:
                self.y_arbol += dy
                self.log(f"la foto corrige la y del árbol {100 * dy:+.1f} cm")
            else:
                self.log(f"la foto ve el árbol {100 * dy:+.1f} cm corrido: demasiado, no le creo y no recojo "
                         "(con la y errada una columna, se agarra el grano del lado)")
                return []
        self.lecturas = lect
        resumen = {}
        for cat, _ in lect.values():
            resumen[cat] = resumen.get(cat, 0) + 1
        objetivos = []
        for fila, cat in self.ci.prioridad:
            if fila not in self.ci.filas:
                continue
            cols = [col for (f, col), (c, conf) in lect.items()
                    if f == fila and c == cat and conf >= self.ci.confianza_min]
            objetivos += [(fila, col, cat) for col in sorted(cols)]       # de izquierda a derecha
        self.log(f"visión {resumen} → recoger {len(objetivos)}: " +
                 ", ".join(f"{f}-{col + 1} {c[:4]}" for f, col, c in objetivos))
        return objetivos

    def _acercar(self, max_mm: float):
        """Tocar la línea del cultivo (x y rumbo exactos), enderezarse y quedar a la holgura de agarre."""
        ang = self._linea(X_CULTIVO, max_mm, 60.0, "línea del cultivo")
        self._girar(-math.degrees(ang), "enderezarse frente al árbol")
        x, y, _ = self.pose()
        self._fijar(x, y, 0.0)
        frente = cancha.ARBOL_X_FRENTE - x - self.medio
        avance = frente - self.c.holgura_agarre_m
        if abs(avance) > 0.005:
            self._ok(self.mega.avanzar(1000 * avance, 60, timeout=15), "distancia de agarre")

    def _estacion(self, y_sop: float) -> float:
        return min(self.c.estaciones_y, key=lambda e: (abs(e - y_sop), -e))

    @staticmethod
    def _y_soporte(fila: str, col: int) -> float:
        return cancha._OFFSETS[fila][col] + cancha.COPA_DESPLAZADA_M

    def _punto(self, fila: str, y_sop: float, holgura: float | None = None) -> np.ndarray:
        """El grano en el sistema del robot (derecho frente al árbol); con `holgura`, como si el frente del robot
        estuviera a esa distancia del tablero."""
        x, y, _ = self.pose()
        if holgura is not None:
            x = cancha.ARBOL_X_FRENTE - self.medio - holgura
        z = cancha.ALTURA_SOPORTE[fila] + cancha.CENTRO_SOBRE_SOPORTE
        return np.array([cancha.ARBOL_X_FRENTE - x - cancha.GRANO_DELANTE_M, self.y_arbol + y_sop - y, z])

    def _a_holgura(self, holgura: float):
        frente = cancha.ARBOL_X_FRENTE - self.pose()[0] - self.medio
        if abs(frente - holgura) > 0.005:
            self._ok(self.mega.avanzar(1000 * (frente - holgura), 60, timeout=15), "distancia de agarre")

    def _ordenar(self, objetivos: list) -> list:
        """Dentro de cada grupo de prioridad, de una estación a la vecina (menos cangrejo, menos error lateral),
        empezando por la más cercana al brazo."""
        y_brazo = self.pose()[1] + self.c.brazo_y - self.y_arbol
        out = []
        for grupo in self.ci.prioridad:
            resto = [o for o in objetivos if (o[0], o[2]) == tuple(grupo)]
            while resto:
                o = min(resto, key=lambda o: abs(self._estacion(self._y_soporte(o[0], o[1])) - y_brazo))
                resto.remove(o)
                out.append(o)
                y_brazo = self._estacion(self._y_soporte(o[0], o[1]))
        return out

    def _recoger_todos(self, objetivos: list):
        for fila, col, cat in self._ordenar(objetivos):
            if self.queda() < self.ci.reserva_s:
                self.log("se acaba el tiempo: no empiezo otro agarre")
                return
            y_sop = self._y_soporte(fila, col)
            e = self._estacion(y_sop)
            lateral = (self.y_arbol + e - self.c.brazo_y) - self.pose()[1]
            if abs(lateral) > 0.01:
                self._cangrejo(lateral, "cangrejo a la estación")
                if self.ci.linea_por_estacion:
                    self._ok(self.mega.avanzar(-40, 60, timeout=10), "despegarse de la línea")
                    self._acercar(160)
            # la distancia al tablero desde la que el brazo lo alcanza (la fila baja pide alejarse)
            holgura = next((h for h in self.c.holguras_agarre_m
                            if self.brazo.alcanzable(self._punto(fila, y_sop, h))), None)
            if holgura is None:
                self.log(f"{fila}-{col + 1}: el brazo no lo alcanza desde ninguna distancia")
                continue
            self._a_holgura(holgura)
            p = self._punto(fila, y_sop)
            # si falla, se reintenta corrido de lado (en el robot no se sabe hacia dónde se erró); el corrimiento
            # que funcionó se mantiene para los siguientes (un error fijo de la foto o del brazo se aprende)
            d = self.ci.reintento_dy_m
            for intento, dy in enumerate([self._corr_y, self._corr_y + d, self._corr_y - d][:self.ci.intentos_agarre]):
                self._revisar()
                if self.brazo.recoger(p + np.array([0.0, dy, 0.0]), cat):
                    self.recogidos[cat] = self.recogidos.get(cat, 0) + 1
                    if dy != self._corr_y:
                        self.log(f"corrimiento lateral aprendido: {100 * dy:+.1f} cm")
                        self._corr_y = dy
                    self.log(f"{fila}-{col + 1} ({cat.lower()}) en el robot | {self.brazo.estado()}")
                    break
                motivo = getattr(self.brazo, "ultimo_motivo", "")
                self.log(f"{fila}-{col + 1}: no salió ({motivo})")
                if "fuera de alcance" in motivo or "falta la postura" in motivo:
                    break
        self.brazo.descansar()
