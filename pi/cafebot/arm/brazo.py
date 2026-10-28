"""Brazo REAL (5 XL430 + garra, por U2D2) con la interfaz que usa la misión (`mision.interfaces.Brazo`).

Coordenadas: la misión pide los granos en el sistema del ROBOT (x adelante, y izquierda, z desde el SUELO).
Aquí se pasan al plano del brazo:
    yaw = dirección horizontal desde el eje de la base hasta el grano
    r   = distancia horizontal desde ese eje           z_brazo = z - altura de la base del brazo
y se resuelven con la IK de mínimo esfuerzo (`cinematica.py`).

Posturas con nombre (ángulos articulares, se ENSEÑAN moviendo el brazo a mano sin torque):
    trabajo               plegado delante, listo para agarrar (por defecto: la de encendido del código antiguo)
    descanso              plegado para navegar (por defecto: la de apagado del código antiguo)
    deposito_maduro       sobre la tolva de los maduros          ┐ hay que grabarlas:
    deposito_sobremaduro  sobre la tolva de los sobremaduros     ┘ python3 -m tools.brazo grabar deposito_maduro

Secuencia de agarre (`recoger`):
    abrir → 6 cm antes del grano (en la dirección del brazo) → entrar en 2 pasos → cerrar
    → ¿la garra quedó abierta por el grano? (si cerró del todo: no había nada) → subir 3 cm → retroceder 8 cm
    → depósito de su categoría → abrir → postura de espera (descanso: plegado detrás del frente)
En cada paso se revisa que las articulaciones lleguen (error < error_max_deg) y que la carga no se dispare:
si algo choca, se abre la garra, se retrocede y se informa el fallo (la misión sigue con el siguiente).
"""
from __future__ import annotations

import json
import math
import pathlib
import time

import numpy as np

from cafebot.arm import dxl
from cafebot.arm.cinematica import ModeloBrazo, directa, inversa, torques_gravedad
from cafebot.mision.interfaces import Brazo
from cafebot.mundo import cancha

IDS = [9, 8, 11, 7, 5]          # base (yaw), hombro, codo1, codo2, muñeca
ID_GARRA = 4
NOMBRES = {9: "base", 8: "hombro", 11: "codo1", 7: "codo2", 5: "muñeca", 4: "garra"}
RAIZ_PI = pathlib.Path(__file__).resolve().parents[2]


def _deg_desde_pasos(tabla: dict) -> dict:
    """Posturas del código antiguo (pasos de encoder) -> grados respecto del cero ortogonal."""
    out = {}
    for c in dxl.BRAZO:
        dif = ((tabla[c.id] - c.centro_pasos + 2048) % 4096) - 2048
        out[str(c.id)] = round((-1 if c.invertir else 1) * dif * 360.0 / 4096, 2)
    return out


# legacy/codigo_brazo/main.py: OFFSETS_PRENDERSE y OFFSETS_APAGARSE
POSTURAS_DEFECTO = {
    "trabajo": _deg_desde_pasos({9: 3074, 8: 1400, 11: 2423, 7: 194, 5: 1587}),
    "descanso": _deg_desde_pasos({9: 3085, 8: 1787, 11: 1719, 7: 78, 5: 1801}),
}


class BrazoXL430(Brazo):
    def __init__(self, cfg, puerto: str | None = None, bus=None, log=print):
        self.cfg = cfg
        self.cb = cfg.brazo
        self.cm = cfg.mision
        self.puerto = puerto or cfg.puertos.u2d2
        self.bus = bus
        self.log = log
        self.modelo = ModeloBrazo(L4=self.cb.garra_largo_m)
        self.base = np.array([self.cm.brazo_x, self.cm.brazo_y, self.cm.brazo_z])
        self.ruta_posturas = RAIZ_PI / self.cb.archivo_posturas
        self.posturas = self.cargar_posturas()
        self.depositos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0, cancha.VERDE: 0}
        self.listo = False
        self.ultimo_motivo = ""
        self._salud = []           # última lectura (estado() NO toca el bus: lo llama la interfaz desde otro hilo)

    # ------------------------------------------------------------------ conexión
    def conectar(self) -> dict:
        """Abre el bus, verifica que estén los 6 servos y escribe la configuración cuidadosa."""
        if self.bus is None:
            self.bus = dxl.BusDXL(self.puerto)
        vivos = self.bus.ping()
        faltan = [s for s in IDS + [ID_GARRA] if s not in vivos]
        if faltan:
            raise IOError("no responden los servos " + ", ".join(f"{s} ({NOMBRES[s]})" for s in faltan))
        apagados = self.bus.configurar()    # configuración + ceros normalizados
        if apagados:
            self.log(f"[brazo] se escribió la configuración (torque apagado en {apagados})")
        self.bus.recuperar_errores()
        return vivos

    def preparar(self) -> bool:
        """Para la misión: conectar, revisar salud y quedar en la postura de espera. False si no se puede usar."""
        try:
            if self.bus is None:
                self.conectar()
            ok, motivo = self.bus.puede_trabajar()
            if not ok:
                self.bus.recuperar_errores()
                ok, motivo = self.bus.puede_trabajar()
            if not ok:
                self.log(f"[brazo] no se puede usar: {motivo}")
                return False
            for nombre in ("deposito_maduro",) + (("deposito_sobremaduro",) if cancha.SOBREMADURO in self.cm.recoger else ()):
                if nombre not in self.posturas:
                    self.log(f"[brazo] falta la postura '{nombre}': grabarla con  python3 -m tools.brazo grabar {nombre}")
                    return False
            self._salud = self.bus.salud()
            self.bus.torque(True)
            self.garra(abrir=True, esperar=False)
            self.ir(self.cb.postura_espera)
            self.listo = True
            return True
        except Exception as e:
            self.log(f"[brazo] error al preparar: {e}")
            return False

    def cerrar(self):
        if self.bus is not None:
            self.bus.cerrar()

    # ------------------------------------------------------------------ posturas
    def cargar_posturas(self) -> dict:
        p = dict(POSTURAS_DEFECTO)
        try:
            p.update(json.loads(self.ruta_posturas.read_text()))
        except (OSError, ValueError):
            pass
        return p

    def grabar_postura(self, nombre: str) -> dict:
        """Guarda los ángulos ACTUALES de las 5 articulaciones (sin la garra) con ese nombre."""
        ang = self.bus.angulos(IDS)
        post = {str(s): round(math.degrees(a), 2) for s, a in ang.items()}
        try:
            datos = json.loads(self.ruta_posturas.read_text())
        except (OSError, ValueError):
            datos = {}
        datos[nombre] = post
        self.ruta_posturas.write_text(json.dumps(datos, indent=2, ensure_ascii=False))
        self.posturas = self.cargar_posturas()
        return post

    def ir(self, nombre: str, t: float | None = None) -> bool:
        if nombre not in self.posturas:
            raise KeyError(f"no existe la postura '{nombre}' (graba una con: python3 -m tools.brazo grabar {nombre})")
        obj = {int(s): math.radians(v) for s, v in self.posturas[nombre].items()}
        return self._mover(obj, t or self.cb.t_mov_s, f"postura {nombre}")

    def garra(self, abrir: bool | None = None, angulo_deg: float | None = None, esperar: bool = True) -> float:
        """Abre / cierra (o va a un ángulo). Devuelve el ángulo alcanzado (°): con un grano entre los dedos
        la garra se detiene antes de cerrar del todo."""
        if angulo_deg is None:
            angulo_deg = self.cb.garra_abierta_deg if abrir else self.cb.garra_cerrada_deg
        self.bus.mover({ID_GARRA: math.radians(angulo_deg)}, 0.5)
        if not esperar:
            return angulo_deg
        time.sleep(0.6)
        self.bus.esperar_quietos(1.0)
        return math.degrees(self.bus.angulos([ID_GARRA])[ID_GARRA])

    # ------------------------------------------------------------------ cinemática en el sistema del robot
    def a_plano(self, p_robot) -> tuple[float, float, float]:
        """(dirección horizontal [rad, en el robot], r, z sobre la base del brazo)."""
        d = np.asarray(p_robot, float) - self.base
        return math.atan2(d[1], d[0]), math.hypot(d[0], d[1]), float(d[2])

    def yaw_articular(self, direccion: float) -> float:
        return math.radians(self.cb.yaw0_deg) + self.cb.yaw_signo * direccion

    def directa_robot(self, ang: dict) -> tuple[np.ndarray, float]:
        """Ángulos articulares (rad, por id) -> (punto de la garra en el robot, pitch en °)."""
        direccion = (ang[9] - math.radians(self.cb.yaw0_deg)) * self.cb.yaw_signo
        r, z, pitch = directa(self.modelo, [ang[8], ang[11], ang[7], ang[5]])
        p = self.base + np.array([r * math.cos(direccion), r * math.sin(direccion), z])
        return p, (math.degrees(pitch) + 180) % 360 - 180

    def ik(self, p_robot, pitch_deg: float | None = None):
        """Ángulos (rad, por id) para llevar la garra a p_robot. Con pitch_deg=None prueba primero la garra
        horizontal (pitch preferido) y si no alcanza, el rango permitido. Devuelve (ángulos, solución) o None."""
        direccion, r, z = self.a_plano(p_robot)
        if pitch_deg is not None:
            candidatos = [math.radians(pitch_deg)]
        else:
            candidatos = [math.radians(self.cb.pitch_preferido_deg),
                          (math.radians(self.cb.pitch_min_deg), math.radians(self.cb.pitch_max_deg))]
        for c in candidatos:
            s = inversa(self.modelo, r, z, c)
            if s is not None:
                q = s.q
                return {9: self.yaw_articular(direccion), 8: q[0], 11: q[1], 7: q[2], 5: q[3]}, s
        return None

    def plan_agarre(self, p_robot):
        """Puntos de la secuencia (nombre, punto, duración) con la MISMA inclinación de garra, o None."""
        sol = self.ik(p_robot)
        if sol is None:
            return None
        pitch = math.degrees(sol[1].pitch)
        direccion, r, z = self.a_plano(p_robot)
        u = np.array([math.cos(direccion), math.sin(direccion), 0.0])     # entrar en la dirección del brazo
        up = np.array([0.0, 0.0, 1.0])
        p = np.asarray(p_robot, float)
        a, rt = self.cb.aproximacion_m, self.cb.retirar_m

        def resolver(q):
            return self.ik(q, pitch) or self.ik(q)      # misma inclinación; si no alcanza, otra

        plan = []
        for nombre, q, t in (("antes", p - a * u, self.cb.t_mov_s),
                             ("entrando", p - a / 2 * u, self.cb.t_fino_s),
                             ("en el grano", p, self.cb.t_fino_s)):
            s = resolver(q)
            if s is None:
                return None
            plan.append((nombre, s[0], t))
        # Subir lo más que se pueda (la fila alta queda al límite del alcance): basta ~1 cm para sacar
        # la esfera del hueco de 25 mm (su fondo queda ~4 mm bajo la cara del soporte).
        for lv in (self.cb.levantar_m, 0.02, 0.015, 0.01):
            s1, s2 = resolver(p + lv * up), resolver(p + lv * up - rt * u)
            if s1 is not None and s2 is not None:
                return plan + [("subir", s1[0], self.cb.t_fino_s), ("retirar", s2[0], self.cb.t_mov_s * 0.7)]
        return None

    def alcanzable(self, p_robot) -> bool:
        return self.plan_agarre(p_robot) is not None

    # ------------------------------------------------------------------ movimiento vigilado
    def _mover(self, obj: dict, t: float, que: str) -> bool:
        self.bus.mover(obj, t)
        time.sleep(t)
        self.bus.esperar_quietos(1.5)
        err = self.bus.error_seguimiento_deg(obj)
        self._salud = self.bus.salud()
        carga = max(abs(s.carga_pct) for s in self._salud if s.id != ID_GARRA)
        if err > self.cb.error_max_deg or carga > self.cb.carga_max_pct:
            self.ultimo_motivo = f"{que}: error {err:.1f}°, carga {carga:.0f} %"
            return False
        return True

    def ir_a_punto(self, p_robot, pitch_deg=None, t=None) -> bool:
        s = self.ik(p_robot, pitch_deg)
        if s is None:
            self.ultimo_motivo = "fuera de alcance"
            return False
        return self._mover(s[0], t or self.cb.t_mov_s, "punto")

    def _abortar(self, direccion):
        """Algo chocó o no había grano: abrir, retroceder y volver a la postura de espera."""
        motivo = self.ultimo_motivo
        try:
            self.garra(abrir=True)
            ang = self.bus.angulos(IDS)
            p, pitch = self.directa_robot(ang)
            atras = p - self.cb.retirar_m * np.array([math.cos(direccion), math.sin(direccion), 0.0])
            s = self.ik(atras, pitch) or self.ik(atras)
            if s is not None:
                self.bus.mover(s[0], self.cb.t_mov_s)
                time.sleep(self.cb.t_mov_s)
            self.ir(self.cb.postura_espera)
        except Exception as e:
            self.log(f"[brazo] al abortar: {e}")
        self.ultimo_motivo = motivo

    def recoger(self, p_robot, categoria: str, depositar: bool = True) -> bool:
        """Saca UN grano en p_robot y lo deja en el depósito de su categoría. True si quedó en el robot."""
        if self.bus is None:
            return False
        destino = "deposito_maduro" if categoria == cancha.MADURO else "deposito_sobremaduro"
        if depositar and destino not in self.posturas:
            destino = "deposito_maduro" if categoria == cancha.MADURO else None
            if destino is None or destino not in self.posturas:
                # nunca sacar un grano sin saber dónde dejarlo: se caería (−7)
                self.ultimo_motivo = f"falta la postura de depósito para {categoria}"
                self.log(f"[brazo] {self.ultimo_motivo}: no agarro")
                return False
        try:
            self.bus.torque(True)            # por si quedó sin torque en el descanso (sin latigazo)
            ok, motivo = self.bus.puede_trabajar()
            if not ok and self.bus.recuperar_errores():
                self.bus.torque(True)
                ok, motivo = self.bus.puede_trabajar()
            if not ok:
                self.ultimo_motivo = motivo
                self.log(f"[brazo] {motivo}: no agarro")
                return False
            plan = self.plan_agarre(p_robot)
            if plan is None:
                self.ultimo_motivo = "fuera de alcance"
                return False
            direccion = self.a_plano(p_robot)[0]
            self.garra(abrir=True, esperar=False)
            for nombre, obj, t in plan[:3]:
                if not self._mover(obj, t, nombre):
                    self.log(f"[brazo] {self.ultimo_motivo}: aborto")
                    self._abortar(direccion)
                    return False
            cierre = self.garra(abrir=False)
            if cierre > self.cb.garra_vacia_deg:
                self.ultimo_motivo = f"la garra cerró hasta {cierre:.0f}°: no había grano"
                self.log(f"[brazo] {self.ultimo_motivo}")
                self._abortar(direccion)
                return False
            for nombre, obj, t in plan[3:]:
                if not self._mover(obj, t, nombre):
                    self.log(f"[brazo] {self.ultimo_motivo} (con el grano)")
                    break                      # igual se intenta llevarlo al depósito
            if not depositar:
                return True
            self.ir(destino)
            self.garra(abrir=True)
            time.sleep(0.3)                    # que el grano caiga antes de mover el brazo
            self.depositos[categoria] = self.depositos.get(categoria, 0) + 1
            self.ir(self.cb.postura_espera)
            return True
        except Exception as e:           # bus caído, servo apagado por protección...
            self.ultimo_motivo = str(e)
            self.log(f"[brazo] error: {e}")
            try:
                self.bus.recuperar_errores()
            except Exception:
                pass
            return False

    def descansar(self):
        if self.bus is None:
            return
        try:
            self.ir("descanso")
            if self.cb.descanso_sin_torque:
                time.sleep(0.3)
                self.bus.torque(False, IDS)
        except Exception as e:
            self.log(f"[brazo] al descansar: {e}")

    def estado(self) -> str:
        dep = " ".join(f"{k[:4]}={v}" for k, v in self.depositos.items() if k != cancha.VERDE)
        if self.bus is None:
            return dep + " | sin conectar"
        s = self._salud
        if not s:
            return dep
        return f"{dep} | {max(x.temp_c for x in s)} °C máx | {min(x.volt_v for x in s):.1f} V"

    # ------------------------------------------------------------------ diagnóstico
    def esfuerzo_pct(self, ang: dict) -> np.ndarray:
        """Torque por gravedad de hombro, codo1, codo2, muñeca en % del bloqueo (modelo)."""
        q = np.array([ang[8], ang[11], ang[7], ang[5]])
        return 100 * np.abs(torques_gravedad(self.modelo, q)) / self.modelo.tau_max
