"""Compuertas de los depósitos con Dynamixel (4 nov): id 2 = maduros, id 13 = sobremaduros (config.compuertas; antes 12/13).

Van en el MISMO bus que el brazo (U2D2): se usa el BusDXL del brazo (una sola conexión al puerto) o, sin brazo, uno
propio. A diferencia del brazo (posición extendida, varias vueltas), cada compuerta queda en MODO POSICIÓN (3), donde
el servo respeta sus límites (Min/Max Position Limit, en su EEPROM): con `limites_deg` medidos, aunque se pida un
ángulo equivocado la compuerta no pasa de ahí. Ángulos en grados del servo, 0..360 (180° = 2048 pasos).

Sin calibrar (abierta/cerrada = None) la compuerta NO se mueve: sus granos quedan en el robot (+3 / +1), que es mejor
que golpear el mecanismo con un ángulo inventado. Calibrar con  python3 -m tools.compuertas calibrar  (docs/mision.md).

Sentido: no hay un parámetro de "dirección": se graban los ángulos REALES de abierta y cerrada y el servo va de uno
al otro. Lo único que importa es que entre los dos no se pase por 0°/360°: en modo posición el servo no cruza el 0
sino que da la vuelta por el otro lado (330° en vez de 30°) y rompería la compuerta. Correr el cero por software
(Homing Offset) no lo arregla: la lectura sigue saltando en el mismo punto del eje. Si pasa, hay que montar el cuerno
(la pieza entre el servo y la compuerta) girado, para que la compuerta trabaje cerca de 180°, y volver a calibrar.
"""
from __future__ import annotations

import math
import time

from cafebot.arm import dxl
from cafebot.mision.interfaces import Depositos

NOMBRES = ("maduros", "sobremaduros")
ADDR_HOMING = 20               # 4 B con signo, EEPROM: Present Position = real + Homing Offset (solo se muestra)
ADDR_MAX_POS = 48              # 4 B, EEPROM (solo en modo posición)
ADDR_MIN_POS = 52              # 4 B, EEPROM
MODO_POSICION = 3


def grados_a_pasos(g: float) -> int:
    return int(round(g * dxl.PASOS_POR_REV / 360.0))


def pasos_a_grados(p: int) -> float:
    return (p % dxl.PASOS_POR_REV) * 360.0 / dxl.PASOS_POR_REV


def diferencia(a: float, b: float) -> float:
    """De a hasta b por el camino CORTO (−180..180]: lo que se movió la compuerta a mano."""
    return (b - a + 180.0) % 360.0 - 180.0


def cruza_cero(a: float, b: float) -> bool:
    """¿El camino corto entre a y b pasa por 0°/360°? Entonces el servo (que no cruza el 0) iría por el largo."""
    return abs(b - a) > 180.0


def limites_con_margen(a: float, b: float, margen: float) -> tuple:
    """(mín, máx) que contiene a y b con `margen` grados de más a cada lado (dentro de 0..360)."""
    return max(0.0, min(a, b) - margen), min(359.9, max(a, b) + margen)


def indice(que) -> int:
    """'maduros' / 'sobremaduros' / 0 / 1 -> 0 o 1."""
    if isinstance(que, int) or str(que).isdigit():
        k = int(que)
        if k in (0, 1):
            return k
        raise ValueError(f"compuerta {que}: 0 (maduros) o 1 (sobremaduros)")
    q = str(que).lower()
    for k, n in enumerate(NOMBRES):
        if n.startswith(q[:3]):
            return k
    raise ValueError(f"compuerta '{que}': maduros o sobremaduros")


def abrir_bus(cfg, log=print, puerto: str | None = None) -> dxl.BusDXL:
    """El bus del U2D2 solo para las compuertas (sin el brazo: tools.compuertas, o la misión sin --brazo).
    Misma velocidad que el brazo (su perfil)."""
    from cafebot.arm.brazo import PerfilBrazo
    from cafebot.drivers.puertos import resolver
    perfil = PerfilBrazo.cargar(cfg.brazo.perfil)
    puerto = resolver(puerto or perfil.puerto or cfg.puertos.u2d2, "u2d2", log)
    return dxl.BusDXL(puerto, perfil.baudios, [dxl.ConfigServo(i) for i in cfg.compuertas.ids])


class CompuertasDXL:
    def __init__(self, bus: dxl.BusDXL, cc, log=print):
        """bus: un BusDXL abierto (el del brazo sirve). cc: config.compuertas."""
        self.bus, self.cc, self.log = bus, cc, log
        self.ids = tuple(int(i) for i in cc.ids)
        self.perfil_tiempo = {}

    # ------------------------------------------------------------------ estado
    def calibrada(self, k: int) -> bool:
        """Abierta y cerrada grabadas, y sin pasar por 0°/360° entre ellas (el servo daría la vuelta larga)."""
        o, c = self.cc.abierta_deg[k], self.cc.cerrada_deg[k]
        return o is not None and c is not None and not cruza_cero(float(c), float(o))

    def limites(self, k: int):
        lim = self.cc.limites_deg[k]
        return None if lim is None or None in tuple(lim) else (float(lim[0]), float(lim[1]))

    def responde(self, k: int) -> bool:
        return self.bus.pk.ping(self.bus.port, self.ids[k])[1] == 0

    def angulo(self, k: int) -> float:
        return pasos_a_grados(self.bus._r(self.ids[k], dxl.ADDR_PRESENT_POSITION, 4, signo=True))

    def leer(self, k: int) -> dict:
        """Todo lo que dice el servo (para tools.compuertas leer)."""
        sid = self.ids[k]
        r = self.bus._r
        return {"id": sid, "modelo": dxl.MODELOS.get(r(sid, dxl.ADDR_MODEL, 2), r(sid, dxl.ADDR_MODEL, 2)),
                "modo": r(sid, dxl.ADDR_OPERATING_MODE, 1), "torque": bool(r(sid, dxl.ADDR_TORQUE_ENABLE, 1)),
                "angulo": self.angulo(k), "temp_c": r(sid, dxl.ADDR_PRESENT_TEMP, 1),
                "error": r(sid, dxl.ADDR_HW_ERROR, 1), "desfase": self.desfase(k), "carga": self.carga(k),
                "limites_servo": (pasos_a_grados(r(sid, ADDR_MIN_POS, 4)), pasos_a_grados(r(sid, ADDR_MAX_POS, 4)))}

    def diagnostico(self, k: int, grados: float) -> list:
        """Por qué la compuerta k no fue a `grados`: lo que dice el servo (vacío si no se ve nada raro)."""
        sid, r, out = self.ids[k], self.bus._r, []
        err = r(sid, dxl.ADDR_HW_ERROR, 1)
        if err:
            bits = {0: "voltaje", 2: "temperatura", 3: "encoder", 4: "golpe eléctrico", 5: "sobrecarga"}
            out.append(f"error de hardware {err} (" + ", ".join(v for b, v in bits.items() if err >> b & 1) +
                       "): el servo apagó el torque (se reinicia solo antes de moverla)")
        if not r(sid, dxl.ADDR_TORQUE_ENABLE, 1):
            out.append("el torque está APAGADO")
        modo = r(sid, dxl.ADDR_OPERATING_MODE, 1)
        if modo != MODO_POSICION:
            out.append(f"modo {modo} (debía ser {MODO_POSICION}, posición)")
        lo, hi = r(sid, ADDR_MIN_POS, 4), r(sid, ADDR_MAX_POS, 4)
        meta, goal = min(4095, grados_a_pasos(grados)), r(sid, dxl.ADDR_GOAL_POSITION, 4, signo=True)
        if goal != meta:
            out.append(f"el servo NO aceptó la meta ({meta} pasos; tiene {goal}; sus límites {lo}..{hi})")
        crudo = r(sid, dxl.ADDR_PRESENT_POSITION, 4, signo=True)
        if not 0 <= crudo < dxl.PASOS_POR_REV:
            out.append(f"la posición cruda es {crudo} (fuera de una vuelta): reiniciarlo (python3 -m tools.compuertas "
                       "recuperar o apagar y prender)")
        pwm, lim_pwm = r(sid, dxl.ADDR_GOAL_PWM, 2), r(sid, dxl.ADDR_PWM_LIMIT, 2)
        if pwm < 100 or lim_pwm < 100:
            out.append(f"PWM muy bajo (meta {pwm}, límite {lim_pwm} de 885): no tiene fuerza")
        return out

    # ------------------------------------------------------------------ configuración
    def configurar(self, k: int) -> list:
        """Modo posición, perfil suave, PWM limitado y los límites medidos. La EEPROM solo se escribe sin torque (y
        se gasta): solo lo que cambió. Devuelve las celdas escritas."""
        sid = self.ids[k]
        modelo, res, _ = self.bus.pk.ping(self.bus.port, sid)
        if res != 0:
            raise IOError(f"la compuerta de {NOMBRES[k]} (id {sid}) no responde (python3 -m tools.brazo escanear)")
        fw = self.bus._r(sid, dxl.ADDR_FIRMWARE, 1)
        self.perfil_tiempo[k] = fw >= dxl.FIRMWARE_PERFIL_TIEMPO
        eeprom = [(dxl.ADDR_RETURN_DELAY, 1, 0), (dxl.ADDR_OPERATING_MODE, 1, MODO_POSICION),
                  (dxl.ADDR_DRIVE_MODE, 1, 0b100 if self.perfil_tiempo[k] else 0),
                  (dxl.ADDR_PWM_LIMIT, 2, int(self.cc.pwm_limit)), (dxl.ADDR_SHUTDOWN, 1, 52)]
        lim = self.limites(k)
        if lim is not None:
            lo, hi = sorted(lim)
            eeprom += [(ADDR_MIN_POS, 4, max(0, grados_a_pasos(lo))), (ADDR_MAX_POS, 4, min(4095, grados_a_pasos(hi)))]
        cambios = [(a, n, v) for a, n, v in eeprom if self.bus._r(sid, a, n) != v]
        if cambios:
            self.bus._w(sid, dxl.ADDR_TORQUE_ENABLE, 1, 0)
            for a, n, v in cambios:
                self.bus._w(sid, a, n, v)
        return cambios

    def recuperar(self, k: int) -> bool:
        """Si saltó una protección (sobrecarga: algo trabó la compuerta) el torque quedó en 0: reboot."""
        sid = self.ids[k]
        if not self.bus._r(sid, dxl.ADDR_HW_ERROR, 1):
            return False
        self.log(f"[compuertas] la de {NOMBRES[k]} (id {sid}) tenía un error de hardware: la reinicio")
        self.bus.pk.reboot(self.bus.port, sid)
        time.sleep(0.4)
        self.configurar(k)
        self.torque(k, True)                # el reboot lo dejó sin torque
        return True

    def preparar(self) -> list:
        """Configura las compuertas que respondan y activa su torque (quedan sujetas donde estén).
        Devuelve los índices de las que no se pueden usar."""
        malas = []
        for k in (0, 1):
            try:
                self.configurar(k)
                self.recuperar(k)
                self.torque(k, True)
            except IOError as e:
                self.log(f"[compuertas] {e}")
                malas.append(k)
        return malas

    def desfase(self, k: int) -> float:
        """Homing Offset del servo, en grados (si alguien lo cambió: los ángulos quedan corridos en eso)."""
        return self.bus._r(self.ids[k], ADDR_HOMING, 4, signo=True) * 360.0 / dxl.PASOS_POR_REV

    def carga(self, k: int) -> float:
        """Present Load (%, estimada): sujetando la compuerta cerrada debería ser poca; mucha = empuja un tope."""
        return self.bus._r(self.ids[k], dxl.ADDR_PRESENT_LOAD, 2, signo=True) / 10.0

    def torque(self, k: int, activo: bool) -> None:
        sid = self.ids[k]
        if activo:   # sin latigazo: la meta = la posición actual antes de energizar
            self.bus._w(sid, dxl.ADDR_GOAL_POSITION, 4, self.bus._r(sid, dxl.ADDR_PRESENT_POSITION, 4, signo=True))
        self.bus._w(sid, dxl.ADDR_TORQUE_ENABLE, 1, 1 if activo else 0)

    # ------------------------------------------------------------------ movimiento
    def mover(self, k: int, grados: float, t: float | None = None, esperar: bool = True) -> float:
        """Lleva la compuerta k a `grados` (0..360) en t segundos. Devuelve el ángulo alcanzado."""
        lim = self.limites(k)
        if lim is not None and not min(lim) - 0.5 <= grados <= max(lim) + 0.5:
            raise ValueError(f"{grados:.1f}° está fuera de los límites de la compuerta de {NOMBRES[k]} "
                             f"({min(lim):.1f}..{max(lim):.1f}°)")
        if not 0.0 <= grados <= 360.0:
            raise ValueError("ángulo del servo: 0..360°")
        sid = self.ids[k]
        t = self.cc.t_mov_s if t is None else t
        meta = min(4095, grados_a_pasos(grados))
        if self.perfil_tiempo.get(k, True):
            self.bus._w(sid, dxl.ADDR_PROFILE_ACC, 4, int(1000 * t * 0.3))
            self.bus._w(sid, dxl.ADDR_PROFILE_VEL, 4, max(100, int(1000 * t)))
        else:        # firmware viejo: perfil por velocidad
            d = abs(meta - self.bus._r(sid, dxl.ADDR_PRESENT_POSITION, 4, signo=True) % 4096)
            v = d / max(0.1, 0.7 * t)
            self.bus._w(sid, dxl.ADDR_PROFILE_VEL, 4, max(1, min(200, math.ceil(v / dxl.UNIDAD_VEL_PASOS_S))))
            self.bus._w(sid, dxl.ADDR_PROFILE_ACC, 4, max(1, min(200, math.ceil(v / (0.3 * t) / dxl.UNIDAD_ACC_PASOS_S2))))
        self.bus._w(sid, dxl.ADDR_GOAL_POSITION, 4, meta)
        if not esperar:
            return grados
        fin = time.monotonic() + t + 1.0
        time.sleep(min(t, 0.05))
        while time.monotonic() < fin and self.bus._r(sid, dxl.ADDR_MOVING, 1):
            time.sleep(0.02)
        return self.angulo(k)

    def abrir(self, k: int) -> bool:
        return self._a(k, self.cc.abierta_deg[k], "abrir")

    def cerrar(self, k: int) -> bool:
        return self._a(k, self.cc.cerrada_deg[k], "cerrar")

    def _a(self, k, grados, que) -> bool:
        if grados is None:
            self.log(f"[compuertas] la de {NOMBRES[k]} no está calibrada (python3 -m tools.compuertas): no la muevo")
            return False
        self.recuperar(k)
        llego = self.mover(k, float(grados))
        if abs(llego - float(grados)) > 8.0:
            self.log(f"[compuertas] al {que} la de {NOMBRES[k]} quedó en {llego:.0f}° y no en {float(grados):.0f}°: "
                     "¿algo la traba?")
            return False
        return True


class DepositosDXL(Depositos):
    """Las compuertas Dynamixel detrás de la interfaz de la misión. bus_fn() da el BusDXL abierto (el del brazo,
    o abrir_bus(cfg)); se pide la primera vez que hace falta (o en preparar(), antes del botón)."""

    def __init__(self, cfg, bus_fn, log=print):
        self.c, self.cc = cfg.mision, cfg.compuertas
        self._bus_fn = bus_fn
        self.log = log
        self.comp: CompuertasDXL | None = None
        self.malas: list = []

    def preparar(self) -> bool:
        if self.comp is None:
            try:
                self.comp = CompuertasDXL(self._bus_fn(), self.cc, self.log)
            except Exception as e:
                self.log(f"[compuertas] sin bus Dynamixel: {e}")
                return False
            self.malas = self.comp.preparar()
            sin = [NOMBRES[k] for k in (0, 1) if not self.comp.calibrada(k)]
            if sin:
                self.log(f"[compuertas] SIN CALIBRAR: {', '.join(sin)} (no se abren; esos granos quedan en el robot)")
        return self.comp is not None

    def vaciar(self, k: int) -> bool:
        if not self.preparar() or k in self.malas or not self.comp.calibrada(k):
            self.log(f"[compuertas] la de {NOMBRES[k]} no se puede usar: los granos quedan en el robot")
            return False
        try:
            ok = self.comp.abrir(k)
            time.sleep(self.c.t_vaciado_s)
            ok = self.comp.cerrar(k) and ok
            time.sleep(0.2)
            return ok
        except (IOError, ValueError) as e:
            self.log(f"[compuertas] {e}")
            return False

    def cerrar_todo(self):
        if not self.preparar():
            return
        for k in (0, 1):
            if k not in self.malas and self.comp.calibrada(k):
                try:
                    self.comp.cerrar(k)
                except (IOError, ValueError) as e:
                    self.log(f"[compuertas] {e}")
