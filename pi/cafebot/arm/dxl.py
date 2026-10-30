"""Bus Dynamixel del brazo (XL430-W250-T, Protocolo 2.0) configurado para CUIDAR los servos.

Cada valor sale del manual oficial:
https://docs.robotis.com/docs/dxl/model_reference/x_series/xl_series/xl430-w250/

Qué cambia respecto a legacy/codigo_brazo/main.py y por qué:
  * Return Delay Time 250 (500 µs) -> 0: cada lectura del bus tarda mucho menos.
  * Perfil por TIEMPO (Drive Mode bit 2): se manda la posición final y la duración; todas
    las articulaciones llegan juntas, con aceleración suave -> menos picos de corriente
    y menos escrituras por el bus (antes: 50 escrituras/s interpolando en Python).
  * Position I Gain = 0 en el brazo (default de fábrica). Con I = 150 más el integrador
    externo, si una postura no se alcanza el servo termina empujando al 100 % de PWM
    hasta que salta la protección de sobrecarga.
  * PWM Limit de la garra bajo: un grano pesa 12 g, apretarlo al 100 % es un "stall"
    permanente mientras se transporta = calor sin ningún beneficio.
  * Monitoreo de temperatura (límite de fábrica 72 °C; tras un apagado por calor el
    manual pide esperar 20 min), voltaje y Hardware Error Status. Si salta la
    protección (Shutdown: sobrecarga, calor, choque eléctrico) el torque queda en 0 y
    solo vuelve con un REBOOT: este módulo lo detecta y lo hace.
  * Bus Watchdog opcional: si el programa de la Pi se cuelga, el brazo se detiene.
  * Ceros NORMALIZADOS al conectar: en modo posición extendida, al encender cada servo lee su
    posición dentro de UNA vuelta (0..4095). Si el cero quedó cerca del borde, mandar el cero
    "tal cual" haría girar la articulación casi una vuelta entera. Igual que el código antiguo,
    el cero efectivo se corre en múltiplos de 4096 para quedar a menos de media vuelta.
  * Perfil por tiempo solo con firmware ≥ 42. Con un firmware anterior, Profile Velocity se
    interpretaría como VELOCIDAD (1500 → 343 rpm): en ese caso se calcula un perfil por velocidad.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

# ---- Tabla de control (manual XL430-W250) ----
ADDR_MODEL = 0                 # 2 B (XL430-W250 = 1060)
ADDR_FIRMWARE = 6              # 1 B
ADDR_ID = 7                    # 1 B, EEPROM, fábrica 1
ADDR_BAUD = 8                  # 1 B, EEPROM, fábrica 1 (57 600)
ADDR_RETURN_DELAY = 9          # 1 B, EEPROM, unidad 2 µs
ADDR_DRIVE_MODE = 10           # 1 B, EEPROM, bit2 = perfil por tiempo
ADDR_OPERATING_MODE = 11       # 1 B, EEPROM, 3 = posición, 4 = posición extendida
ADDR_TEMP_LIMIT = 31           # 1 B, EEPROM, °C (fábrica 72, NO subir)
ADDR_PWM_LIMIT = 36            # 2 B, EEPROM, 0..885 (885 = 100 %), tope absoluto
ADDR_SHUTDOWN = 63             # 1 B, EEPROM, fábrica 52 = sobrecarga|choque|calor
ADDR_TORQUE_ENABLE = 64
ADDR_LED = 65
ADDR_HW_ERROR = 70             # 1 B
ADDR_POS_D = 80
ADDR_POS_I = 82
ADDR_POS_P = 84
ADDR_BUS_WATCHDOG = 98         # 1 B, unidad 20 ms, 0 = apagado
ADDR_GOAL_PWM = 100            # 2 B, en modo posición es un limitador final
ADDR_PROFILE_ACC = 108         # 4 B, ms si el perfil es por tiempo
ADDR_PROFILE_VEL = 112         # 4 B, ms (duración total) si el perfil es por tiempo
ADDR_GOAL_POSITION = 116       # 4 B
ADDR_MOVING = 122
ADDR_PRESENT_LOAD = 126        # 2 B con signo, 0,1 % (ESTIMADO, solo tendencia)
ADDR_PRESENT_POSITION = 132    # 4 B
ADDR_PRESENT_VOLTAGE = 144     # 2 B, 0,1 V
ADDR_PRESENT_TEMP = 146        # 1 B, °C

FIRMWARE_PERFIL_TIEMPO = 42
UNIDAD_VEL_PASOS_S = 0.229 * 4096 / 60.0      # 1 unidad de Profile Velocity = 0,229 rpm
UNIDAD_ACC_PASOS_S2 = 214.577 * 4096 / 3600.0  # 1 unidad de Profile Acceleration = 214,577 rev/min²
HW_ERR_BITS = {0: "voltaje", 2: "sobrecalentamiento", 3: "encoder", 4: "choque eléctrico", 5: "sobrecarga"}
BAUD_VALOR = {9_600: 0, 57_600: 1, 115_200: 2, 1_000_000: 3, 2_000_000: 4, 3_000_000: 5, 4_000_000: 6,
              4_500_000: 7}                    # valor de Baud Rate (8) según el manual
MODELOS = {1060: "XL430-W250", 1090: "2XL430-W250", 1070: "XC430-W150", 1080: "XC430-W240",
           1190: "XL330-M077", 1200: "XL330-M288", 1020: "XM430-W350", 1030: "XM430-W210"}

PASOS_POR_REV = 4096
TEMP_AVISO_C = 60       # avisar y preferir descanso
TEMP_DESCANSO_C = 66    # no iniciar más agarres hasta enfriar
VOLT_MIN_V = 10.5       # bajo esto el torque disponible cae (1,0 N·m a 9 V)


@dataclass
class ConfigServo:
    id: int
    p: int = 640            # ganancias de fábrica del XL430
    i: int = 0
    d: int = 3600
    pwm_limit: int = 885
    invertir: bool = False
    centro_pasos: int = 2048  # paso que corresponde a 0 rad (cero ortogonal)


# IDs y sentidos de legacy/codigo_brazo/main.py (config_giro). Ceros = pasos con el brazo RECTO hacia arriba:
# 29 oct, brazo rearmado, leídos con el programa antiguo (antes: base 3088, hombro 1042, codo1 3074, codo2 1546).
# El perfil del brazo (pi/brazos/<nombre>.json, `tools.brazo cero`) manda sobre estos valores.
BRAZO = [
    ConfigServo(9, invertir=False, centro_pasos=3085),   # base (yaw)      29 oct: lectura -1011 (= 3085 + vueltas)
    ConfigServo(8, invertir=True, centro_pasos=2045),    # hombro          29 oct (88° distinto del de antes)
    ConfigServo(11, invertir=True, centro_pasos=3143),   # codo1           29 oct
    ConfigServo(7, invertir=True, centro_pasos=1532),    # codo2           29 oct
    ConfigServo(5, invertir=True, centro_pasos=1014),    # muñeca          [CONFIRMAR] 29 oct no respondió
]
GARRA = ConfigServo(4, p=600, i=0, d=1000, pwm_limit=300, centro_pasos=336)  # ~34 % de PWM


@dataclass
class Salud:
    id: int
    temp_c: int
    volt_v: float
    carga_pct: float
    error: int

    @property
    def errores(self) -> list[str]:
        return [n for b, n in HW_ERR_BITS.items() if self.error & (1 << b)]


def escanear(puerto: str, baudios=(1_000_000, 57_600, 115_200, 2_000_000)) -> dict:
    """¿Qué servos hay en el bus y a qué velocidad? {baudios: {id: (modelo, firmware)}}.
    Los XL430 salen de fábrica a 57 600; el código antiguo los dejó a 1 000 000."""
    from dynamixel_sdk import PacketHandler, PortHandler
    port, pk = PortHandler(puerto), PacketHandler(2.0)
    if not port.openPort():
        raise IOError(f"No se pudo abrir {puerto}")
    out = {}
    try:
        for b in baudios:
            if not port.setBaudRate(b):
                continue
            datos, res = pk.broadcastPing(port)
            if res == 0 and datos:
                out[b] = {sid: (int(v[0]), int(v[1])) for sid, v in datos.items()}
    finally:
        port.closePort()
    return out


def cambiar_id(puerto: str, viejo: int, nuevo: int, baudios_actual: int = 57_600,
               baudios_nuevo: int = 1_000_000) -> None:
    """Deja un servo NUEVO (de fábrica: id 1 a 57 600) con el id y la velocidad del brazo. Escribe su EEPROM.
    Si otro servo del bus tiene el mismo id viejo a esa velocidad (todos los de fábrica son id 1), conectar SOLO
    el servo nuevo. No se hace si el id nuevo ya está ocupado (dos servos con el mismo id no se pueden separar
    sin desconectarlos)."""
    from dynamixel_sdk import PacketHandler, PortHandler
    if not 0 <= nuevo <= 252 or baudios_nuevo not in BAUD_VALOR:
        raise ValueError(f"id 0..252 y baudios en {sorted(BAUD_VALOR)}")
    port, pk = PortHandler(puerto), PacketHandler(2.0)
    if not port.openPort():
        raise IOError(f"No se pudo abrir {puerto}")

    def responde(sid, baudios):
        port.setBaudRate(baudios)
        return pk.ping(port, sid)[1] == 0

    def w(sid, addr, val):
        res, _ = pk.write1ByteTxRx(port, sid, addr, val)
        if res != 0:
            raise IOError(f"DXL {sid}: escritura {addr} falló ({pk.getTxRxResult(res)})")

    try:
        if nuevo != viejo or baudios_nuevo != baudios_actual:
            if responde(nuevo, baudios_nuevo):
                raise IOError(f"ya hay un servo con id {nuevo} a {baudios_nuevo} baudios")
        if not responde(viejo, baudios_actual):
            raise IOError(f"el id {viejo} no responde a {baudios_actual} baudios (python3 -m tools.brazo escanear)")
        w(viejo, ADDR_TORQUE_ENABLE, 0)          # la EEPROM solo se escribe sin torque
        if nuevo != viejo:
            w(viejo, ADDR_ID, nuevo)
        if baudios_nuevo != baudios_actual:
            try:                                 # responde a la velocidad VIEJA y recién ahí cambia
                w(nuevo, ADDR_BAUD, BAUD_VALOR[baudios_nuevo])
            except IOError:
                pass                             # se verifica abajo
            time.sleep(0.05)
        if not responde(nuevo, baudios_nuevo):
            raise IOError(f"no responde como id {nuevo} a {baudios_nuevo} baudios (python3 -m tools.brazo escanear)")
    finally:
        port.closePort()


class BusDXL:
    def __init__(self, puerto: str = "/dev/cafebot-u2d2", baudios: int = 1_000_000, servos=None):
        """servos: lista de ConfigServo (ids, ceros y sentidos de ESTE brazo; ver arm/brazo.py, perfiles).
        Sin ella se usan los del código antiguo (BRAZO + GARRA)."""
        from dynamixel_sdk import PacketHandler, PortHandler

        self.port = PortHandler(puerto)
        self.pk = PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(baudios):
            raise IOError(f"No se pudo abrir {puerto}")
        self.servos = {c.id: c for c in (servos or BRAZO + [GARRA])}
        self.centro = {c.id: c.centro_pasos for c in self.servos.values()}   # cero efectivo (normalizado)
        self.firmware: dict[int, int] = {}
        self.perfil_tiempo = True

    # -------------------------------------------------------------- inventario
    def ping(self) -> dict[int, tuple[int, int]]:
        """{id: (modelo, firmware)} de los servos que responden."""
        vivos = {}
        for sid in self.servos:
            modelo, res, _ = self.pk.ping(self.port, sid)
            if res == 0:
                fw = self._r(sid, ADDR_FIRMWARE, 1)
                vivos[sid] = (modelo, fw)
                self.firmware[sid] = fw
        return vivos

    def normalizar_centros(self) -> dict[int, int]:
        """Corre cada cero en múltiplos de 4096 para que quede a menos de media vuelta de la posición
        actual (ver arriba). Llamar después de encender, de configurar y de cada reboot."""
        for sid, c in self.servos.items():
            pos = self._r(sid, ADDR_PRESENT_POSITION, 4, signo=True)
            dif = ((pos - c.centro_pasos + 2048) % PASOS_POR_REV) - 2048
            self.centro[sid] = pos - dif
        return dict(self.centro)

    # -------------------------------------------------------------- bajo nivel
    def _w(self, sid: int, addr: int, n: int, val: int) -> None:
        f = {1: self.pk.write1ByteTxRx, 2: self.pk.write2ByteTxRx, 4: self.pk.write4ByteTxRx}[n]
        res, err = f(self.port, sid, addr, val & ((1 << (8 * n)) - 1))
        if res != 0:
            raise IOError(f"DXL {sid}: escritura {addr} falló ({self.pk.getTxRxResult(res)})")

    def _r(self, sid: int, addr: int, n: int, signo: bool = False) -> int:
        f = {1: self.pk.read1ByteTxRx, 2: self.pk.read2ByteTxRx, 4: self.pk.read4ByteTxRx}[n]
        val, res, err = f(self.port, sid, addr)
        if res != 0:
            raise IOError(f"DXL {sid}: lectura {addr} falló ({self.pk.getTxRxResult(res)})")
        if signo and val >= 1 << (8 * n - 1):
            val -= 1 << (8 * n)
        return val

    # -------------------------------------------------------------- configuración
    def configurar(self) -> list[int]:
        """Escribe la configuración "cuidadosa". Las celdas EEPROM solo se pueden escribir SIN torque (y se
        gastan), así que solo se apaga el torque de un servo si algo de su EEPROM hay que cambiarlo (la primera
        vez). Devuelve los ids a los que se les apagó el torque."""
        if not self.firmware:
            self.ping()
        self.perfil_tiempo = bool(self.firmware) and min(self.firmware.values()) >= FIRMWARE_PERFIL_TIEMPO
        apagados = []
        for c in self.servos.values():
            eeprom = [(ADDR_RETURN_DELAY, 1, 0),
                      (ADDR_OPERATING_MODE, 1, 4),                  # posición extendida (como legacy)
                      (ADDR_DRIVE_MODE, 1, 0b100 if self.perfil_tiempo else 0),
                      (ADDR_PWM_LIMIT, 2, c.pwm_limit),
                      (ADDR_SHUTDOWN, 1, 52)]                       # protecciones de fábrica: NO quitarlas
            cambios = [(a, n, v) for a, n, v in eeprom if self._r(c.id, a, n) != v]
            if cambios:
                self._w(c.id, ADDR_TORQUE_ENABLE, 1, 0)
                apagados.append(c.id)
                for a, n, v in cambios:
                    self._w(c.id, a, n, v)
            self._w(c.id, ADDR_POS_P, 2, c.p)                      # RAM: se puede con torque
            self._w(c.id, ADDR_POS_I, 2, c.i)
            self._w(c.id, ADDR_POS_D, 2, c.d)
        self.normalizar_centros()
        return apagados

    def torque(self, activo: bool, ids=None) -> None:
        for sid in ids or self.servos:
            if activo:  # sin latigazo: la meta = posición actual antes de energizar
                self._w(sid, ADDR_GOAL_POSITION, 4, self._r(sid, ADDR_PRESENT_POSITION, 4, signo=True))
            self._w(sid, ADDR_TORQUE_ENABLE, 1, 1 if activo else 0)

    # -------------------------------------------------------------- movimiento
    def rad_a_pasos(self, sid: int, rad: float) -> int:
        c = self.servos[sid]
        return int(round(self.centro[sid] + (-1 if c.invertir else 1) * rad * PASOS_POR_REV / (2 * math.pi)))

    def pasos_a_rad(self, sid: int, pasos: int) -> float:
        c = self.servos[sid]
        return (-1 if c.invertir else 1) * (pasos - self.centro[sid]) * 2 * math.pi / PASOS_POR_REV

    def pasos(self, ids=None) -> dict[int, int]:
        """Posición actual en pasos del encoder (sin cero ni sentido): lo que se graba como cero."""
        return {sid: self._r(sid, ADDR_PRESENT_POSITION, 4, signo=True) for sid in (ids or self.servos)}

    def angulos(self, ids=None) -> dict[int, float]:
        """Ángulo actual (rad, respecto del cero) de cada articulación."""
        return {sid: self.pasos_a_rad(sid, p) for sid, p in self.pasos(ids).items()}

    def led(self, sid: int, encendido: bool) -> None:
        self._w(sid, ADDR_LED, 1, 1 if encendido else 0)

    def mover(self, objetivos: dict[int, float], duracion_s: float, acel_frac: float = 0.3) -> None:
        """Mueve varias articulaciones (rad) para que TODAS lleguen en duracion_s,
        con rampas de aceleración/frenado de acel_frac de la duración."""
        t_ms = max(100, int(duracion_s * 1000))
        a_ms = int(t_ms * min(0.5, acel_frac))
        metas = {sid: self.rad_a_pasos(sid, rad) for sid, rad in objetivos.items()}
        for sid, meta in metas.items():
            if self.perfil_tiempo:
                self._w(sid, ADDR_PROFILE_ACC, 4, a_ms)
                self._w(sid, ADDR_PROFILE_VEL, 4, t_ms)
            else:   # perfil por velocidad: la velocidad que hace falta para llegar en t con rampas
                d = abs(meta - self._r(sid, ADDR_PRESENT_POSITION, 4, signo=True))
                f = max(0.1, min(0.5, acel_frac))
                v = d / (duracion_s * (1 - f)) if duracion_s > 0 else 0
                self._w(sid, ADDR_PROFILE_VEL, 4, max(1, min(200, math.ceil(v / UNIDAD_VEL_PASOS_S))))
                self._w(sid, ADDR_PROFILE_ACC, 4, max(1, min(200, math.ceil(v / (f * duracion_s) / UNIDAD_ACC_PASOS_S2))))
        for sid, meta in metas.items():
            self._w(sid, ADDR_GOAL_POSITION, 4, meta)

    def error_seguimiento_deg(self, objetivos: dict[int, float]) -> float:
        """Mayor diferencia (°) entre la meta y la posición alcanzada: si es grande, algo lo bloquea."""
        act = self.angulos(list(objetivos))
        return max(abs(math.degrees(objetivos[s] - act[s])) for s in objetivos)

    def esperar_quietos(self, timeout_s: float = 5.0) -> bool:
        fin = time.monotonic() + timeout_s
        while time.monotonic() < fin:
            if not any(self._r(sid, ADDR_MOVING, 1) for sid in self.servos):
                return True
            time.sleep(0.02)
        return False

    # -------------------------------------------------------------- salud
    def salud(self) -> list[Salud]:
        out = []
        for sid in self.servos:
            out.append(Salud(sid, self._r(sid, ADDR_PRESENT_TEMP, 1),
                             self._r(sid, ADDR_PRESENT_VOLTAGE, 2) / 10.0,
                             self._r(sid, ADDR_PRESENT_LOAD, 2, signo=True) / 10.0,
                             self._r(sid, ADDR_HW_ERROR, 1)))
        return out

    def puede_trabajar(self) -> tuple[bool, str]:
        """False si algún servo está caliente, con voltaje bajo o con error."""
        for s in self.salud():
            if s.error:
                return False, f"servo {s.id}: {', '.join(s.errores)}"
            if s.temp_c >= TEMP_DESCANSO_C:
                return False, f"servo {s.id} a {s.temp_c} °C"
            if s.volt_v < VOLT_MIN_V:
                return False, f"voltaje {s.volt_v:.1f} V"
        return True, ""

    def recuperar_errores(self) -> list[int]:
        """Reinicia los servos que saltaron por protección y vuelve a configurarlos."""
        reiniciados = []
        for s in self.salud():
            if s.error:
                self.pk.reboot(self.port, s.id)
                reiniciados.append(s.id)
        if reiniciados:
            time.sleep(0.5)
            self.configurar()          # también re-normaliza los ceros (el reboot reinicia la vuelta)
        return reiniciados

    def watchdog(self, ms: int) -> None:
        """Bus Watchdog: si no hay comunicación en `ms`, el servo se detiene (0 = apagado)."""
        v = 0 if ms <= 0 else max(1, min(127, ms // 20))
        for sid in self.servos:
            self._w(sid, ADDR_BUS_WATCHDOG, 1, v)

    def cerrar(self) -> None:
        self.port.closePort()
