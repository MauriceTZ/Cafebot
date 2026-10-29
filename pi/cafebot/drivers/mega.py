"""Enlace con el Arduino Mega (base móvil). ÚNICO dueño del puerto serial.

Protocolo: docs/protocolo_mega.md. Resumen:
    Pi -> Mega : "A 500", "W 90", "L 90 300", "G x y th", "V vx wz", "S", ...
    Mega -> Pi : "OK #7"  (meta encolada)      "DONE #7 x y th_mrad"
                 "ABORT #7 MOTIVO x y th_mrad"  "EVT STOP MOTIVO ..."
                 "EVT IR mask subida bajada x y th"  (un sensor de línea cambió: van a `ir_eventos`)
                 "T ms x y th_mrad vx wz_mrad modo postura cola ir"   (telemetría)
                 "IRV v0 v1 v2 v3 mask"  (lecturas crudas de los IR, con `ir_monitor(hz)`)

Uso típico:
    mega = Mega("/dev/cafebot-mega"); mega.conectar()
    mega.telemetria(20)
    r = mega.avanzar(500)            # bloquea hasta DONE/ABORT
    if not r.ok: print("abortado:", r.motivo)
"""
from __future__ import annotations

import math
import queue
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable

NOMBRES_IR = ("DI", "DD", "TI", "TD")   # bits 0..3 del mask del Mega


@dataclass
class Telemetria:
    t_ms: int = 0
    x: float = 0.0        # mm
    y: float = 0.0        # mm
    th: float = 0.0       # rad
    vx: float = 0.0       # mm/s
    wz: float = 0.0       # rad/s
    modo: int = 0
    postura: int = 0
    cola: int = 0
    ir: int = 0           # bit0 DI, bit1 DD, bit2 TI, bit3 TD (1 = negro)
    recibida: float = 0.0  # time.monotonic() local


@dataclass
class Resultado:
    id: int
    ok: bool
    motivo: str = ""
    x: float = 0.0
    y: float = 0.0
    th: float = 0.0


@dataclass
class Evento:
    tipo: str             # "IR", "STOP", ...
    args: list[str] = field(default_factory=list)
    recibido: float = 0.0


@dataclass
class EventoIR:
    """Un sensor de línea cambió de estado. Pose del Mega (su odometría) en el tick del evento."""
    mascara: int          # estado actual (bit k = sensor k sobre NEGRO)
    subida: int           # sensores que pasaron de blanco a negro (entran a una línea)
    bajada: int           # sensores que pasaron de negro a blanco (salen de una línea)
    x: float              # m
    y: float              # m
    th: float             # rad
    recibido: float       # time.monotonic()


@dataclass
class EstadoIR:
    mascara: int
    valores: list         # lectura actual de cada sensor (0..1023)
    umbrales: list
    histeresis: int
    blanco: list
    negro: list
    seguridad: bool
    mascara_freno: int
    auto: bool

    @staticmethod
    def desde(linea: str) -> "EstadoIR":
        """IR mask=5 v=130/860/140/910 u=500/... h=60 b=... n=... seg=0/15 auto=0"""
        d = dict(t.split("=", 1) for t in linea.split()[1:] if "=" in t)
        lista = lambda k: [int(x) for x in d[k].split("/")]
        seg = d.get("seg", "0/15").split("/")
        return EstadoIR(int(d["mask"]), lista("v"), lista("u"), int(d["h"]), lista("b"), lista("n"),
                        seg[0] == "1", int(seg[1]) if len(seg) > 1 else 15, d.get("auto", "0") == "1")


def _pose_desde(tokens: list[str]) -> tuple[float, float, float]:
    try:
        return float(tokens[0]), float(tokens[1]), float(tokens[2]) / 1000.0
    except (IndexError, ValueError):
        return 0.0, 0.0, 0.0


class Mega:
    def __init__(self, puerto: str, baudios: int = 115200, log: Callable[[str], None] | None = None):
        self.puerto = puerto
        self.baudios = baudios
        self.log = log or (lambda s: None)
        self.tele = Telemetria()
        self.eventos: "queue.Queue[Evento]" = queue.Queue(maxsize=200)
        self.ir_eventos: "queue.Queue[EventoIR]" = queue.Queue(maxsize=500)   # los consume la navegación
        self.ir_crudo: tuple | None = None              # (valores, mascara, t) de la última línea IRV
        self.ir_historia: deque = deque(maxlen=3000)    # para graficar / calibrar
        self._ir_mascara = 0
        self._respuestas: "queue.Queue[str]" = queue.Queue()
        self._resultados: dict[int, Resultado] = {}
        self._cv = threading.Condition()
        self._lock_tx = threading.Lock()
        self._serial = None
        self._hilo: threading.Thread | None = None
        self._corriendo = False
        self.version = ""

    # ------------------------------------------------------------------ conexión
    def conectar(self, timeout: float = 4.0, serial_obj=None) -> None:
        """Abre el puerto y espera "READY" (el Mega se reinicia al abrir el USB)."""
        if serial_obj is None:
            import serial
            serial_obj = serial.Serial(self.puerto, self.baudios, timeout=0.05)
        self._serial = serial_obj
        self._corriendo = True
        self._hilo = threading.Thread(target=self._bucle_rx, name="mega-rx", daemon=True)
        self._hilo.start()
        fin = time.monotonic() + timeout
        while time.monotonic() < fin and not self.version:
            time.sleep(0.05)
        if not self.version:
            # Quizá no se reinició (DTR desactivado): preguntar directamente
            self._enviar("H")
            time.sleep(0.3)
        if not self.version:
            raise TimeoutError(f"El Mega no respondió READY en {self.puerto}")

    def cerrar(self) -> None:
        try:
            self.parar()
        except Exception:
            pass
        self._corriendo = False
        if self._hilo:
            self._hilo.join(timeout=1.0)
        if self._serial:
            self._serial.close()

    # ------------------------------------------------------------------ recepción
    def _bucle_rx(self) -> None:
        buf = b""
        while self._corriendo:
            datos = self._serial.read(256)
            if not datos:
                continue
            buf += datos
            while b"\n" in buf:
                linea, buf = buf.split(b"\n", 1)
                self._procesar(linea.decode("ascii", "replace").strip())

    def _procesar(self, linea: str) -> None:
        if not linea:
            return
        tok = linea.split()
        cab = tok[0]
        ahora = time.monotonic()
        if cab == "T" and len(tok) >= 11:
            try:
                v = [float(t) for t in tok[1:11]]
            except ValueError:
                return
            self.tele = Telemetria(int(v[0]), v[1], v[2], v[3] / 1000.0, v[4], v[5] / 1000.0,
                                   int(v[6]), int(v[7]), int(v[8]), int(v[9]), ahora)
            return
        if cab in ("DONE", "ABORT") and len(tok) >= 2 and tok[1].startswith("#"):
            ident = int(tok[1][1:])
            motivo = tok[2] if cab == "ABORT" and len(tok) > 2 else ""
            resto = tok[3:] if cab == "ABORT" else tok[2:]
            x, y, th = _pose_desde(resto)
            with self._cv:
                self._resultados[ident] = Resultado(ident, cab == "DONE", motivo, x, y, th)
                self._cv.notify_all()
            self.log(linea)
            return
        if cab == "IRV" and len(tok) >= 6:
            try:
                v = [int(t) for t in tok[1:6]]
            except ValueError:
                return
            self.ir_crudo = (v[:4], v[4], ahora)
            self.ir_historia.append((ahora, v[:4], v[4]))
            return
        if cab == "EVT" and len(tok) >= 2 and tok[1] == "IR":
            try:
                n = [float(t) for t in tok[2:]]
                if len(n) >= 6:                       # EVT IR mask subida bajada x y th
                    m, sub, baj, x, y, th = int(n[0]), int(n[1]), int(n[2]), n[3], n[4], n[5]
                else:                                 # formato antiguo: EVT IR mask x y th
                    m, x, y, th = int(n[0]), n[1], n[2], n[3]
                    sub, baj = m & ~self._ir_mascara, self._ir_mascara & ~m
            except (ValueError, IndexError):
                return
            self._ir_mascara = m
            ev = EventoIR(m, sub, baj, x / 1000.0, y / 1000.0, th / 1000.0, ahora)
            if self.ir_eventos.full():
                self.ir_eventos.get_nowait()          # nadie los consume: descartar los más viejos
            self.ir_eventos.put(ev)
            return
        if cab == "EVT":
            if self.eventos.full():
                self.eventos.get_nowait()
            self.eventos.put(Evento(tok[1] if len(tok) > 1 else "", tok[2:], ahora))
            self.log(linea)
            return
        if cab == "READY":
            self.version = linea[6:]
            self.log(linea)
            return
        if cab.startswith("=="):
            self.version = self.version or linea
            return
        # Respuesta a un comando (OK, ERR, POSE, ENC, IR, J, ...)
        self._respuestas.put(linea)
        self.log(linea)

    # ------------------------------------------------------------------ envío
    def _enviar(self, linea: str) -> None:
        with self._lock_tx:
            self._serial.write((linea + "\n").encode("ascii"))

    def comando(self, linea: str, timeout: float = 1.0) -> str:
        """Envía y devuelve la primera respuesta (OK/ERR/POSE...)."""
        with self._lock_tx:
            while not self._respuestas.empty():
                self._respuestas.get_nowait()
            self._serial.write((linea + "\n").encode("ascii"))
        try:
            return self._respuestas.get(timeout=timeout)
        except queue.Empty:
            raise TimeoutError(f"Sin respuesta a '{linea}'")

    def _encolar(self, linea: str) -> int:
        r = self.comando(linea)
        if not r.startswith("OK #"):
            raise RuntimeError(f"'{linea}' -> {r}")
        return int(r[4:].split()[0])

    def esperar(self, ident: int, timeout: float = 30.0) -> Resultado:
        with self._cv:
            if not self._cv.wait_for(lambda: ident in self._resultados, timeout):
                self.parar()
                return Resultado(ident, False, "TIMEOUT_PI", self.tele.x, self.tele.y, self.tele.th)
            return self._resultados.pop(ident)

    # ------------------------------------------------------------------ movimientos
    def avanzar(self, mm: float, vel: float = 0, esperar: bool = True, timeout: float = 30.0):
        i = self._encolar(f"A {mm:.0f} {vel:.0f}")
        return self.esperar(i, timeout) if esperar else i

    def girar(self, grados: float, wmax_deg: float = 0, esperar: bool = True, timeout: float = 30.0):
        i = self._encolar(f"W {grados:.1f} {wmax_deg:.1f}")
        return self.esperar(i, timeout) if esperar else i

    def cangrejo(self, angulo_deg: float, mm: float, vel: float = 0, esperar: bool = True, timeout: float = 30.0):
        i = self._encolar(f"L {angulo_deg:.1f} {mm:.0f} {vel:.0f}")
        return self.esperar(i, timeout) if esperar else i

    def ir_a(self, x: float, y: float, th_deg: float | None = None, esperar: bool = True, timeout: float = 60.0):
        linea = f"G {x:.0f} {y:.0f}" + ("" if th_deg is None else f" {th_deg:.1f}")
        i = self._encolar(linea)
        return self.esperar(i, timeout) if esperar else i

    def velocidad(self, vx_mm_s: float, wz_rad_s: float) -> None:
        """Hay que repetirlo cada <500 ms o el Mega se detiene solo (watchdog)."""
        self._enviar(f"V {vx_mm_s:.0f} {wz_rad_s:.3f}")

    def velocidad_cangrejo(self, angulo_deg: float, v_mm_s: float) -> None:
        self._enviar(f"K {angulo_deg:.1f} {v_mm_s:.0f}")

    def parar(self) -> None:
        self.comando("S")

    # ------------------------------------------------------------------ estado / ajustes
    def fijar_pose(self, x_mm: float, y_mm: float, th_deg: float) -> str:
        """Corrige la odometría del Mega con la pose estimada por la Pi (LiDAR, líneas)."""
        return self.comando(f"R {x_mm:.0f} {y_mm:.0f} {th_deg:.2f}")

    def telemetria(self, hz: float) -> str:
        return self.comando(f"M {hz:.0f}")

    def seguridad_ir(self, activa: bool, mascara: int | None = None) -> str:
        """Freno inmediato al ver una línea NUEVA (solo los sensores de `mascara`; None = todos)."""
        return self.comando(f"IS {1 if activa else 0}" + ("" if mascara is None else f" {mascara}"))

    # ------------------------------------------------------------------ sensores de línea (QTR-1A)
    def ir_estado(self) -> EstadoIR:
        return EstadoIR.desde(self.comando("I"))

    def ir_monitor(self, hz: float) -> str:
        """Lecturas crudas a hz (0 = apagar): quedan en `ir_crudo` e `ir_historia`."""
        return self.comando(f"IM {hz:.0f}")

    def ir_calibrar_blanco(self) -> EstadoIR:
        """Los 4 sensores sobre la lona blanca."""
        return EstadoIR.desde(self.comando("IW", timeout=2.0))

    def ir_calibrar_negro(self, k: int | None = None) -> EstadoIR:
        """El sensor k (o los 4) sobre una línea negra. Si no hay contraste con el blanco, el Mega no lo
        guarda y manda "EVT IRCAL contraste_bajo k lectura blanco" (queda en `eventos`)."""
        return EstadoIR.desde(self.comando("IN" + ("" if k is None else f" {k}"), timeout=2.0))

    def ir_auto(self, iniciar: bool) -> dict:
        """IA 1: empieza a registrar mín./máx.; IA 0: termina y calcula blanco/negro/umbral de los
        sensores que vieron contraste. Devuelve {'calibrados': mascara, 'min': [...], 'max': [...]}."""
        r = self.comando(f"IA {1 if iniciar else 0}")
        if not r.startswith("OK IA") or iniciar:
            return {"respuesta": r}
        d = dict(t.split("=", 1) for t in r.split()[2:] if "=" in t)
        return {"calibrados": int(d.get("calibrados", 0)),
                "min": [int(x) for x in d.get("min", "0/0/0/0").split("/")],
                "max": [int(x) for x in d.get("max", "0/0/0/0").split("/")], "respuesta": r}

    def ir_umbral(self, k: int, valor: int) -> EstadoIR:
        return EstadoIR.desde(self.comando(f"IU {k} {int(valor)}"))

    def ir_histeresis(self, valor: int) -> EstadoIR:
        return EstadoIR.desde(self.comando(f"IH {int(valor)}"))

    def ir_guardar(self) -> str:
        """Guarda la calibración IR en la EEPROM del Mega."""
        return self.comando("IG")

    def ir_defecto(self) -> EstadoIR:
        return EstadoIR.desde(self.comando("ID"))

    def ir_aplicar(self, umbrales=None, histeresis=None) -> EstadoIR | None:
        """Envía los umbrales de la config de la Pi (robot_local.json, sección "ir"), si los hay."""
        est = None
        if umbrales is not None:
            for k, u in enumerate(umbrales):
                est = self.ir_umbral(k, u)
        if histeresis is not None:
            est = self.ir_histeresis(histeresis)
        return est

    def enviar_sim(self, linea: str) -> None:
        """Solo con el firmware simulado: líneas "@A pin valor" para la planta (no son comandos)."""
        self._enviar(linea)

    def pose(self) -> tuple[float, float, float]:
        r = self.comando("P")
        x, y, th = _pose_desde(r.split()[1:4])
        return x, y, th

    def pose_telemetria(self) -> tuple[float, float, float]:
        t = self.tele
        return t.x, t.y, t.th

    @staticmethod
    def grados(rad: float) -> float:
        return math.degrees(rad)
