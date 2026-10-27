"""Enlace con el Arduino Mega (base móvil). ÚNICO dueño del puerto serial.

Protocolo: docs/protocolo_mega.md. Resumen:
    Pi -> Mega : "A 500", "W 90", "L 90 300", "G x y th", "V vx wz", "S", ...
    Mega -> Pi : "OK #7"  (meta encolada)      "DONE #7 x y th_mrad"
                 "ABORT #7 MOTIVO x y th_mrad"  "EVT IR mask x y th" / "EVT STOP MOTIVO ..."
                 "T ms x y th_mrad vx wz_mrad modo postura cola ir"   (telemetría)

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
from dataclasses import dataclass, field
from typing import Callable


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
        self.eventos: "queue.Queue[Evento]" = queue.Queue()
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
        if cab == "EVT":
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

    def seguridad_ir(self, activa: bool) -> str:
        return self.comando(f"IS {1 if activa else 0}")

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
