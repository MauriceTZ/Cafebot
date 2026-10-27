"""Misión de cosecha: árbol (LiDAR) -> ruta -> granos (cámara) -> brazo -> depósito.

Máquina de estados NO bloqueante: el bucle principal llama a `update()` en cada vuelta.
No mueve motores directamente: reutiliza el RouteExecutor existente (vía `approach_fn`) y,
cuando exista, el brazo (vía brazo_interfaz.BrazoBase). La percepción solo informa; aquí se decide.

    INACTIVA -> APROXIMANDO -> ESTABILIZANDO -> ESCANEANDO -> RECOGIENDO -> (ESPERA_RESCAN) -> ESCANEANDO ...
                                                    \\-> FIN  (sin más granos / sin brazo / tiempo)

Regla del proyecto: ante la duda NO se toca (0 puntos es mejor que -7). Solo se recogen granos
con categoría MADURO / SOBREMADURO vista de forma ESTABLE en N cuadros seguidos.

Pendiente a propósito (sin los sensores IR del borde no hay exploración autónoma): el árbol lo
elige el operador con [TAB] y se inicia con [K]; el traslado a las cajas del beneficiadero se
agrega cuando se mapeen las cajas.
"""
from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from enum import Enum

PUNTOS = {"MADURO": 7, "SOBREMADURO": 5}      # puntaje en la caja correcta (reglamento)


class Estado(Enum):
    INACTIVA = "inactiva"
    APROXIMANDO = "aproximándose al árbol"
    ESTABILIZANDO = "estabilizando cámara"
    ESCANEANDO = "escaneando granos"
    RECOGIENDO = "recogiendo grano"
    ESPERA_RESCAN = "esperando nuevo escaneo"
    FIN = "misión terminada"
    ABORTADA = "misión abortada"


@dataclass
class ConfigMision:
    t_max_aproximacion_s: float = 180.0
    t_estabilizar_s: float = 1.0        # robot quieto + cámara estable antes de mirar
    t_escaneo_max_s: float = 8.0        # sin granos recogibles estables en este tiempo -> FIN
    t_espera_rescan_s: float = 1.5      # tras recoger: dejar que la cámara vea el soporte vacío
    cuadros_min_rescan: int = 4         # cuadros nuevos antes de volver a decidir
    intentos_por_grano: int = 2         # 1 intento + 1 reintento (CLAUDE.md), luego se salta
    t_max_recogida_s: float = 40.0
    max_granos: int = 16                # tope por árbol
    deadline_ronda: float | None = None # time.monotonic() absoluto del fin de ronda (8 min)


class MisionCosecha:
    def __init__(self, vision, brazo, route_executor, approach_fn, cfg: ConfigMision | None = None,
                 reloj=time.monotonic):
        """vision: VisionWorker | route_executor: RouteExecutor (usa .active y .status)
        approach_fn(lm) -> (ok: bool, mensaje: str): planifica y ARRANCA la ruta al árbol."""
        self.vision = vision
        self.brazo = brazo
        self.route = route_executor
        self.approach_fn = approach_fn
        self.cfg = cfg or ConfigMision()
        self.reloj = reloj
        self.estado = Estado.INACTIVA
        self.arbol = None
        self.bitacora = deque(maxlen=8)
        self.recogidos = {"MADURO": 0, "SOBREMADURO": 0}
        self.fallidos = 0
        self.objetivo = None
        self._bloqueados = []           # (cx, cy, diam) de granos que fallaron: se saltan
        self._intentos = 0
        self._t_estado = 0.0
        self._frame_min = 0
        self._reportados = set()

    # ------------------------------------------------------------ utilidades
    @property
    def activa(self) -> bool:
        return self.estado not in (Estado.INACTIVA, Estado.FIN, Estado.ABORTADA)

    @property
    def puntaje_estimado(self) -> int:
        return sum(PUNTOS[k] * v for k, v in self.recogidos.items())

    def _log(self, texto):
        self.bitacora.append(texto)

    def _ir(self, estado: Estado):
        self.estado = estado
        self._t_estado = self.reloj()

    def _en_estado(self) -> float:
        return self.reloj() - self._t_estado

    # ------------------------------------------------------------ control
    def iniciar(self, lm):
        if self.activa:
            return False, "La misión ya está en curso"
        if lm is None or lm.cls != "ARBOL":
            return False, "Selecciona un landmark ARBOL confirmado con [TAB]"
        if self.vision is None:
            return False, "Visión no habilitada: ejecuta con --vision"
        ok, msg = self.approach_fn(lm)
        if not ok:
            return False, msg
        self.arbol = lm.name
        self.recogidos = {"MADURO": 0, "SOBREMADURO": 0}
        self.fallidos = 0
        self._bloqueados = []
        self._reportados = set()
        self.objetivo = None
        self._log(f"Aproximación a {lm.name}")
        self._ir(Estado.APROXIMANDO)
        return True, f"Misión de cosecha en {lm.name}: {msg}"

    def abortar(self, motivo="abortada por el operador"):
        if not self.activa:
            return
        try:
            self.brazo.cancelar()
        except Exception:
            pass
        self._log(f"ABORTADA: {motivo}")
        self._ir(Estado.ABORTADA)
        self._apagar_vision()

    def _apagar_vision(self):
        if self.vision is not None and not getattr(self.vision, "manual", False):
            self.vision.set_active(False)

    def _terminar(self, motivo):
        self._log(f"FIN: {motivo}")
        self._ir(Estado.FIN)
        self._apagar_vision()

    # ------------------------------------------------------------ selección
    def _es_bloqueado(self, g) -> bool:
        return any(math.hypot(g.cx - x, g.cy - y) < 0.8 * max(g.diam_px, d)
                   for x, y, d in self._bloqueados)

    def elegir_objetivo(self, granos):
        """Solo granos ESTABLES MADURO/SOBREMADURO, no bloqueados y con depósito libre.
        Prioridad: maduros (+7) antes que sobremaduros (+5); luego el más cercano al centro."""
        cands = [g for g in granos
                 if g.recogible and not self._es_bloqueado(g)
                 and not self.brazo.deposito_lleno(g.categoria)]
        if not cands:
            return None
        return min(cands, key=lambda g: (0 if g.categoria == "MADURO" else 1,
                                         math.hypot(g.dx_mm, g.dy_mm)))

    # ------------------------------------------------------------ bucle
    def update(self):
        if not self.activa:
            return
        c = self.cfg
        if c.deadline_ronda is not None and self.reloj() >= c.deadline_ronda:
            self.brazo.cancelar()
            self._terminar("se acabó el tiempo de la ronda")
            return

        if self.estado is Estado.APROXIMANDO:
            if self.route is not None and self.route.active:
                if self._en_estado() > c.t_max_aproximacion_s:
                    self.route.stop()
                    self.abortar("tiempo máximo de aproximación")
                return
            if self.route is not None and self.route.status == "ruta completada":
                self._log("Llegué al árbol: estabilizando cámara")
                self.vision.set_active(True)
                self._ir(Estado.ESTABILIZANDO)
            else:
                st = self.route.status if self.route is not None else "sin ejecutor"
                self.abortar(f"la aproximación no terminó ({st})")

        elif self.estado is Estado.ESTABILIZANDO:
            if self._en_estado() >= c.t_estabilizar_s:
                self._frame_min = self.vision.snapshot().frame_id + c.cuadros_min_rescan
                self._ir(Estado.ESCANEANDO)

        elif self.estado is Estado.ESCANEANDO:
            snap = self.vision.snapshot()
            if snap.frame_id < self._frame_min:
                if self._en_estado() > c.t_escaneo_max_s + 6.0:      # la cámara no entrega cuadros
                    self.abortar(f"sin imágenes de la cámara ({self.vision.error or self.vision.status})")
                return
            for g in snap.granos:                   # informar lo visto (una vez por grano)
                if g.estable and g.track_id not in self._reportados:
                    self._reportados.add(g.track_id)
                    self._log(f"Visto: {g.resumen()} -> {g.accion}")
            if sum(self.recogidos.values()) + self.fallidos >= c.max_granos:
                self._terminar(f"tope de {c.max_granos} granos alcanzado")
                return
            objetivo = self.elegir_objetivo(snap.granos)
            if objetivo is None:
                if self._en_estado() >= c.t_escaneo_max_s:
                    self._terminar("no quedan granos recogibles estables")
                return
            if not self.brazo.disponible():
                self.objetivo = objetivo
                self._terminar(f"BRAZO NO INTEGRADO: recogería {objetivo.resumen()}")
                return
            self.objetivo = objetivo
            self._intentos = 0
            self._iniciar_recogida(objetivo)

        elif self.estado is Estado.RECOGIENDO:
            r = self.brazo.estado()
            if r == "OK":
                self.brazo.reconocer()
                g = self.objetivo
                self.recogidos[g.categoria] += 1
                self._log(f"Recogido {g.clase} ({g.categoria}) -> depósito. "
                          f"Puntaje estimado en cajas: {self.puntaje_estimado}")
                self.vision.reset_tracks()
                self._ir(Estado.ESPERA_RESCAN)
            elif r == "FALLO":
                motivo = self._motivo_brazo()
                self.brazo.reconocer()
                g = self.objetivo
                if motivo:
                    self._log(f"Brazo: {motivo}")
                if self._intentos < c.intentos_por_grano:
                    self._log(f"Fallo al recoger #{g.track_id}: reintento {self._intentos + 1}")
                    self._iniciar_recogida(g)
                else:
                    self.fallidos += 1
                    self._bloqueados.append((g.cx, g.cy, g.diam_px))
                    self._log(f"Grano #{g.track_id} saltado tras {self._intentos} intentos")
                    self.vision.reset_tracks()
                    self._ir(Estado.ESPERA_RESCAN)
            elif self._en_estado() > c.t_max_recogida_s:
                self.brazo.cancelar()
                self.abortar("el brazo no respondió a tiempo")

        elif self.estado is Estado.ESPERA_RESCAN:
            if self._en_estado() >= c.t_espera_rescan_s:
                self._frame_min = self.vision.snapshot().frame_id + c.cuadros_min_rescan
                self._ir(Estado.ESCANEANDO)

    def _iniciar_recogida(self, g):
        self._intentos += 1
        if self.brazo.iniciar_recogida(g):
            self._log(f"Recogiendo {g.resumen()}")
            self._ir(Estado.RECOGIENDO)
        else:
            self._bloqueados.append((g.cx, g.cy, g.diam_px))
            motivo = self._motivo_brazo()
            self._log(f"El brazo rechazó #{g.track_id}" + (f": {motivo}" if motivo else ""))
            self._ir(Estado.ESPERA_RESCAN)

    def _motivo_brazo(self) -> str:
        try:
            return (self.brazo.ultimo_error() or "").splitlines()[0][:120]
        except Exception:
            return ""

    # ------------------------------------------------------------ HUD
    def status_text(self) -> str:
        if self.estado is Estado.INACTIVA:
            return "MISION    : inactiva  ([TAB] elegir ARBOL, [K] iniciar cosecha)"
        extra = ""
        if self.activa and self.objetivo is not None and self.estado is Estado.RECOGIENDO:
            extra = f" | objetivo {self.objetivo.clase}"
        return (f"MISION    : {self.estado.value} | árbol {self.arbol} | maduros={self.recogidos['MADURO']} "
                f"sobremaduros={self.recogidos['SOBREMADURO']} | fallidos={self.fallidos} | "
                f"pts estimados={self.puntaje_estimado}{extra}")
