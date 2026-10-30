"""Bordes de la cancha con los 4 sensores de línea (QTR-1A junto a cada rueda) y el freno de borde del Mega.

Primitivas (clase `Bordes`):
    hasta_linea(mover, máscara)   mover con el freno activo SOLO en esos sensores: el Mega frena en el mismo ciclo
                                  en que uno ve la línea y avisa ABORT ... BORDE
    tocar(...)                    tocar un borde con un PAR de sensores (atrás TI/TD, derecha DD/TD, adelante DI/DD).
                                  NUNCA avanza después del primer toque (la rueda ya está sobre la huincha): si
                                  tocó uno solo, se despega 3 cm, gira hacia el otro y vuelve a tocar (bisección)
                                  hasta que tocan juntos: así queda paralelo al borde sin salirse de la cancha
    alinear(...)                  = tocar(..., enderezar=True)

Rutinas:
    buscar_bordes(...)            (tools.ir borde) borde de atrás + borde derecho, midiendo blanco/negro y umbrales
    RecorridoBorde                (tools.navegacion, tecla F) desde una esquina, recorrer el rectángulo de la cancha
                                  con el borde a la DERECHA y volver a la esquina de partida

Recorrido (sentido antihorario, el borde siempre a la derecha). La esquina de partida: borde detrás y borde a la
derecha. Lado k = el borde que va a la derecha durante el tramo k; al final del tramo k el borde de adelante es el
lado k+1. En cada tramo:
    1. avanzar con el freno en DI/DD, de a `tramo_m`; entre trozos, cangrejo a la derecha hasta tocar el lado k
       (con DD/TD): corrige el rumbo (la deriva de las ruedas) y la distancia al borde;
    2. al tocar el borde de adelante: enderezarse con él (DI/DD), retroceder `sep_m`, tocar el lado k otra vez y
       separarse `sep_m`: queda en la esquina, a la misma distancia de los dos bordes;
    3. girar 90° a la izquierda (el borde de adelante pasa a la derecha).
Después de 4 tramos y 4 giros el robot está en la esquina de partida, mirando igual: lo que dicen la odometría y
la pose fusionada con el LiDAR comparado con la partida es su ERROR DE CIERRE (la verdad la dan los bordes).
Cada toque queda en el mapa (pose fusionada) y en el marco de la odometría: con ellos se ajustan las 4 rectas
de los bordes y se mide el rectángulo (largo, ancho, ángulos).

Ojo: los sensores están junto a las ruedas, así que al tocar un borde la rueda ya está sobre la línea (en la
cancha es falta). Es una prueba de taller. Las líneas interiores (beneficiadero, cultivo) también frenan.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field

import numpy as np

from cafebot.drivers.mega import NOMBRES_IR
from cafebot.navegacion.geometria import componer, invertir, relativo, transformar

DI, DD, TI, TD = 0, 1, 2, 3
# Giro (signo) que endereza el robot según qué sensor del par tocó primero (+ = antihorario)
SIGNOS = {"atras": {TI: -1, TD: +1}, "derecha": {DD: +1, TD: -1}, "adelante": {DI: +1, DD: -1}}
PARES = {"atras": (TI, TD), "derecha": (DD, TD), "adelante": (DI, DD)}


class Detenido(Exception):
    """Se pidió detener el recorrido."""


@dataclass
class Toque:
    sensor: int
    borde: int | None              # índice del lado de la cancha (recorrido) o None
    mega: np.ndarray               # punto del sensor en el marco de la odometría del Mega (m)
    mapa: np.ndarray | None        # en el marco del mapa (pose fusionada), si hay
    t: float = 0.0


@dataclass
class ResultadoToque:
    primero: int | None            # sensor que tocó primero (None = los dos juntos)
    extra_m: float | None          # recorrido extra hasta que tocó el otro (None = el otro no tocó)
    sep_m: float                   # distancia entre los dos sensores del par
    giro_deg: float | None         # giro que deja el robot paralelo al borde
    toques: dict = field(default_factory=dict)   # sensor -> EventoIR


class Bordes:
    def __init__(self, mega, cfg, log=print, pose_mapa_fn=None, cola=None):
        """pose_mapa_fn() -> pose fusionada (m, m, rad) AHORA, para ubicar los toques en el mapa (opcional)."""
        self.mega = mega
        self.cfg = cfg
        self.log = log
        self.pose_mapa_fn = pose_mapa_fn
        self.cola = cola if cola is not None else (mega.suscribir_ir() if hasattr(mega, "suscribir_ir")
                                                   else mega.ir_eventos)
        self.sensores = [np.array(p, float) for p in cfg.ir.posiciones]
        self.toques: list[Toque] = []
        self._parar = threading.Event()

    # ------------------------------------------------------------------ básico
    def cerrar(self):
        """Deja de recibir eventos IR (la cola propia)."""
        if hasattr(self.mega, "desuscribir_ir"):
            self.mega.desuscribir_ir(self.cola)

    def detener(self):
        self._parar.set()
        try:
            self.mega.parar()
        except Exception:
            pass

    def revisar(self):
        if self._parar.is_set():
            raise Detenido()

    def vaciar(self) -> list:
        evs = []
        while not self.cola.empty():
            evs.append(self.cola.get_nowait())
        return evs

    def lecturas(self, n=5) -> np.ndarray:
        """Promedio de n lecturas de los 4 sensores (robot quieto)."""
        return np.mean([self.mega.ir_estado().valores for _ in range(n)], axis=0)

    def pose_mega(self) -> np.ndarray:
        x, y, th = self.mega.pose()
        return np.array([x / 1000.0, y / 1000.0, th])

    def hasta_linea(self, mover, mascara: int):
        """Mueve con el freno de borde activo SOLO para los sensores de `mascara`. Devuelve (resultado,
        {sensor: EventoIR de su primera entrada a negro})."""
        self.revisar()
        self.vaciar()
        self.mega.seguridad_ir(True, mascara)
        try:
            r = mover()
        finally:
            self.mega.seguridad_ir(False)
        time.sleep(0.15)
        toque = {}
        for ev in self.vaciar():
            for k in range(4):
                if ev.subida & mascara & (1 << k) and k not in toque:
                    toque[k] = ev
        self.revisar()
        return r, toque

    def _registrar(self, toque: dict, borde):
        """Guarda dónde estaba cada sensor al tocar: en el marco del Mega y (si hay pose fusionada) en el mapa."""
        mega_ahora = self.pose_mega()
        mapa_ahora = self.pose_mapa_fn() if self.pose_mapa_fn else None
        for k, ev in toque.items():
            mega_ev = np.array([ev.x, ev.y, ev.th])
            p_mega = transformar(self.sensores[k][None, :], mega_ev)[0]
            p_mapa = None
            if mapa_ahora is not None:
                en_robot = transformar(self.sensores[k][None, :], relativo(mega_ahora, mega_ev))
                p_mapa = transformar(en_robot, mapa_ahora)[0]
            self.toques.append(Toque(k, borde, p_mega, p_mapa, ev.recibido))

    # ------------------------------------------------------------------ movimientos
    def mov(self, lado: str, mm: float, vel: float):
        """Orden de movimiento hacia `lado` (atras, adelante, derecha, izquierda) para hasta_linea."""
        to = mm / max(vel, 1.0) + 10
        return {"atras": lambda: self.mega.avanzar(-mm, vel, timeout=to),
                "adelante": lambda: self.mega.avanzar(mm, vel, timeout=to),
                "derecha": lambda: self.mega.cangrejo(-90, mm, vel, timeout=to),
                "izquierda": lambda: self.mega.cangrejo(90, mm, vel, timeout=to)}[lado]

    def mover(self, lado: str, mm: float, vel: float = 60.0):
        """Movimiento corto SIN freno de borde (despegarse de una línea)."""
        self.revisar()
        r = self.mov(lado, mm, vel)()
        self.revisar()
        return r

    def tocar(self, lado: str, max_mm: float, vel: float, nombre: str = "", borde=None, enderezar: bool = True,
              tol_deg: float = 0.7, despegar_mm: float = 30.0, paso_deg: float = 6.0,
              max_giro_deg: float = 35.0) -> ResultadoToque | None:
        """Ir hacia `lado` hasta tocar un borde con su par de sensores. None si no lo encontró.

        NUNCA sigue avanzando después del primer toque: el sensor está junto a la rueda, así que en ese momento la
        rueda ya está sobre la huincha (19 mm) y un poco más la saca de la cancha. Si toca uno solo y `enderezar`:
        se despega `despegar_mm`, gira hacia el lado del sensor que NO tocó y vuelve a tocar despacio; cuando toca
        primero el otro, se pasó: vuelve con la mitad del giro (bisección), hasta que los dos tocan juntos o el giro
        que falta es menor que `tol_deg`. giro_deg = lo que giró en total (YA aplicado)."""
        nombre = nombre or f"borde de {lado}"
        a, b = PARES[lado]
        mascara = (1 << a) | (1 << b)
        r, toque = self.hasta_linea(self.mov(lado, max_mm, vel), mascara)
        if r.ok or r.motivo != "BORDE":
            self.log(f"  {nombre}: NO se encontró ({'recorrió todo sin ver línea' if r.ok else r.motivo})")
            return None
        sep = float(np.linalg.norm(self.sensores[a] - self.sensores[b]))
        self._registrar(toque, borde)
        if len(toque) != 1:                                  # los dos juntos: ya está derecho
            return ResultadoToque(None, 0.0, sep, 0.0, toque)
        primero = next(iter(toque))
        if not enderezar:
            return ResultadoToque(primero, None, sep, None, toque)
        opuesto = {"atras": "adelante", "adelante": "atras", "derecha": "izquierda", "izquierda": "derecha"}[lado]
        signo, paso, total, ultimo, acotado = SIGNOS[lado][primero], paso_deg, 0.0, primero, False
        for _ in range(12):
            self.mover(opuesto, despegar_mm, 40)
            self.revisar()
            g = signo * paso                                 # hacia el sensor que no tocó
            self.mega.girar(g, timeout=10)
            total += g
            r, toque = self.hasta_linea(self.mov(lado, despegar_mm + 50, 30), mascara)
            if r.ok or r.motivo != "BORDE":
                self.log(f"  {nombre}: perdí el borde al enderezar ({'no lo tocó' if r.ok else r.motivo})")
                return None
            self._registrar(toque, borde)
            if len(toque) != 1:
                break                                        # los dos juntos
            k = next(iter(toque))
            if k != ultimo:                                  # se pasó: volver con la mitad
                signo, acotado = SIGNOS[lado][k], True
                paso /= 2
            elif acotado:
                paso /= 2
            else:
                paso = min(paso * 1.5, 12.0)                 # muy torcido: pasos más largos (la rueda no llega
            ultimo = k                                       # a la línea: se despegó 3 cm antes de girar)
            if acotado and paso < tol_deg:
                break
            if abs(total) > max_giro_deg:
                self.log(f"  {nombre}: más de {max_giro_deg:.0f}° torcido: no sigo enderezando")
                break
        if abs(total) > 0.05:
            self.log(f"  {nombre}: estaba torcido {abs(total):.1f}° (giré {total:+.1f}°)")
        return ResultadoToque(primero, None, sep, total, toque)

    def alinear(self, lado: str, max_mm: float, vel: float, despegar_mm: float = 30.0, nombre: str = "",
                borde=None, tol_deg: float = 0.7) -> ResultadoToque | None:
        """Tocar el borde y quedar paralelo a él (ver tocar)."""
        return self.tocar(lado, max_mm, vel, nombre, borde, enderezar=True, tol_deg=tol_deg,
                          despegar_mm=min(despegar_mm, 40.0))


# ---------------------------------------------------------------------------- tools.ir borde
def buscar_bordes(mega, cfg, vel=80.0, max_mm=1500.0, despegar_mm=40.0, alinear=True, log=print) -> dict:
    """Desde cualquier punto de la cancha: retrocede hasta el borde de ATRÁS (sensores TI/TD), se endereza con él,
    y va en cangrejo hacia la DERECHA hasta el borde derecho (DD/TD). Al tocar cada borde mide las lecturas sobre
    el negro, para comprobar los umbrales. Las ruedas quedan junto a la línea: solo para el taller.
    Devuelve un resumen con lo medido (blanco/negro por sensor, umbrales sugeridos, torcimiento)."""
    b = Bordes(mega, cfg, log)
    try:
        return _buscar_bordes(b, mega, vel, max_mm, despegar_mm, alinear, log)
    finally:
        b.cerrar()


def _buscar_bordes(b, mega, vel, max_mm, despegar_mm, alinear, log) -> dict:
    out = {"atras": None, "derecha": None, "torcido_atras_deg": None, "torcido_derecha_deg": None}
    est = mega.ir_estado()
    if est.mascara:
        log("¡Ojo! ya hay sensores sobre negro al empezar: "
            f"{', '.join(NOMBRES_IR[k] for k in range(4) if est.mascara & (1 << k))}. Alejar el robot de las líneas.")
        out["error"] = "sensores sobre negro al empezar"
        return out
    blanco = b.lecturas()
    negro = [None] * 4
    out["umbrales_antes"] = list(est.umbrales)
    log("Blanco (al partir): " + "  ".join(f"{NOMBRES_IR[k]} {blanco[k]:.0f}" for k in range(4))
        + f"   | umbrales {est.umbrales}, histéresis {est.histeresis}")

    def negro_de(par, avance):
        """Con los sensores recién llegados al borde: entrar 8 mm (al medio de la huincha) y leer los que
        están sobre negro (si el robot quedó torcido, el otro sensor sigue sobre la madera)."""
        b.mover(avance, 8, 30)
        v, est = b.lecturas(), mega.ir_estado()
        for k in par:
            if est.mascara & (1 << k):
                negro[k] = v[k] if negro[k] is None else (negro[k] + v[k]) / 2
        log("  sobre el borde: " + "  ".join(f"{NOMBRES_IR[k]} {v[k]:.0f}" + ("" if est.mascara & (1 << k) else
                                                                           " (no está sobre la línea)") for k in par))

    # 1) Atrás
    log("1) Retrocediendo hasta el borde de ATRÁS...")
    res = b.tocar("atras", max_mm, vel, "borde de atrás", enderezar=alinear)
    if res is None:
        out["error"] = "no encontró el borde de atrás"
        return out
    out["atras_inicial"] = out["atras"] = res
    out["torcido_inicial_deg"] = out["torcido_atras_deg"] = res.giro_deg
    if res.primero is not None:
        log(f"  tocó primero {NOMBRES_IR[res.primero]}"
            + (f"; enderezado girando {res.giro_deg:+.1f}°" if res.giro_deg is not None else " (sin enderezar)"))
    negro_de((TI, TD), "atras")
    b.mover("adelante", despegar_mm + 8)

    # 2) Derecha (cangrejo a −90°)
    log("2) Cangrejo a la DERECHA hasta el borde derecho...")
    res = b.tocar("derecha", max_mm, vel, "borde derecho", enderezar=False)     # sin girar: ya está derecho
    if res is None:
        out["error"] = "no encontró el borde derecho"
    else:
        out["derecha"] = res
        if res.primero is not None:
            log(f"  tocó primero {NOMBRES_IR[res.primero]} (ya enderezado con el de atrás: si pasa siempre, el "
                "cangrejo se desvía o los bordes no son perpendiculares)")
        negro_de((DD, TD), "derecha")
        b.mover("izquierda", despegar_mm + 8)

    # 3) Umbrales: punto medio entre el blanco y el negro medidos
    est = mega.ir_estado()
    out["blanco"] = [round(float(x)) for x in blanco]
    out["negro"] = [None if x is None else round(float(x)) for x in negro]
    out["sugeridos"] = list(est.umbrales)
    log("3) Umbrales (sensor: blanco / negro medidos -> umbral actual, sugerido):")
    for k in range(4):
        if negro[k] is None:
            log(f"  {NOMBRES_IR[k]}: blanco {blanco[k]:.0f} / negro sin medir (no tocó ningún borde) -> umbral {est.umbrales[k]}")
            continue
        c = negro[k] - blanco[k]
        sug = int(round((blanco[k] + negro[k]) / 2))
        out["sugeridos"][k] = sug if c >= 120 else est.umbrales[k]
        nota = "CONTRASTE BAJO: acercar el sensor al piso (3 mm)" if c < 300 else "ok"
        margen = min(est.umbrales[k] - est.histeresis / 2 - blanco[k], negro[k] - est.umbrales[k] - est.histeresis / 2)
        log(f"  {NOMBRES_IR[k]}: blanco {blanco[k]:.0f} / negro {negro[k]:.0f} (contraste {c:.0f}, {nota}) -> umbral "
            f"{est.umbrales[k]} (margen {margen:.0f}), sugerido {sug}")
    return out


# ---------------------------------------------------------------------------- recorrido del rectángulo
def ajustar_recta(pts: np.ndarray):
    """(punto, dirección unitaria) de la recta por mínimos cuadrados (componente principal)."""
    c = pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts - c)
    return c, vt[0]


def cortar(r1, r2):
    (p, u), (q, v) = r1, r2
    m = np.array([u, -v]).T
    if abs(np.linalg.det(m)) < 1e-9:
        return None
    t = np.linalg.solve(m, q - p)
    return p + t[0] * u


def rectangulo_desde_toques(toques: list[Toque], marco: str = "mapa") -> dict | None:
    """Recta de cada lado (0..3) con sus toques, y las 4 esquinas de la huincha (lado k ∩ lado k+1)."""
    rectas = {}
    for k in range(4):
        pts = [getattr(t, marco) for t in toques if t.borde == k and getattr(t, marco) is not None]
        if len(pts) >= 2:
            rectas[k] = ajustar_recta(np.array(pts))
    if len(rectas) < 4:
        return {"rectas": rectas, "esquinas": None}
    esq = [cortar(rectas[k], rectas[(k + 1) % 4]) for k in range(4)]
    if any(e is None for e in esq):
        return {"rectas": rectas, "esquinas": None}
    esq = np.array(esq)
    largos = [float(np.linalg.norm(esq[k] - esq[k - 1])) for k in range(4)]      # lado k: esquina k-1 -> esquina k
    angulos = []
    for k in range(4):
        u, v = rectas[k][1], rectas[(k + 1) % 4][1]
        angulos.append(math.degrees(math.acos(min(1.0, abs(float(u @ v))))))
    residuos = []
    for k in range(4):
        pts = np.array([getattr(t, marco) for t in toques if t.borde == k and getattr(t, marco) is not None])
        normal = np.array([-rectas[k][1][1], rectas[k][1][0]])
        residuos.append(float(np.abs((pts - rectas[k][0]) @ normal).max()))
    return {"rectas": rectas, "esquinas": esq, "largos": largos, "angulos": angulos, "residuo_max": max(residuos)}


class RecorridoBorde(Bordes):
    def __init__(self, mega, cfg, pose_mapa_fn=None, log=print, vel=120.0, sep_m=0.10, tramo_m=0.8,
                 max_lado_m=6.0, vel_toque=60.0):
        """sep_m: distancia del sensor al borde al recorrer y al girar (con 0,10 el centro queda a ~21 cm de cada
        borde y la rueda pasa a ~5 cm de la línea al girar 90°). tramo_m: cada cuánto se toca el borde derecho."""
        super().__init__(mega, cfg, log, pose_mapa_fn)
        self.vel, self.sep_mm, self.tramo_mm = vel, 1000 * sep_m, 1000 * tramo_m
        self.max_lado_mm, self.vel_toque = 1000 * max_lado_m, vel_toque
        self.estado = "listo"
        self.lado = None
        self.esquinas = []           # [{k, mega: pose, mapa: pose}]  0 = partida, 4 = vuelta completa
        self.recorridos = []         # mm recorridos (odometría) en cada tramo
        self.resultado = None
        self.activo = False
        self._hilo = None

    # ------------------------------------------------------------------ control
    def iniciar(self):
        self._parar.clear()
        self.activo = True
        self._hilo = threading.Thread(target=self._correr, name="recorrido-borde", daemon=True)
        self._hilo.start()

    def esperar(self, timeout=None) -> bool:
        if self._hilo:
            self._hilo.join(timeout)
        return not self.activo

    def _correr(self):
        try:
            self._recorrido()
        except Detenido:
            self.estado = "detenido"
            self.log("[borde] detenido")
        except Exception as e:                # el robot no debe quedar andando por un error de la rutina
            self.estado = f"ERROR: {e}"
            self.log(f"[borde] ERROR: {e}")
            try:
                self.mega.parar()
            except Exception:
                pass
        finally:
            try:
                self.mega.seguridad_ir(False)
            except Exception:
                pass
            self.cerrar()
            self.activo = False

    def _esquina(self, k: int):
        self.esquinas.append({"k": k, "mega": self.pose_mega(),
                              "mapa": None if self.pose_mapa_fn is None else np.asarray(self.pose_mapa_fn(), float)})

    def _derecha(self, k: int, max_mm: float, nombre: str, corregir: bool = True):
        """Tocar el lado k (a la derecha), separarse sep_m y (si `corregir`) enderezarse con él.
        False si no lo encontró. En las esquinas no se corrige: manda el borde de adelante, que es el que
        se sigue después del giro."""
        res = self.tocar("derecha", max_mm, self.vel_toque, nombre, borde=k, enderezar=corregir)
        if res is None:
            return False
        self.mover("izquierda", self.sep_mm)
        return True

    # ------------------------------------------------------------------ rutina
    def _recorrido(self):
        est = self.mega.ir_estado()
        if est.mascara:
            raise RuntimeError("hay sensores sobre negro al empezar: separar el robot de las líneas")
        # Esquina de partida: borde detrás (lado 3) y a la derecha (lado 0)
        self.estado = "esquina de partida: borde de atrás"
        if self.alinear("atras", 1500, self.vel_toque, self.sep_mm, "borde de atrás", borde=3) is None:
            raise RuntimeError("no encontré el borde de atrás (¿partió en una esquina, con el borde detrás?)")
        self.mover("adelante", self.sep_mm)
        self.estado = "esquina de partida: borde derecho"
        if not self._derecha(0, 1500, "borde derecho"):
            raise RuntimeError("no encontré el borde derecho")
        self._esquina(0)
        self.log("[borde] esquina de partida lista")

        for k in range(4):
            self.lado = k
            p0, hecho = self.pose_mega(), 0.0
            while True:
                self.estado = f"tramo {k + 1}/4: avanzando ({hecho / 1000:.2f} m)"
                trozo = min(self.tramo_mm, self.max_lado_mm - hecho)
                if trozo <= 0:
                    raise RuntimeError(f"tramo {k + 1}: {self.max_lado_mm / 1000:.1f} m sin encontrar el borde de adelante")
                r, toque = self.hasta_linea(self.mov("adelante", trozo, self.vel), (1 << DI) | (1 << DD))
                hecho = float(np.linalg.norm(self.pose_mega()[:2] - p0[:2])) * 1000
                if r.motivo == "BORDE":
                    break
                if not r.ok:
                    raise RuntimeError(f"tramo {k + 1}: el avance se detuvo ({r.motivo})")
                self.estado = f"tramo {k + 1}/4: tocando el borde derecho"
                if not self._derecha(k, self.sep_mm + 250, f"lado {k}"):
                    raise RuntimeError(f"tramo {k + 1}: perdí el borde derecho")
            self.recorridos.append(hecho)
            # Borde de adelante: otra pasada despacio para enderezarse con él (el toque rápido queda registrado)
            self.estado = f"tramo {k + 1}/4: enderezando con el borde de adelante"
            self._registrar(toque, (k + 1) % 4)
            self.mover("atras", 60)
            if self.alinear("adelante", 200, self.vel_toque, self.sep_mm, "borde de adelante", borde=(k + 1) % 4) is None:
                raise RuntimeError(f"tramo {k + 1}: perdí el borde de adelante")
            self.mover("atras", self.sep_mm)
            self.estado = f"esquina {k + 1}: borde derecho"
            if not self._derecha(k, self.sep_mm + 250, f"lado {k} (esquina)", corregir=False):
                raise RuntimeError(f"esquina {k + 1}: perdí el borde derecho")
            self.estado = f"esquina {k + 1}: girando 90°"
            self.revisar()
            self.mega.girar(90, timeout=15)
            self._esquina(k + 1)
            self.log(f"[borde] tramo {k + 1}: {hecho / 1000:.2f} m (odometría)")
        self.lado = None
        self.resultado = self.cierre()
        self.estado = "VUELTA COMPLETA | " + self.resumen()
        self.log("[borde] " + self.estado)

    # ------------------------------------------------------------------ resultados
    def cierre(self) -> dict:
        """Error de cierre (la esquina final ES la de partida) y el rectángulo medido con los toques."""
        out = {}
        if len(self.esquinas) >= 5:
            e0, e4 = self.esquinas[0], self.esquinas[4]
            # Lo que debería medirse: en la partida el centro está a (0,1115+sep) del borde de atrás y (0,1085+sep)
            # del derecho; en la llegada, al revés (tocó con los de adelante y la derecha): ~3 mm de diferencia
            sx, sy = abs(self.cfg.ir.posiciones[0][0]), abs(self.cfg.ir.posiciones[0][1])
            esperado = np.array([sy - sx, sx - sy, 0.0])
            for marco in ("mega", "mapa"):
                if e0[marco] is not None and e4[marco] is not None:
                    d = relativo(e0[marco], e4[marco]) - esperado
                    out[marco] = {"error_m": float(math.hypot(d[0], d[1])), "dx": float(d[0]), "dy": float(d[1]),
                                  "error_deg": float(math.degrees(d[2]))}
        for marco in ("mega", "mapa"):
            r = rectangulo_desde_toques(self.toques, marco)
            if r and r.get("esquinas") is not None:
                out["rect_" + marco] = r
        return out

    def resumen(self) -> str:
        r = self.resultado or {}
        partes = []
        rm = r.get("rect_mapa") or r.get("rect_mega")
        if rm:
            l = rm["largos"]
            partes.append(f"cancha {l[0]:.2f} × {l[1]:.2f} m (lados {', '.join(f'{x:.2f}' for x in l)}), "
                          f"ángulos {', '.join(f'{a:.1f}' for a in rm['angulos'])}°")
        for marco, nombre in (("mega", "odometría"), ("mapa", "LiDAR+odometría")):
            if marco in r:
                partes.append(f"cierre {nombre}: {100 * r[marco]['error_m']:.1f} cm, {r[marco]['error_deg']:+.1f}°")
        return " | ".join(partes) or "sin datos"

    def rectangulo_mega_en_mapa(self) -> np.ndarray | None:
        """Esquinas del rectángulo medido con la ODOMETRÍA, llevadas al mapa haciéndolas coincidir en la partida
        (así se ve cuánto se aleja la odometría sola)."""
        r = (self.resultado or {}).get("rect_mega")
        if not r or not self.esquinas or self.esquinas[0]["mapa"] is None:
            return None
        e0 = self.esquinas[0]
        T = componer(e0["mapa"], invertir(e0["mega"]))
        return transformar(r["esquinas"], T)
