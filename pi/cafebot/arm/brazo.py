"""Brazo REAL (5 XL430 + garra, por U2D2) con la interfaz que usa la misión (`mision.interfaces.Brazo`).

PERFIL DE CADA BRAZO (`pi/brazos/<nombre>.json`; se arma con `python3 -m tools.brazo`, ver docs/brazo.md):
ids de los servos, el CERO de cada articulación (pasos del encoder con el brazo recto hacia arriba), su SENTIDO,
sus LÍMITES, las POSTURAS con nombre y un historial de las calibraciones. Cada brazo se calibra por separado
(`--perfil`); la misión usa `brazo.perfil` de la config. Sin archivo se usan los valores del código antiguo
(legacy/codigo_brazo/main.py).

Convención de las articulaciones (la del URDF y de cinematica.py): 0° = cada tramo alineado con el anterior,
con el brazo recto hacia ARRIBA, la base mirando hacia ADELANTE del robot y la garra ABIERTA. Positivo =
hacia ADELANTE (base: hacia la IZQUIERDA del robot; garra: cerrando).

Coordenadas: la misión pide los granos en el sistema del ROBOT (x adelante, y izquierda, z desde el SUELO).
Aquí se pasan al plano del brazo:
    yaw = dirección horizontal desde el eje de la base hasta el grano
    r   = distancia horizontal desde ese eje           z_brazo = z - altura de la base del brazo
y se resuelven con la IK de mínimo esfuerzo (`cinematica.py`), dentro de los límites del perfil.
Con `banco=True` (brazo fuera del robot) el origen es la base del brazo: x adelante desde el eje de la base,
z hacia arriba desde el plano donde está apoyada.

Posturas con nombre (ángulos articulares, se ENSEÑAN moviendo el brazo a mano sin torque):
    trabajo               plegado delante (por defecto: la de encendido del código antiguo)
    descanso              plegado para navegar (por defecto: la de apagado del código antiguo)
    deposito_maduro       sobre la tolva de los maduros          ┐ hay que grabarlas:
    deposito_sobremaduro  sobre la tolva de los sobremaduros     ┘ python3 -m tools.brazo leer  (tecla G)
    transporte            (opcional) brazo PLEGADO y alto para girar la base con un grano sin barrer la cámara
                          ni lo que haya sobre el robot. Si existe, para ir a un depósito (y volver) el brazo
                          primero se pliega SIN girar la base, después gira la base plegado y recién ahí se
                          estira a la postura de destino (solo cuenta la forma: el ángulo de la base no se usa).

Secuencia de agarre (`recoger`):
    abrir → 6 cm antes del grano (en la dirección del brazo) → entrar en 2 pasos → cerrar
    → ¿la garra quedó abierta por el grano? (si cerró del todo: no había nada) → subir 3 cm → retroceder 8 cm
    → depósito de su categoría → abrir → postura de espera (descanso: plegado detrás del frente)
En cada paso se revisa que las articulaciones lleguen (error < error_max_deg) y que la carga no se dispare:
si algo choca, se abre la garra, se retrocede y se informa el fallo (la misión sigue con el siguiente).
`brazo.parar.set()` (ESC en las herramientas) detiene la secuencia después del movimiento en curso.
"""
from __future__ import annotations

import datetime
import json
import math
import pathlib
import threading
import time
import unicodedata
from dataclasses import dataclass, field

import numpy as np

from cafebot.arm import dxl
from cafebot.arm.cinematica import ModeloBrazo, directa, inversa, torques_gravedad
from cafebot.mision.interfaces import Brazo
from cafebot.mundo import cancha

ARTICULACIONES = ("base", "hombro", "codo1", "codo2", "muneca")   # en este orden; la garra va aparte
TODAS = ARTICULACIONES + ("garra",)
MOSTRAR = {"base": "base", "hombro": "hombro", "codo1": "codo1", "codo2": "codo2", "muneca": "muñeca",
           "garra": "garra"}
SENTIDO_POSITIVO = {"base": "hacia la IZQUIERDA del robot",
                    "hombro": "hacia ADELANTE", "codo1": "hacia ADELANTE", "codo2": "hacia ADELANTE",
                    "muneca": "hacia ADELANTE (la punta de la garra va hacia adelante)",
                    "garra": "CERRANDO los dedos"}
SENTIDO_NEGATIVO = {"base": "hacia la DERECHA del robot",
                    "hombro": "hacia ATRÁS", "codo1": "hacia ATRÁS", "codo2": "hacia ATRÁS",
                    "muneca": "hacia ATRÁS", "garra": "ABRIENDO los dedos"}
RAIZ_PI = pathlib.Path(__file__).resolve().parents[2]
PASOS_GRADO = dxl.PASOS_POR_REV / 360.0

# legacy/codigo_brazo/main.py: OFFSETS_ORTOGONAL / config_giro, y los límites del modelo (cinematica.py)
_LEGADO = {"base": dxl.BRAZO[0], "hombro": dxl.BRAZO[1], "codo1": dxl.BRAZO[2], "codo2": dxl.BRAZO[3],
           "muneca": dxl.BRAZO[4], "garra": dxl.GARRA}
_LIMITES = {"base": (-150.0, 150.0), "hombro": (-100.0, 100.0), "codo1": (-150.0, 150.0),
            "codo2": (-130.0, 130.0),          # tope de descanso del codo2: +129° (paso 78 en el código antiguo)
            "muneca": (-120.0, 120.0), "garra": (-10.0, 90.0)}
_POSTURAS_LEGADO = {"trabajo": {9: 3074, 8: 1400, 11: 2423, 7: 194, 5: 1587},     # OFFSETS_PRENDERSE
                    "descanso": {9: 3085, 8: 1787, 11: 1719, 7: 78, 5: 1801}}     # OFFSETS_APAGARSE


def nombre_articulacion(texto: str) -> str:
    """'Muñeca' -> 'muneca'. Error claro si no es una articulación."""
    t = unicodedata.normalize("NFKD", str(texto).strip().lower()).encode("ascii", "ignore").decode()
    if t not in TODAS:
        raise ValueError(f"'{texto}' no es una articulación ({', '.join(MOSTRAR[n] for n in TODAS)})")
    return t


def _dif_pasos(a, b) -> int:
    """a − b en pasos, llevado a ± media vuelta."""
    return ((int(a) - int(b) + 2048) % dxl.PASOS_POR_REV) - 2048


@dataclass
class PerfilBrazo:
    nombre: str = "principal"
    baudios: int = 1_000_000
    puerto: str = ""                                # vacío = el U2D2 de la config (puertos.u2d2)
    art: dict = field(default_factory=dict)         # articulación -> {id, centro, invertir, min_deg, max_deg}
    posturas: dict = field(default_factory=dict)    # nombre -> {articulación: grados}
    historial: list = field(default_factory=list)   # "fecha: qué se calibró"
    ruta: pathlib.Path | None = None

    # ------------------------------------------------------------------ archivo
    @staticmethod
    def de_fabrica(nombre="principal") -> "PerfilBrazo":
        art = {n: {"id": c.id, "centro": c.centro_pasos, "invertir": c.invertir,
                   "min_deg": _LIMITES[n][0], "max_deg": _LIMITES[n][1]} for n, c in _LEGADO.items()}
        p = PerfilBrazo(nombre, art=art)
        for nombre_p, tabla in _POSTURAS_LEGADO.items():
            p.posturas[nombre_p] = p.pasos_a_grados(tabla)
        return p

    @staticmethod
    def cargar(nombre="principal", carpeta=None, posturas_antiguas=None) -> "PerfilBrazo":
        """pi/brazos/<nombre>.json sobre los valores de fábrica. Si todavía no existe, se traen las posturas
        grabadas con la versión anterior (`posturas_antiguas`, claves = id del servo)."""
        carpeta = pathlib.Path(carpeta) if carpeta else RAIZ_PI / "brazos"
        p = PerfilBrazo.de_fabrica(nombre)
        p.ruta = carpeta / f"{nombre}.json"
        if p.ruta.exists():
            try:
                datos = json.loads(p.ruta.read_text())
                p.baudios = int(datos.get("baudios", p.baudios))
                p.puerto = str(datos.get("puerto", ""))
                for n, v in datos.get("articulaciones", {}).items():
                    p.art[nombre_articulacion(n)].update(v)
                for n, post in datos.get("posturas", {}).items():
                    p.posturas[n] = p.normalizar_postura(post)
                p.historial = list(datos.get("historial", []))
            except (ValueError, KeyError, TypeError, AttributeError) as e:
                # con un perfil roto NO se sigue con ceros de fábrica: el brazo se movería a cualquier lado
                raise ValueError(f"perfil del brazo con errores ({p.ruta}): {e}") from e
        elif posturas_antiguas is not None and pathlib.Path(posturas_antiguas).exists():
            try:
                for n, post in json.loads(pathlib.Path(posturas_antiguas).read_text()).items():
                    p.posturas[n] = p.normalizar_postura(post)
            except (OSError, ValueError, AttributeError):
                pass
        return p

    def guardar(self) -> pathlib.Path:
        self.ruta.parent.mkdir(parents=True, exist_ok=True)
        datos = {"baudios": self.baudios, "puerto": self.puerto, "articulaciones": self.art,
                 "posturas": self.posturas, "historial": self.historial[-60:]}
        tmp = self.ruta.with_suffix(".tmp")
        tmp.write_text(json.dumps(datos, indent=2, ensure_ascii=False) + "\n")
        tmp.replace(self.ruta)          # de una vez: un corte no deja el perfil a medias
        return self.ruta

    def anotar(self, texto: str) -> None:
        self.historial.append(f"{datetime.datetime.now():%Y-%m-%d %H:%M} {texto}")

    # ------------------------------------------------------------------ conversiones
    @property
    def ids(self) -> dict:
        return {n: int(self.art[n]["id"]) for n in TODAS}

    def nombre_de(self, sid: int) -> str:
        return next((n for n in TODAS if int(self.art[n]["id"]) == int(sid)), str(sid))

    def servos(self) -> list:
        """ConfigServo para el bus: las ganancias "cuidadosas" de dxl.py con los ids/ceros/sentidos de este brazo."""
        out = []
        for n in TODAS:
            base, a = _LEGADO[n], self.art[n]
            out.append(dxl.ConfigServo(int(a["id"]), p=base.p, i=base.i, d=base.d, pwm_limit=base.pwm_limit,
                                       invertir=bool(a["invertir"]), centro_pasos=int(a["centro"])))
        return out

    def pasos_a_grados(self, tabla: dict) -> dict:
        """{id: pasos} (como las tablas del código antiguo) -> {articulación: grados}."""
        out = {}
        for n in ARTICULACIONES:
            a = self.art[n]
            if int(a["id"]) in tabla:
                s = -1 if a["invertir"] else 1
                out[n] = round(s * _dif_pasos(tabla[int(a["id"])], a["centro"]) / PASOS_GRADO, 2)
        return out

    def normalizar_postura(self, post: dict) -> dict:
        """Acepta claves por nombre ("hombro") o por id de servo ("8", formato anterior)."""
        out = {}
        for k, v in post.items():
            n = self.nombre_de(int(k)) if str(k).isdigit() else nombre_articulacion(k)
            if n in ARTICULACIONES:
                out[n] = float(v)
        return out

    def limites_rad(self) -> tuple:
        """Límites de hombro, codo1, codo2 y muñeca para la IK."""
        return tuple((math.radians(self.art[n]["min_deg"]), math.radians(self.art[n]["max_deg"]))
                     for n in ARTICULACIONES[1:])

    # ------------------------------------------------------------------ calibración (solo el perfil)
    def fijar_cero(self, n: str, pasos: int, rearmado: bool = False) -> float:
        """La posición `pasos` pasa a ser el 0° de `n`. Devuelve cuánto se corrió el cero (°).
        Las posturas y los límites (que son poses FÍSICAS: se enseñaron o se midieron) se corrigen para quedar
        físicamente iguales. `rearmado=True` (se desmontó o cambió el servo: su encoder ya no calza con la pieza):
        no se corrigen, porque su valor en grados sigue valiendo respecto del cero nuevo."""
        a = self.art[n]
        s = -1 if a["invertir"] else 1
        delta = s * _dif_pasos(pasos, a["centro"]) / PASOS_GRADO
        a["centro"] = int(pasos) % dxl.PASOS_POR_REV
        if not rearmado and delta:
            for post in self.posturas.values():
                if n in post:
                    post[n] = round(post[n] - delta, 2)
            a["min_deg"], a["max_deg"] = round(a["min_deg"] - delta, 1), round(a["max_deg"] - delta, 1)
        return delta

    def invertir(self, n: str) -> bool:
        """Cambia el sentido de `n`. Las posturas y los límites quedan físicamente iguales (cambia el signo)."""
        a = self.art[n]
        a["invertir"] = not bool(a["invertir"])
        for post in self.posturas.values():
            if n in post:
                post[n] = -post[n]
        a["min_deg"], a["max_deg"] = -a["max_deg"], -a["min_deg"]
        return a["invertir"]

    def asignar_ids(self, ids: dict) -> None:
        """{articulación: id}. Error si dos articulaciones quedarían con el mismo id."""
        nuevo = dict(self.ids)
        nuevo.update({nombre_articulacion(n): int(v) for n, v in ids.items()})
        repetidos = {v for v in nuevo.values() if list(nuevo.values()).count(v) > 1}
        if repetidos:
            raise ValueError(f"ids repetidos: {sorted(repetidos)} ({nuevo})")
        for n, v in nuevo.items():
            self.art[n]["id"] = v


class BrazoXL430(Brazo):
    def __init__(self, cfg, puerto: str | None = None, bus=None, log=print, perfil: PerfilBrazo | None = None,
                 banco: bool = False):
        self.cfg = cfg
        self.cb = cfg.brazo
        self.cm = cfg.mision
        self.perfil = perfil or PerfilBrazo.cargar(self.cb.perfil, posturas_antiguas=RAIZ_PI / self.cb.archivo_posturas)
        self.puerto = puerto or self.perfil.puerto or cfg.puertos.u2d2
        self.bus = bus
        self.log = log
        self.banco = banco
        self.base = np.zeros(3) if banco else np.array([self.cm.brazo_x, self.cm.brazo_y, self.cm.brazo_z])
        self.depositos = {cancha.MADURO: 0, cancha.SOBREMADURO: 0, cancha.VERDE: 0}
        self.listo = False
        self.ultimo_motivo = ""
        self.parar = threading.Event()   # ESC: no se manda ningún movimiento más hasta la próxima orden
        self._salud = []           # última lectura (estado() NO toca el bus: lo llama la interfaz desde otro hilo)
        self._actualizar_perfil()

    def _actualizar_perfil(self):
        self.id = self.perfil.ids
        self.IDS = [self.id[n] for n in ARTICULACIONES]
        self.ID_GARRA = self.id["garra"]
        self.modelo = ModeloBrazo(L4=self.cb.garra_largo_m, limites=self.perfil.limites_rad())

    @property
    def posturas(self) -> dict:
        return self.perfil.posturas

    def mostrar(self, sid: int) -> str:
        n = self.perfil.nombre_de(sid)
        return MOSTRAR.get(n, n)

    # ------------------------------------------------------------------ conexión
    def conectar(self) -> dict:
        """Abre el bus, verifica que estén los 6 servos y escribe la configuración cuidadosa."""
        if self.bus is None:
            self.bus = dxl.BusDXL(self.puerto, self.perfil.baudios, self.perfil.servos())
        vivos = self.bus.ping()
        faltan = [s for s in self.IDS + [self.ID_GARRA] if s not in vivos]
        if faltan:
            raise IOError("no responden los servos " + ", ".join(f"{s} ({self.mostrar(s)})" for s in faltan)
                          + f" a {self.perfil.baudios} baudios (python3 -m tools.brazo escanear muestra cuáles hay)")
        apagados = self.bus.configurar()    # configuración + ceros normalizados
        if apagados:
            self.log(f"[brazo] se escribió la configuración (torque apagado en {apagados})")
        self.bus.recuperar_errores()
        return vivos

    def recargar_bus(self):
        """Después de cambiar ceros, sentidos o ids en el perfil: el bus toma los valores nuevos."""
        self._actualizar_perfil()
        if self.bus is not None:
            self.bus.servos = {c.id: c for c in self.perfil.servos()}
            self.bus.normalizar_centros()

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
                    self.log(f"[brazo] falta la postura '{nombre}': grabarla con  python3 -m tools.brazo leer  (tecla G)")
                    return False
            self._salud = self.bus.salud()
            self.bus.torque(True)
            self.garra(abrir=True, esperar=False)
            self.ir_plegado(self.cb.postura_espera)
            self.listo = True
            return True
        except Exception as e:
            self.log(f"[brazo] error al preparar: {e}")
            return False

    def cerrar(self):
        if self.bus is not None:
            self.bus.cerrar()

    # ------------------------------------------------------------------ calibración
    def angulos_nombre(self) -> dict:
        """{articulación: grados} actuales (incluye la garra)."""
        ang = self.bus.angulos([self.id[n] for n in TODAS])
        return {n: math.degrees(ang[self.id[n]]) for n in TODAS}

    def pasos_nombre(self) -> dict:
        p = self.bus.pasos([self.id[n] for n in TODAS])
        return {n: p[self.id[n]] for n in TODAS}

    def cero(self, articulaciones=TODAS, rearmado: bool = False) -> dict:
        """La posición ACTUAL pasa a ser el 0° de esas articulaciones (brazo recto hacia arriba, base mirando
        adelante, garra abierta). Se guarda en el perfil. Devuelve {articulación: (cero viejo, nuevo, Δ°)}."""
        pasos = self.pasos_nombre()
        hecho = {}
        for n in articulaciones:
            viejo = int(self.perfil.art[n]["centro"])
            delta = self.perfil.fijar_cero(n, pasos[n], rearmado)
            hecho[n] = (viejo, int(self.perfil.art[n]["centro"]), delta)
        self.perfil.anotar("cero " + ", ".join(f"{n} {v}→{c} ({d:+.1f}°)" for n, (v, c, d) in hecho.items())
                           + (" [rearmado]" if rearmado else ""))
        self.perfil.guardar()
        self.recargar_bus()
        return hecho

    def invertir_sentido(self, articulacion: str) -> bool:
        inv = self.perfil.invertir(articulacion)
        self.perfil.anotar(f"sentido de {articulacion} invertido (invertir={inv})")
        self.perfil.guardar()
        self.recargar_bus()
        return inv

    def fijar_limites(self, limites_deg: dict) -> None:
        """{articulación: (mín, máx)} en grados (p. ej. los extremos recorridos a mano, con margen)."""
        for n, (lo, hi) in limites_deg.items():
            if hi - lo < 5:
                raise ValueError(f"{n}: rango {lo:.0f}..{hi:.0f}° demasiado chico")
        for n, (lo, hi) in limites_deg.items():
            self.perfil.art[n]["min_deg"], self.perfil.art[n]["max_deg"] = round(lo, 1), round(hi, 1)
        self.perfil.anotar("límites " + ", ".join(f"{n} {lo:+.0f}..{hi:+.0f}°" for n, (lo, hi) in limites_deg.items()))
        self.perfil.guardar()
        self._actualizar_perfil()

    def mover_articulacion(self, articulacion: str, grados: float, t: float = 2.0) -> bool:
        """Mueve UNA articulación (las demás quedan donde están). La primera prueba con torque."""
        a = self.perfil.art[articulacion]
        if not a["min_deg"] - 1e-6 <= grados <= a["max_deg"] + 1e-6:
            self.ultimo_motivo = (f"{grados:+.0f}° está fuera de los límites de {MOSTRAR[articulacion]} "
                                  f"({a['min_deg']:+.0f}..{a['max_deg']:+.0f}°)")
            return False
        return self._mover({self.id[articulacion]: math.radians(grados)}, t, MOSTRAR[articulacion])

    # ------------------------------------------------------------------ posturas
    def grabar_postura(self, nombre: str) -> dict:
        """Guarda los ángulos ACTUALES de las 5 articulaciones (sin la garra) con ese nombre."""
        ang = self.angulos_nombre()
        post = {n: round(ang[n], 2) for n in ARTICULACIONES}
        self.perfil.posturas[nombre] = post
        self.perfil.anotar(f"postura '{nombre}' grabada")
        self.perfil.guardar()
        return post

    def ir(self, nombre: str, t: float | None = None) -> bool:
        if nombre not in self.posturas:
            raise KeyError(f"no existe la postura '{nombre}' (grábala con: python3 -m tools.brazo leer, tecla G)")
        obj = {self.id[n]: math.radians(v) for n, v in self.posturas[nombre].items()}
        return self._mover(obj, t or self.cb.t_mov_s, f"postura {nombre}")

    def ir_plegado(self, nombre: str) -> bool:
        """A una postura pasando por 'transporte' si está grabada: 1) se pliega con la base donde está, 2) gira la
        base plegado, 3) se estira a la postura. Sin 'transporte' (o si la base casi no gira): directo."""
        if "transporte" not in self.posturas or nombre == "transporte":
            return self.ir(nombre)
        fin = self.posturas[nombre]
        base_ahora = math.degrees(self.bus.angulos([self.id["base"]])[self.id["base"]])
        if abs(fin["base"] - base_ahora) < 20:
            return self.ir(nombre)
        plegado = {self.id[n]: math.radians(v) for n, v in self.posturas["transporte"].items() if n != "base"}
        plegado[self.id["base"]] = math.radians(base_ahora)
        if not self._mover(plegado, self.cb.t_mov_s, "plegar (transporte)"):
            return False
        plegado[self.id["base"]] = math.radians(fin["base"])
        t_giro = self.cb.t_mov_s * max(1.0, abs(fin["base"] - base_ahora) / 120)
        if not self._mover(plegado, t_giro, "girar plegado (transporte)"):
            return False
        return self.ir(nombre)

    def garra(self, abrir: bool | None = None, angulo_deg: float | None = None, esperar: bool = True) -> float:
        """Abre / cierra (o va a un ángulo). Devuelve el ángulo alcanzado (°): con un grano entre los dedos
        la garra se detiene antes de cerrar del todo."""
        if angulo_deg is None:
            angulo_deg = self.cb.garra_abierta_deg if abrir else self.cb.garra_cerrada_deg
        self.bus.mover({self.ID_GARRA: math.radians(angulo_deg)}, 0.5)
        if not esperar:
            return angulo_deg
        time.sleep(0.6)
        self.bus.esperar_quietos(1.0)
        return math.degrees(self.bus.angulos([self.ID_GARRA])[self.ID_GARRA])

    # ------------------------------------------------------------------ cinemática en el sistema del robot
    def a_plano(self, p_robot) -> tuple[float, float, float]:
        """(dirección horizontal [rad, en el robot], r, z sobre la base del brazo)."""
        d = np.asarray(p_robot, float) - self.base
        return math.atan2(d[1], d[0]), math.hypot(d[0], d[1]), float(d[2])

    def yaw_articular(self, direccion: float) -> float:
        return math.radians(self.cb.yaw0_deg) + self.cb.yaw_signo * direccion

    def directa_robot(self, ang: dict) -> tuple[np.ndarray, float]:
        """Ángulos articulares (rad, por id) -> (punto de la garra en el robot, pitch en °)."""
        i = self.id
        direccion = (ang[i["base"]] - math.radians(self.cb.yaw0_deg)) * self.cb.yaw_signo
        r, z, pitch = directa(self.modelo, [ang[i["hombro"]], ang[i["codo1"]], ang[i["codo2"]], ang[i["muneca"]]])
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
        yaw = self.yaw_articular(direccion)
        lo, hi = self.perfil.art["base"]["min_deg"], self.perfil.art["base"]["max_deg"]
        if not math.radians(lo) <= yaw <= math.radians(hi):
            return None
        i = self.id
        for c in candidatos:
            s = inversa(self.modelo, r, z, c)
            if s is not None:
                q = s.q
                return {i["base"]: yaw, i["hombro"]: q[0], i["codo1"]: q[1], i["codo2"]: q[2], i["muneca"]: q[3]}, s
        return None

    def plan_agarre(self, p_robot):
        """Puntos de la secuencia (nombre, ángulos, duración) con la MISMA inclinación de garra, o None."""
        sol = self.ik(p_robot)
        if sol is None:
            return None
        pitch = math.degrees(sol[1].pitch)
        direccion, r, z = self.a_plano(p_robot)
        u = np.array([math.cos(direccion), math.sin(direccion), 0.0])     # entrar en la dirección del brazo
        up = np.array([0.0, 0.0, 1.0])
        p = np.asarray(p_robot, float)
        # Nunca por encima del propio robot: si el grano está cerca del eje de la base, la aproximación y el retiro
        # se acortan (si no, el punto quedaría DETRÁS del eje y la base giraría 180° en medio de la secuencia).
        libre = r - self.cb.r_min_m
        a, rt = min(self.cb.aproximacion_m, libre), max(0.0, min(self.cb.retirar_m, libre))
        if a < 0.015:
            return None

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
        if self.parar.is_set():
            raise RuntimeError(f"detenido antes de: {que}")
        self.bus.mover(obj, t)
        time.sleep(t)
        self.bus.esperar_quietos(1.5)
        err = self.bus.error_seguimiento_deg(obj)
        self._salud = self.bus.salud()
        carga = max(abs(s.carga_pct) for s in self._salud if s.id != self.ID_GARRA)
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
            ang = self.bus.angulos(self.IDS)
            p, pitch = self.directa_robot(ang)
            rt = max(0.0, min(self.cb.retirar_m, self.a_plano(p)[1] - self.cb.r_min_m))   # sin cruzar el eje de la base
            atras = p - rt * np.array([math.cos(direccion), math.sin(direccion), 0.0])
            s = self.ik(atras, pitch) or self.ik(atras)
            if s is not None and not self.parar.is_set():
                self.bus.mover(s[0], self.cb.t_mov_s)
                time.sleep(self.cb.t_mov_s)
            self.ir_plegado(self.cb.postura_espera)
        except Exception as e:
            self.log(f"[brazo] al abortar: {e}")
        self.ultimo_motivo = motivo

    def recoger(self, p_robot, categoria: str, depositar: bool = True) -> bool:
        """Saca UN grano en p_robot y lo deja en el depósito de su categoría. True si quedó en el robot."""
        if self.bus is None:
            return False
        destino = "deposito_maduro" if categoria == cancha.MADURO else "deposito_sobremaduro"
        if depositar and destino not in self.posturas:
            # nunca sacar un grano sin saber dónde dejarlo: se caería (−7)
            self.ultimo_motivo = f"falta la postura '{destino}'"
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
            if not self.ir_plegado(destino) and not self.ir(destino):
                # como antes: casi siempre quedó cerca de la tolva (error o carga algo sobre el límite); guardarlo en
                # la garra sería peor (se soltaría al abrir para el próximo grano, delante del árbol)
                self.log(f"[brazo] {self.ultimo_motivo} (yendo al depósito con el grano): se suelta igual")
            self.garra(abrir=True)
            time.sleep(0.3)                    # que el grano caiga antes de mover el brazo
            self.depositos[categoria] = self.depositos.get(categoria, 0) + 1
            self.ir_plegado(self.cb.postura_espera)
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
            self.ir_plegado("descanso")
            if self.cb.descanso_sin_torque:
                time.sleep(0.3)
                self.bus.torque(False, self.IDS)
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
        i = self.id
        q = np.array([ang[i["hombro"]], ang[i["codo1"]], ang[i["codo2"]], ang[i["muneca"]]])
        return 100 * np.abs(torques_gravedad(self.modelo, q)) / self.modelo.tau_max
