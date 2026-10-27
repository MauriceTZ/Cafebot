"""Máquina de estados de una ronda completa (8 minutos).

INICIO -> LEER_CANCHA -> IR_A_ARBOL -> ESCANEAR_ARBOL -> TOMAR_GRANO (repite)
       -> IR_A_CAJAS -> IR_A_ARBOL (siguiente) ... -> FIN

La ronda no sabe si habla con el robot real o con el simulador: recibe
`esp` (ESP32Simulada o ESP32Serial), `percepcion` y `reloj`.

La percepción debe tener estos métodos:
  leer_cancha() -> Sorteo
  escanear_arbol(arbol) -> {posicion: clase}   (clases de clasificador.py)
  ajuste_fino() -> mm a corregir el carro, o None si no ve esfera
  soporte_vacio(posicion) -> bool
  grano_en_garra() -> bool

Para probarla: python simulador.py
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from clasificador import CATEGORIA, FILAS
from esp32 import ErrorRobot, duracion_estimada

# ---------------------------------------------------------------- mapa
# Origen: esquina del beneficiadero. +y apunta hacia los árboles.
# rumbo en grados: 90 = mirando a los árboles, 270 = mirando al beneficiadero.
# TODAS las coordenadas son PROVISIONALES: medirlas en el plano oficial.
SECTORES = {i: (356 + 712 * (i - 1), 350) for i in range(1, 6)}
ARBOLES = {1: (600, 2750), 2: (1800, 2750), 3: (3000, 2750)}
Y_ANTES_ARBOL = 2300      # punto de llegada frente al árbol
Y_ANTES_CAJA = 1000       # punto de llegada frente a una caja
DIST_ARBOL_MM = 150       # APPROACH: distancia final al árbol
DIST_CAJA_MM = 60         # APPROACH: distancia final al borde de la caja
RETROCESO_MM = 200        # retroceder antes de girar, para no golpear nada

# Separación entre soportes a lo largo de una fila (PROVISIONAL, pedir a diseño)
SEPARACION_SOPORTES_MM = 70

# ---------------------------------------------------------------- estrategia
DURACION_RONDA_S = 480
MARGEN_S = 20              # nunca apurar los últimos segundos
CAPACIDAD = {"RIPE": 8, "OVERRIPE": 8}   # granos por compartimento (pedir a diseño)
MAX_CORRECCION_MM = 25     # si el ajuste fino pide más, algo anda mal: no tocar
REINTENTOS_AGARRE = 1
TIEMPO_ESCANEO_S = 2.0     # estimación de visión para planificar
TIEMPO_VISION_GRANO_S = 0.8

COMPARTIMENTO = {"MADURO": "RIPE", "SOBREMADURO": "OVERRIPE"}
CAJA = {"RIPE": "ROJA", "OVERRIPE": "AZUL"}
ORDEN_FILAS = ["baja", "media", "alta"]


@dataclass
class Sorteo:
    sector_partida: int
    sector_roja: int
    sector_azul: int
    piscinas: list = field(default_factory=list)  # posiciones 1..5 ocupadas


def x_soporte(posicion: str) -> float:
    """Posición del carro lateral (mm, 0 = centro) para un soporte."""
    fila, i = posicion.split("-")
    n = FILAS[fila]
    return (int(i) - 1 - (n - 1) / 2) * SEPARACION_SOPORTES_MM


def angulo_giro(desde: float, hacia: float) -> float:
    return (hacia - desde + 180) % 360 - 180


class Ronda:
    def __init__(self, esp, percepcion, reloj, log=print):
        self.esp = esp
        self.percepcion = percepcion
        self.reloj = reloj
        self._log = log
        self.estado = "INICIO"
        self.sorteo = None
        self.x = self.y = 0.0
        self.rumbo = 90.0
        self.arboles_pendientes = []
        self.arbol = None
        self.objetivos = []           # [(posicion, categoria)]
        self.carga = {"RIPE": 0, "OVERRIPE": 0}
        self.carro_mm = 0.0
        self.estadisticas = {"tomados": 0, "fallidos": 0, "caidos": 0, "omitidos_no_se": 0}

    # ------------------------------------------------------------ utilidades

    def log(self, mensaje: str):
        self._log(f"[{self.reloj.ahora():6.1f}s] {self.estado:14s} {mensaje}")

    def cmd(self, texto: str, timeout: float = 15.0) -> str:
        r = self.esp.comando(texto, timeout)
        if not r.ok:
            raise ErrorRobot(f"{texto} -> ERR {r.datos}")
        return r.datos

    def restante(self) -> float:
        return DURACION_RONDA_S - MARGEN_S - self.reloj.ahora()

    def ruta(self, x1, y1, rumbo1) -> list:
        """Ruta en L desde la pose actual: primero en y, luego en x, luego
        girar al rumbo final. TODO: esquivar piscinas según self.sorteo."""
        cmds, rumbo = [], self.rumbo
        for eje, delta in (("y", y1 - self.y), ("x", x1 - self.x)):
            if abs(delta) < 5:
                continue
            if eje == "y":
                destino = 90.0 if delta > 0 else 270.0
            else:
                destino = 0.0 if delta > 0 else 180.0
            giro = angulo_giro(rumbo, destino)
            if abs(giro) > 0.5:
                cmds.append(f"TURN {giro:.0f}")
            cmds.append(f"MOVE {abs(delta):.0f}")
            rumbo = destino
        giro = angulo_giro(rumbo, rumbo1)
        if abs(giro) > 0.5:
            cmds.append(f"TURN {giro:.0f}")
        return cmds

    def mover(self, mm: float):
        """MOVE que además actualiza la pose estimada (nunca usar MOVE suelto)."""
        self.cmd(f"MOVE {mm:.0f}")
        self.x += mm * math.cos(math.radians(self.rumbo))
        self.y += mm * math.sin(math.radians(self.rumbo))

    def ejecutar_ruta(self, x1, y1, rumbo1):
        for c in self.ruta(x1, y1, rumbo1):
            self.cmd(c)
        self.x, self.y, self.rumbo = x1, y1, rumbo1

    def tiempo(self, cmds) -> float:
        return sum(duracion_estimada(c) for c in cmds)

    def tiempo_grano(self) -> float:
        seq = ["LIFT X", "SLIDE 0", "SLIDE 0", "EXTEND", "GRIP", "RETRACT",
               "FLAP RIPE", "RELEASE"]
        return self.tiempo(seq) + TIEMPO_VISION_GRANO_S

    def tiempo_entrega(self) -> float:
        """Estimación del tiempo para vaciar lo que se lleva, desde aquí."""
        x, y, rumbo = self.x, self.y, self.rumbo
        total = duracion_estimada(f"MOVE {RETROCESO_MM}") if self.arbol else 0.0
        for comp in ("RIPE", "OVERRIPE"):
            if self.carga[comp] == 0:
                continue
            sx, _ = SECTORES[self.sector_caja(comp)]
            total += self.tiempo(self.ruta(sx, Y_ANTES_CAJA, 270.0))
            total += duracion_estimada("APPROACH") + duracion_estimada("DUMP")
            total += duracion_estimada(f"MOVE {DIST_CAJA_MM}")
            self.x, self.y, self.rumbo = sx, Y_ANTES_CAJA, 270.0
        self.x, self.y, self.rumbo = x, y, rumbo
        return total

    def sector_caja(self, comp: str) -> int:
        return self.sorteo.sector_roja if comp == "RIPE" else self.sorteo.sector_azul

    def lleva_granos(self) -> bool:
        return self.carga["RIPE"] + self.carga["OVERRIPE"] > 0

    # ------------------------------------------------------------ estados

    def ejecutar(self):
        estados = {
            "INICIO": self.e_inicio,
            "LEER_CANCHA": self.e_leer_cancha,
            "IR_A_ARBOL": self.e_ir_a_arbol,
            "ESCANEAR_ARBOL": self.e_escanear_arbol,
            "TOMAR_GRANO": self.e_tomar_grano,
            "IR_A_CAJAS": self.e_ir_a_cajas,
        }
        try:
            while self.estado != "FIN":
                siguiente = estados[self.estado]()
                if siguiente != self.estado:
                    self.log(f"-> {siguiente}")
                self.estado = siguiente
        except ErrorRobot as e:
            # Conservador: ante cualquier error del robot, detenerse. Los
            # granos que lleve adentro igual suman (+3 / +1).
            self.log(f"ERROR: {e}. Deteniendo.")
            self.esp.comando("STOP")
            self.estado = "FIN"
        self.log(f"Fin. Carga a bordo: {self.carga}. {self.estadisticas}")

    def e_inicio(self):
        self.log("Botón presionado, empieza la ronda")
        return "LEER_CANCHA"

    def e_leer_cancha(self):
        self.sorteo = self.percepcion.leer_cancha()
        self.x, self.y = SECTORES[self.sorteo.sector_partida]
        self.rumbo = 90.0
        self.log(f"Sorteo: {self.sorteo}")
        # Primero el árbol más cercano a la partida.
        self.arboles_pendientes = sorted(ARBOLES, key=lambda a: abs(ARBOLES[a][0] - self.x))
        return "IR_A_ARBOL"

    def e_ir_a_arbol(self):
        if not self.arboles_pendientes:
            return "IR_A_CAJAS" if self.lleva_granos() else "FIN"
        arbol = self.arboles_pendientes[0]
        ax, _ = ARBOLES[arbol]
        ida = self.ruta(ax, Y_ANTES_ARBOL, 90.0)
        # ¿Alcanza para llegar, escanear, tomar al menos 2 granos y entregarlos?
        necesario = (self.tiempo(ida) + duracion_estimada("APPROACH") + TIEMPO_ESCANEO_S
                     + 2 * self.tiempo_grano() + 60)
        if self.restante() < necesario:
            self.log(f"No alcanza el tiempo para el árbol {arbol} "
                     f"(quedan {self.restante():.0f}s, se necesitan ~{necesario:.0f}s)")
            return "IR_A_CAJAS" if self.lleva_granos() else "FIN"
        self.log(f"Yendo al árbol {arbol}")
        self.ejecutar_ruta(ax, Y_ANTES_ARBOL, 90.0)
        self.cmd(f"APPROACH {DIST_ARBOL_MM}")
        # TODO: APPROACH avanza una distancia que no conocemos de antemano;
        # con la ESP32 real, leer ODOM aquí y actualizar self.x / self.y.
        self.arbol = arbol
        return "ESCANEAR_ARBOL"

    def e_escanear_arbol(self):
        clases = self.percepcion.escanear_arbol(self.arbol)
        objetivos = []
        for pos, clase in clases.items():
            cat = CATEGORIA[clase]
            if cat in COMPARTIMENTO:
                objetivos.append((pos, cat))
            elif cat == "DESCONOCIDO":
                self.estadisticas["omitidos_no_se"] += 1
        # Fila por fila, el carro en zigzag para no ir y volver.
        def clave(item):
            fila, i = item[0].split("-")
            nf = ORDEN_FILAS.index(fila)
            return (nf, int(i) if nf % 2 == 0 else -int(i))
        self.objetivos = sorted(objetivos, key=clave)
        resumen = {c: sum(1 for v in clases.values() if v == c) for c in set(clases.values())}
        self.log(f"Árbol {self.arbol}: {resumen} -> {len(self.objetivos)} objetivos")
        return "TOMAR_GRANO"

    def e_tomar_grano(self):
        # Saltar los que ya no caben.
        while self.objetivos and self.carga[COMPARTIMENTO[self.objetivos[0][1]]] >= \
                CAPACIDAD[COMPARTIMENTO[self.objetivos[0][1]]]:
            self.log(f"Compartimento lleno, se omite {self.objetivos.pop(0)[0]}")
        if not self.objetivos:
            self.arboles_pendientes.remove(self.arbol)
            return "IR_A_CAJAS" if self.lleva_granos() else self.salir_del_arbol("IR_A_ARBOL")
        # ¿Alcanza para tomar uno más Y entregar todo?
        if self.restante() < self.tiempo_grano() + self.tiempo_entrega() + 5:
            self.log("Poco tiempo: dejo de recolectar")
            return "IR_A_CAJAS"
        pos, cat = self.objetivos.pop(0)
        self.tomar(pos, cat)
        return "TOMAR_GRANO"

    def tomar(self, pos: str, cat: str):
        fila = pos.split("-")[0]
        comp = COMPARTIMENTO[cat]
        self.cmd(f"LIFT {fila.upper()}")
        self.mover_carro(x_soporte(pos))
        for intento in range(1 + REINTENTOS_AGARRE):
            correccion = self.percepcion.ajuste_fino()
            if correccion is None or abs(correccion) > MAX_CORRECCION_MM:
                self.log(f"{pos}: el ajuste fino no ve bien la esfera ({correccion}), se omite")
                self.estadisticas["fallidos"] += 1
                return
            self.mover_carro(self.carro_mm + correccion)
            self.cmd("EXTEND")
            self.cmd("GRIP")
            self.cmd("RETRACT")
            vacio = self.percepcion.soporte_vacio(pos)
            en_garra = self.percepcion.grano_en_garra()
            if en_garra:
                self.cmd(f"FLAP {comp}")
                self.cmd("RELEASE")
                self.carga[comp] += 1
                self.estadisticas["tomados"] += 1
                if not vacio:
                    self.log(f"{pos}: OJO, tomado pero el soporte no se ve vacío")
                self.log(f"{pos}: {cat} guardado en {comp} (carga {self.carga})")
                return
            self.cmd("RELEASE")
            if vacio:
                self.estadisticas["caidos"] += 1
                self.log(f"{pos}: ¡el grano se cayó! (probable -7 abandonado)")
                return
            self.log(f"{pos}: agarre fallido (intento {intento + 1})")
        self.estadisticas["fallidos"] += 1

    def mover_carro(self, mm: float):
        self.cmd(f"SLIDE {mm:.1f}")
        self.carro_mm = mm

    def salir_del_arbol(self, siguiente: str) -> str:
        if self.arbol is not None:
            self.cmd("LIFT BAJA")
            self.mover_carro(0.0)
            self.mover(-RETROCESO_MM)
            self.arbol = None
        return siguiente

    def e_ir_a_cajas(self):
        self.salir_del_arbol("IR_A_CAJAS")
        pendientes = [c for c in ("RIPE", "OVERRIPE") if self.carga[c] > 0]
        # La caja más cercana primero.
        pendientes.sort(key=lambda c: abs(SECTORES[self.sector_caja(c)][0] - self.x))
        for comp in pendientes:
            sx, _ = SECTORES[self.sector_caja(comp)]
            viaje = self.tiempo(self.ruta(sx, Y_ANTES_CAJA, 270.0)) + \
                duracion_estimada("APPROACH") + duracion_estimada("DUMP")
            if self.reloj.ahora() + viaje > DURACION_RONDA_S - 5:
                self.log(f"No alcanzo a llegar a la caja {CAJA[comp]}: me quedo con la carga")
                return "FIN"
            self.log(f"Llevando {self.carga[comp]} granos a la caja {CAJA[comp]}")
            self.ejecutar_ruta(sx, Y_ANTES_CAJA, 270.0)
            self.cmd(f"APPROACH {DIST_CAJA_MM}")
            self.cmd(f"DUMP {comp}")
            self.carga[comp] = 0
            self.mover(-DIST_CAJA_MM)
        return "IR_A_ARBOL"
