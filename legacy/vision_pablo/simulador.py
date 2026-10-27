"""Simulador simple de la ronda: prueba la estrategia y estima el puntaje.

No simula física ni visión real: los errores se modelan con probabilidades.
Los tiempos salen de esp32.duracion_estimada (provisionales).

Uso:
  python simulador.py                 # una ronda con todo el detalle
  python simulador.py --rondas 500    # estadísticas de muchas rondas
  python simulador.py --p-confusion 0.05 --rondas 500   # visión peor
"""
from __future__ import annotations

import argparse
import random
import statistics
import sys

from clasificador import CATEGORIA, FILAS, NO_SE, nombres_posiciones
from esp32 import ESP32Simulada, RelojSimulado
from ronda import ARBOLES, SECTORES, Ronda, Sorteo, x_soporte

COLORES = ["ROJO", "NARANJO", "AMARILLO", "VERDE", "AZUL", "NEGRO"]


class Mundo:
    """Estado verdadero de la cancha. Reacciona a los comandos de la ESP32."""

    def __init__(self, rng: random.Random, p_falla_agarre=0.10, p_caida=0.03):
        self.rng = rng
        self.p_falla_agarre = p_falla_agarre
        self.p_caida = p_caida
        sectores = rng.sample(list(SECTORES), 3)
        piscinas = sorted(set(p for p in (rng.randint(1, 6), rng.randint(1, 6)) if p <= 5))
        self.sorteo = Sorteo(*sectores, piscinas=piscinas)
        self.arboles = {a: {pos: rng.choice(COLORES) for pos in nombres_posiciones()}
                        for a in ARBOLES}
        self.arbol_actual = None
        self.fila = None
        self.carro = 0.0
        self.garra = None
        self.flap = "RIPE"
        self.compartimentos = {"RIPE": [], "OVERRIPE": []}
        self.cajas = {"ROJA": [], "AZUL": []}
        self.abandonados = []

    def pos_bajo_garra(self):
        if self.arbol_actual is None or self.fila is None:
            return None
        n = FILAS[self.fila]
        for i in range(1, n + 1):
            pos = f"{self.fila}-{i}"
            if abs(x_soporte(pos) - self.carro) < 20:
                return pos
        return None

    def al_comando(self, texto: str, esp) -> str:
        partes = texto.split()
        nombre = partes[0]
        if nombre == "LIFT":
            self.fila = partes[1].lower()
        elif nombre == "SLIDE":
            self.carro = float(partes[1])
        elif nombre == "MOVE" and float(partes[1]) < 0:
            self.arbol_actual = None  # se alejó del árbol
        elif nombre == "GRIP":
            pos = self.pos_bajo_garra()
            color = self.arboles[self.arbol_actual].get(pos) if pos else None
            if color:
                r = self.rng.random()
                if r < self.p_falla_agarre:
                    pass  # no lo agarró, sigue en el soporte
                elif r < self.p_falla_agarre + self.p_caida:
                    self.abandonados.append(color)
                    self.arboles[self.arbol_actual][pos] = None
                else:
                    self.garra = color
                    self.arboles[self.arbol_actual][pos] = None
        elif nombre == "FLAP":
            self.flap = partes[1]
        elif nombre == "RELEASE":
            if self.garra:
                self.compartimentos[self.flap].append(self.garra)
                self.garra = None
        elif nombre == "DUMP":
            caja = self.caja_cercana(esp.x, esp.y)
            granos = self.compartimentos[partes[1]]
            if caja:
                self.cajas[caja].extend(granos)
            else:
                self.abandonados.extend(granos)  # ¡vació en el suelo!
            self.compartimentos[partes[1]] = []
        return ""

    def caja_cercana(self, x, y):
        for caja, sector in (("ROJA", self.sorteo.sector_roja), ("AZUL", self.sorteo.sector_azul)):
            sx, sy = SECTORES[sector]
            if abs(sx - x) < 300 and abs(sy - y) < 800:
                return caja
        return None

    def puntaje(self) -> int:
        p = 0
        for caja, granos in self.cajas.items():
            for g in granos:
                cat = CATEGORIA[g]
                if cat == "VERDE":
                    p -= 7
                elif cat == "MADURO":
                    p += 7 if caja == "ROJA" else -7
                else:
                    p += 5 if caja == "AZUL" else -5
        a_bordo = self.compartimentos["RIPE"] + self.compartimentos["OVERRIPE"]
        if self.garra:
            a_bordo.append(self.garra)
        for g in a_bordo:
            p += {"MADURO": 3, "SOBREMADURO": 1, "VERDE": -7}[CATEGORIA[g]]
        p -= 7 * len(self.abandonados)
        return p


class PercepcionSimulada:
    def __init__(self, mundo: Mundo, reloj: RelojSimulado, rng: random.Random,
                 p_no_se=0.08, p_confusion=0.01):
        self.mundo = mundo
        self.reloj = reloj
        self.rng = rng
        self.p_no_se = p_no_se
        self.p_confusion = p_confusion

    def leer_cancha(self) -> Sorteo:
        self.reloj.esperar(1.0)
        return self.mundo.sorteo

    def escanear_arbol(self, arbol: int) -> dict:
        self.reloj.esperar(1.5)
        self.mundo.arbol_actual = arbol
        resultado = {}
        for pos, color in self.mundo.arboles[arbol].items():
            verdad = color or "VACIO"
            r = self.rng.random()
            if r < self.p_no_se:
                resultado[pos] = NO_SE
            elif r < self.p_no_se + self.p_confusion:
                resultado[pos] = self.rng.choice([c for c in COLORES if c != verdad])
            else:
                resultado[pos] = verdad
        return resultado

    def ajuste_fino(self):
        self.reloj.esperar(0.4)
        pos = self.mundo.pos_bajo_garra()
        if pos is None or self.mundo.arboles[self.mundo.arbol_actual][pos] is None:
            return None
        return self.rng.gauss(0, 3)

    def soporte_vacio(self, pos: str) -> bool:
        self.reloj.esperar(0.2)
        return self.mundo.arboles[self.mundo.arbol_actual][pos] is None

    def grano_en_garra(self) -> bool:
        return self.mundo.garra is not None


def simular(semilla, args, log=None):
    rng = random.Random(semilla)
    mundo = Mundo(rng, args.p_falla_agarre, args.p_caida)
    reloj = RelojSimulado()
    x, y = SECTORES[mundo.sorteo.sector_partida]
    esp = ESP32Simulada(reloj, x, y, 90.0, mundo=mundo)
    percepcion = PercepcionSimulada(mundo, reloj, rng, args.p_no_se, args.p_confusion)
    ronda = Ronda(esp, percepcion, reloj, log=log or (lambda m: None))
    ronda.ejecutar()
    return mundo, ronda, reloj


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--rondas", type=int, default=1)
    p.add_argument("--semilla", type=int, default=1)
    p.add_argument("--p-no-se", type=float, default=0.08,
                   help="prob. de que la visión diga NO_SE")
    p.add_argument("--p-confusion", type=float, default=0.01,
                   help="prob. de que la visión se equivoque de color")
    p.add_argument("--p-falla-agarre", type=float, default=0.10)
    p.add_argument("--p-caida", type=float, default=0.03,
                   help="prob. de que el grano se caiga al suelo al agarrarlo")
    args = p.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")  # acentos en la consola de Windows

    if args.rondas == 1:
        mundo, ronda, reloj = simular(args.semilla, args, log=print)
        print(f"\nCajas: {mundo.cajas}")
        print(f"A bordo: {mundo.compartimentos}  Abandonados: {mundo.abandonados}")
        print(f"PUNTAJE: {mundo.puntaje()}   tiempo usado: {reloj.ahora():.0f} s")
        return

    puntajes, tiempos, tomados = [], [], []
    for s in range(args.semilla, args.semilla + args.rondas):
        mundo, ronda, reloj = simular(s, args)
        puntajes.append(mundo.puntaje())
        tiempos.append(reloj.ahora())
        tomados.append(ronda.estadisticas["tomados"])
    negativos = sum(1 for v in puntajes if v <= 0)
    print(f"{args.rondas} rondas simuladas")
    print(f"Puntaje: promedio {statistics.mean(puntajes):.1f}, mínimo {min(puntajes)}, "
          f"máximo {max(puntajes)}, rondas con 0 o menos: {negativos}")
    print(f"Granos tomados por ronda: promedio {statistics.mean(tomados):.1f}")
    print(f"Tiempo usado: promedio {statistics.mean(tiempos):.0f} s")


if __name__ == "__main__":
    main()
