"""Calibrar con el brazo de verdad las ALTURAS de cada piso del árbol y las posturas de las CANASTAS internas.
No mueve la base del robot (no hace falta el Mega). Lo que se guarda lo usan tools.ascensor y la misión.

    python3 -m tools.calibrar_cosecha pisos        # alturas (y corrección de profundidad / lado) de cada piso
    python3 -m tools.calibrar_cosecha canastas     # ángulos del brazo sobre cada canasta interna

PISOS: poner el robot DERECHO frente al árbol, con el frente a la distancia que indica cada piso (la misma a la que
lo deja tools.ascensor: borde de la línea + retroceso del piso) y la base del brazo frente a una pelota de ese piso
(el brazo está 6,3 cm a la izquierda del centro del robot). El brazo va al piso con la garra abierta, unos cm antes de
la pelota; se corrige hasta que la garra quede justo frente al centro de la pelota, se entra, se cierra y se guarda.
  1 2 3   piso BAJA / MEDIA / ALTA (el brazo va ahí)        e  ENTRAR hasta la pelota     v  volver (antes)
  ↑ ↓     altura ±5 mm   , .  altura ±1 mm                 i k  hacia el árbol / atrás ±5 mm
  ← →     izquierda / derecha ±5 mm                         c  cerrar (dice si tomó algo)   a  abrir
  s       sacar (sube y retrocede)    d  dejarla en la canasta de maduros    h  brazo a descanso
  t       GUARDAR el piso (altura, profundidad y lado) en robot_local.json, sección ascensor
  q       salir

CANASTAS: el brazo va a la postura de la canasta (pasando por arriba si gira la base); se corrige articulación por
articulación (o a mano, sin torque) y se guarda en el perfil del brazo (pi/brazos/<perfil>.json).
  m n     canasta de MADUROS / SOBREMADUROS (el brazo va ahí)
  1..5    elegir articulación: 1 base  2 hombro  3 codo1  4 codo2  5 muñeca
  ↑ ↓     esa articulación ±2°      [ ]  ±10°
  x       soltar el brazo (SIN torque, sujetarlo) para llevarlo a mano / x otra vez: lo sostiene donde quedó
  a c     abrir (soltar el grano) / cerrar
  p       PROBAR: descanso -> canasta -> abrir -> descanso (como en un agarre)
  t       GUARDAR la postura actual como la de esa canasta        h  brazo a descanso      q  salir
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import traceback

import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha

FILAS = {"1": "baja", "2": "media", "3": "alta"}
CANASTAS = {"m": ("deposito_maduro", cancha.MADURO), "n": ("deposito_sobremaduro", cancha.SOBREMADURO)}
ARTS = {"1": "base", "2": "hombro", "3": "codo1", "4": "codo2", "5": "muneca"}
PASOS_PLAN = ["antes", "entrando", "en la pelota", "arriba", "retirado"]


def conectar_brazo(a):
    from tools.brazo import crear
    b = crear(argparse.Namespace(perfil=a.perfil, puerto=a.puerto, banco=False, si=True))
    b.bus.torque(True)
    comp = None
    if b.cfg.compuertas.tipo == "dxl":           # las compuertas cerradas: que lo que caiga en la canasta quede ahí
        try:
            from cafebot.arm.compuertas import DepositosDXL
            comp = DepositosDXL(b.cfg, lambda: b.bus)
            comp.preparar()
        except Exception as e:
            print(f"[compuertas] {e}")
    return b


class Pisos:
    def __init__(self, b, log=print):
        self.b, self.cfg, self.log = b, b.cfg, log
        self.ca = self.cfg.ascensor
        sx = 0.5 * (self.cfg.ir.posiciones[0][0] + self.cfg.ir.posiciones[1][0])
        self.x_linea = sx + self.ca.linea_a_cara_m          # la cara del árbol con los IR sobre la línea
        self.fila = "media"
        self.plan, self.paso = None, None

    def aj(self, que, fila=None):
        return float(getattr(self.ca, f"{que}_{fila or self.fila}_m"))

    def poner(self, que, v):
        setattr(self.ca, f"{que}_{self.fila}_m", round(float(v), 4))

    def frente(self, fila=None):
        """Del frente del robot a la cara del árbol cuando tools.ascensor agarra en ese piso."""
        return self.x_linea + self.aj("retroceso", fila) - self.cfg.geometria.largo / 2

    def punto(self):
        x = self.x_linea + self.aj("retroceso") - self.ca.grano_delante_m + self.aj("dx")
        return np.array([x, self.cfg.mision.brazo_y + self.aj("dy"), self.aj("z")])

    def estado(self):
        p = self.punto()
        paso = PASOS_PLAN[self.paso] if self.paso is not None else "esperando"
        self.log(f"[{self.fila}] centro de la pelota a {100 * p[2]:.1f} cm del suelo ({100 * (p[2] - self.b.base[2]):+.1f} "
                 f"sobre la base del brazo) | profundidad {100 * self.aj('dx'):+.1f} cm, lado {100 * self.aj('dy'):+.1f} cm"
                 f" | el frente del robot a {100 * self.frente():.1f} cm de la cara del árbol | brazo: {paso}")

    def elegir(self, fila):
        self.fila = fila
        self.log(f"\nPISO {fila.upper()}: el frente del robot a {100 * self.frente():.1f} cm de la cara del árbol, derecho, "
                 f"con la base del brazo frente a una pelota de ese piso")
        self.ir_paso(0, nuevo=True)

    def ir_paso(self, i, nuevo=False):
        b = self.b
        if nuevo or self.plan is None:
            self.plan = b.plan_agarre(self.punto())
            if self.plan is None:
                self.paso = None
                raise RuntimeError("el brazo no alcanza ese punto: probar otra altura / profundidad (o el retroceso "
                                   "del piso en tools.ascensor)")
        if i == 0 and self.paso is None:
            b.garra(abrir=True)
        b.parar.clear()
        nombre, obj, t = self.plan[i]
        if not b._mover(obj, t, nombre):
            raise RuntimeError(f"{nombre}: {b.ultimo_motivo}")
        self.paso = i
        self.estado()

    def ajustar(self, que, d):
        self.poner(que, self.aj(que) + d)
        if self.paso in (0, 2):
            i = self.paso
            self.plan = None
            self.ir_paso(i, nuevo=True)
        else:
            self.estado()

    def entrar(self):
        if self.paso is None:
            self.ir_paso(0, nuevo=True)
        self.ir_paso(1)
        self.ir_paso(2)

    def volver(self):
        if self.paso is not None:
            self.ir_paso(0)

    def cerrar(self):
        ang = self.b.garra(abrir=False)
        self.log(f"  garra cerrada en {ang:.1f}°: " + ("VACÍA (cerró del todo)" if ang > self.cfg.brazo.garra_vacia_deg
                                                       else "¡con algo adentro!"))

    def sacar(self):
        if self.paso is None:
            raise RuntimeError("primero 1 / 2 / 3 y e")
        self.ir_paso(3)
        self.ir_paso(4)

    def depositar(self):
        b = self.b
        if "deposito_maduro" not in b.posturas:
            raise RuntimeError("no hay postura 'deposito_maduro': guardarla con  python3 -m tools.calibrar_cosecha canastas")
        b.parar.clear()
        if not b.ir_plegado("deposito_maduro"):
            self.log(f"  {b.ultimo_motivo}")
        b.garra(abrir=True)
        time.sleep(0.3)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        self.paso = None
        self.estado()

    def guardar(self):
        f = self.fila
        claves = {f"{q}_{f}_m": self.aj(q) for q in ("z", "dx", "dy")}
        ruta = guardar_local("ascensor", claves)
        self.log(f"GUARDADO piso {f}: altura {100 * claves[f'z_{f}_m']:.1f} cm, profundidad {100 * claves[f'dx_{f}_m']:+.1f}, "
                 f"lado {100 * claves[f'dy_{f}_m']:+.1f} cm -> {ruta} (subirlo al repo)")


class Canastas:
    def __init__(self, b, log=print):
        self.b, self.cfg, self.log = b, b.cfg, log
        self.nombre, self.cat = CANASTAS["m"]
        self.art = "base"
        self.suelto = False

    def estado(self):
        ang = self.b.angulos_nombre()
        guard = self.b.posturas.get(self.nombre, {})
        self.log(f"[{self.nombre}] articulación elegida: {self.art.upper()} | ahora: " + "  ".join(
            f"{n} {ang[n]:+.1f}°" + (f"({guard[n]:+.1f})" if n in guard and abs(guard[n] - ang[n]) > 0.5 else "")
            for n in ("base", "hombro", "codo1", "codo2", "muneca")) + ("  | SIN TORQUE" if self.suelto else "")
            + "   (entre paréntesis: lo guardado)")

    def ir(self, tecla):
        self.nombre, self.cat = CANASTAS[tecla]
        if self.nombre not in self.b.posturas:
            self.log(f"no hay postura '{self.nombre}' todavía: llevar el brazo a mano (x) sobre la canasta y guardar (t)")
            return
        self.b.parar.clear()
        if not self.b.ir_plegado(self.nombre):
            self.log(f"  no llegó bien: {self.b.ultimo_motivo}")
        self.estado()

    def mover(self, d):
        if self.suelto:
            raise RuntimeError("está SIN torque: x para sostenerlo primero")
        ang = self.b.angulos_nombre()
        self.b.parar.clear()
        if not self.b.mover_articulacion(self.art, ang[self.art] + d, t=0.4):
            self.log(f"  {self.b.ultimo_motivo}")
        self.estado()

    def soltar(self):
        b = self.b
        if not self.suelto:
            input("El brazo va a quedar SIN torque: SUJETARLO con la mano y ENTER... ")
            b.bus.torque(False, b.IDS)
            self.suelto = True
            self.log("SIN torque: llevarlo a mano sobre la canasta y apretar x para que lo sostenga ahí")
        else:
            b.bus.torque(True, b.IDS)
            self.suelto = False
            self.log("con torque: lo sostiene donde quedó")
        self.estado()

    def probar(self):
        b = self.b
        if self.suelto:
            raise RuntimeError("está SIN torque: x primero")
        if self.nombre not in b.posturas:
            raise RuntimeError(f"no hay postura '{self.nombre}': llevar el brazo sobre la canasta y guardarla (t)")
        b.parar.clear()
        b.garra(abrir=False)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        if not b.ir_plegado(self.nombre):
            self.log(f"  yendo a la canasta: {b.ultimo_motivo}")
        b.garra(abrir=True)
        time.sleep(0.4)
        b.ir_plegado(self.cfg.brazo.postura_espera)
        self.log("prueba hecha: descanso -> canasta -> abrir -> descanso")

    def guardar(self):
        if self.suelto:
            self.b.bus.torque(True, self.b.IDS)
            self.suelto = False
        post = self.b.grabar_postura(self.nombre)
        self.log(f"GUARDADA '{self.nombre}' en {self.b.perfil.ruta}: "
                 + " ".join(f"{k} {v:+.1f}°" for k, v in post.items()) + " (subirlo al repo)")


def bucle(modo, obj, b):
    import termios
    import tty
    from tools.cabezal import leer_tecla
    ayuda = __doc__.split("PISOS:" if modo == "pisos" else "CANASTAS:")[1]
    ayuda = ayuda.split("CANASTAS:")[0] if modo == "pisos" else ayuda
    print(ayuda)
    viejo = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())
    try:
        if modo == "pisos":
            obj.elegir("media")
        else:
            obj.estado()
        while True:
            k = leer_tecla(0.2)
            if not k:
                continue
            if k in ("q", "Q"):
                break
            try:
                if modo == "pisos":
                    acciones = {
                        "arriba": lambda: obj.ajustar("z", 0.005), "abajo": lambda: obj.ajustar("z", -0.005),
                        ".": lambda: obj.ajustar("z", 0.001), ",": lambda: obj.ajustar("z", -0.001),
                        "i": lambda: obj.ajustar("dx", 0.005), "k": lambda: obj.ajustar("dx", -0.005),
                        "izq": lambda: obj.ajustar("dy", 0.005), "der": lambda: obj.ajustar("dy", -0.005),
                        "e": obj.entrar, "v": obj.volver, "c": obj.cerrar,
                        "a": lambda: obj.log(f"  garra abierta en {b.garra(abrir=True):.1f}°"),
                        "s": obj.sacar, "d": obj.depositar, "t": obj.guardar,
                        "h": lambda: (b.ir_plegado(b.cfg.brazo.postura_espera), setattr(obj, "paso", None), obj.estado()),
                    }
                    if k in FILAS:
                        obj.elegir(FILAS[k])
                    elif k in acciones:
                        acciones[k]()
                else:
                    acciones = {
                        "arriba": lambda: obj.mover(2), "abajo": lambda: obj.mover(-2),
                        "]": lambda: obj.mover(10), "[": lambda: obj.mover(-10),
                        "x": obj.soltar, "p": obj.probar, "t": obj.guardar,
                        "a": lambda: obj.log(f"  garra abierta en {b.garra(abrir=True):.1f}°"),
                        "c": lambda: obj.log(f"  garra cerrada en {b.garra(abrir=False):.1f}°"),
                        "h": lambda: (b.ir_plegado(b.cfg.brazo.postura_espera), obj.estado()),
                    }
                    if k in CANASTAS:
                        obj.ir(k)
                    elif k in ARTS:
                        obj.art = ARTS[k]
                        obj.estado()
                    elif k in acciones:
                        acciones[k]()
            except RuntimeError as e:
                print(f"  {e}")
            except Exception as e:
                print(f"  ERROR {e}")
                traceback.print_exc()
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, viejo)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("modo", choices=("pisos", "canastas"))
    ap.add_argument("--perfil", default=None, help="otro brazo (pi/brazos/<perfil>.json)")
    ap.add_argument("--puerto", default=None, help="puerto del U2D2 si no lo encuentra solo")
    a = ap.parse_args()
    b = conectar_brazo(a)
    try:
        obj = Pisos(b) if a.modo == "pisos" else Canastas(b)
        bucle(a.modo, obj, b)
    finally:
        try:
            b.bus.torque(True, b.IDS)
        except Exception:
            pass
        print("El brazo queda con torque donde está (para descansarlo: python3 -m tools.brazo ir descanso).")
        b.cerrar()


if __name__ == "__main__":
    main()
