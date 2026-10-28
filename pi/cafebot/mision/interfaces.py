"""Contratos entre la misión y los módulos que el equipo desarrolla por separado.

La misión SOLO habla con estas clases. Para conectar el brazo o la visión reales, basta con
escribir una clase que herede de `Brazo` o `Vision` e implemente sus métodos; nada más cambia.

    Sistema del ROBOT: x adelante, y izquierda, z hacia arriba desde el SUELO (metros).
    Categorías: "MADURO", "SOBREMADURO", "VERDE", "VACIO", "NO_SE" (no tocar),
                "FUERA" (el soporte no cae dentro de la foto: la misión se corre de lado y saca otra).

Implementaciones reales: cafebot/arm/brazo.py (BrazoXL430) y cafebot/vision/vision.py (VisionCamara).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass

import numpy as np

from cafebot.mundo import cancha


# ====================================================================== visión
@dataclass
class LecturaSoporte:
    clave: tuple          # (arbol, fila, col): ver cancha.Soporte
    categoria: str        # MADURO / SOBREMADURO / VERDE / VACIO / NO_SE / FUERA
    confianza: float      # 0..1


class Vision:
    def iniciar(self):
        """Antes del botón (abrir la cámara, cargar referencias): lo lento, fuera del reloj de la ronda."""

    def preparar_arbol(self):
        """Antes de la foto del árbol (p. ej. inclinar la cámara hacia arriba: X 2 <ángulo>)."""

    def clasificar_arbol(self, arbol: int, pose_robot_cancha, tablero=None) -> list[LecturaSoporte]:
        """Foto del árbol (frente del robot a ~50 cm) y clasificación de sus 16 soportes.
        pose_robot_cancha: (x, y, θ) del robot en la cancha.
        tablero: (x de la cara, y del centro, ángulo) del pie del árbol en el sistema del robot, medido por el
        LiDAR (más preciso que la pose para saber dónde cae cada soporte en la imagen). Puede ser None."""
        raise NotImplementedError

    def color_cajas(self, zonas_con_caja: list[int], pose_robot_cancha) -> dict[int, str]:
        """{zona: 'ROJA' | 'AZUL'} para las cajas que vea DESDE LA PARTIDA. Puede devolver {} si no sabe."""
        return {}

    def color_caja(self, zona: int, pose_robot_cancha) -> str | None:
        """Con el robot mirando hacia la caja de `zona`: 'ROJA', 'AZUL' o None si no está seguro."""
        return None


# ====================================================================== brazo
class Brazo:
    def preparar(self) -> bool:
        """Encender y llevar a la postura de trabajo. False si no se puede usar."""
        return True

    def recoger(self, p_robot, categoria: str) -> bool:
        """Sacar UN grano cuyo centro está en p_robot = (x, y, z) del sistema del robot, y dejarlo
        en el depósito interno de su categoría. True si quedó dentro del robot."""
        raise NotImplementedError

    def alcanzable(self, p_robot) -> bool:
        """¿Puede llegar a ese punto desde la pose actual de la base?"""
        return True

    def descansar(self):
        """Postura plegada y segura para navegar (idealmente apoyado y sin torque)."""

    def estado(self) -> str:
        return ""


class BrazoNulo(Brazo):
    """Sin brazo: la misión navega, mira y se acerca, pero no recoge (pruebas de navegación)."""

    def recoger(self, p_robot, categoria):
        print(f"   [brazo nulo] recogería {categoria} en x={p_robot[0]:.3f} y={p_robot[1]:+.3f} z={p_robot[2]:.3f}")
        time.sleep(0.5)
        return False

    def estado(self):
        return "sin brazo (solo navegación)"


# ====================================================================== simulados
class VisionSimulada(Vision):
    """Usa la verdad del escenario simulado, con errores: a veces NO_SE, rara vez se equivoca."""

    def __init__(self, escenario: cancha.Escenario, rng=None, p_no_se=0.10, p_error=0.02, cfg_camara=None,
                 ve_cajas_al_partir=True):
        """cfg_camara: si se da, los soportes que la cámara real no vería (modelo de proyeccion.py) salen FUERA.
        ve_cajas_al_partir=False: como la cámara real, que desde la partida no ve las cajas."""
        self.esc = escenario
        self.rng = rng or np.random.default_rng()
        self.p_no_se, self.p_error = p_no_se, p_error
        self.cfg_camara = cfg_camara
        self.ve_cajas_al_partir = ve_cajas_al_partir

    def clasificar_arbol(self, arbol, pose_robot_cancha, tablero=None):
        time.sleep(0.5)
        visibles = None
        if self.cfg_camara is not None:
            from cafebot.vision.proyeccion import ModeloCamara
            from cafebot.vision.vision import tablero_desde_pose
            t = tablero if tablero is not None else tablero_desde_pose(arbol, pose_robot_cancha)
            visibles = set(ModeloCamara(self.cfg_camara).posiciones_soportes(arbol, t))
        out = []
        for s in cancha.soportes(arbol):
            real = self.esc.granos.get(s.clave, ("VACIO", ""))[0]
            r = self.rng.random()
            if visibles is not None and f"{s.fila}-{s.col + 1}" not in visibles:
                out.append(LecturaSoporte(s.clave, "FUERA", 0.0))
            elif r < self.p_no_se:
                out.append(LecturaSoporte(s.clave, "NO_SE", 0.3))
            elif r < self.p_no_se + self.p_error:
                otra = self.rng.choice([c for c in (cancha.MADURO, cancha.SOBREMADURO, cancha.VERDE) if c != real])
                out.append(LecturaSoporte(s.clave, str(otra), 0.8))
            else:
                out.append(LecturaSoporte(s.clave, real, 0.95))
        return out

    def color_cajas(self, zonas_con_caja, pose_robot_cancha):
        if not self.ve_cajas_al_partir:
            return {}
        col = {self.esc.zona_caja_roja: "ROJA", self.esc.zona_caja_azul: "AZUL"}
        return {z: col[z] for z in zonas_con_caja if z in col}

    def color_caja(self, zona, pose_robot_cancha):
        time.sleep(0.3)
        return {self.esc.zona_caja_roja: "ROJA", self.esc.zona_caja_azul: "AZUL"}.get(zona)


class BrazoSimulado(Brazo):
    """Recoge si el grano está al alcance (mismo modelo de alcance que docs/brazo.md) y de la
    categoría real del escenario va al depósito. Registra lo que queda dentro del robot."""

    def __init__(self, escenario: cancha.Escenario, pose_robot_fn, base_xyz=(0.10, 0.0, 0.36), alcance=0.38,
                 t_agarre=4.0, p_falla=0.1, rng=None):
        self.esc = escenario
        self.pose_robot_fn = pose_robot_fn      # pose REAL del robot en la cancha (simulación)
        self.base = np.array(base_xyz)
        self.alcance = alcance
        self.t = t_agarre
        self.p_falla = p_falla
        self.rng = rng or np.random.default_rng()
        self.depositos = {cancha.MADURO: [], cancha.SOBREMADURO: [], cancha.VERDE: []}
        self.botados = []

    def alcanzable(self, p_robot):
        d = np.asarray(p_robot) - self.base
        return math.hypot(d[0], d[1]) < 0.20 and np.linalg.norm(d) < self.alcance

    def _soporte_real(self, p_robot):
        """¿Qué grano hay realmente donde apunta el brazo? (con la pose REAL del robot)"""
        x, y, th = self.pose_robot_fn()
        c, s = math.cos(th), math.sin(th)
        px, py = x + c * p_robot[0] - s * p_robot[1], y + s * p_robot[0] + c * p_robot[1]
        mejor, dmin = None, 1e9
        for a in (1, 2, 3):
            for sop in cancha.soportes(a):
                d = math.dist((px, py, p_robot[2]), (sop.x, sop.y, sop.z))
                if d < dmin:
                    mejor, dmin = sop, d
        return mejor, dmin

    def recoger(self, p_robot, categoria):
        time.sleep(self.t)
        if not self.alcanzable(p_robot):
            return False
        sop, error = self._soporte_real(p_robot)
        if error > 0.025 or sop.clave not in self.esc.granos:       # la garra deja ±2,5 cm de margen
            return False
        if self.rng.random() < self.p_falla:
            return False
        cat, _ = self.esc.granos.pop(sop.clave)                     # sale del árbol
        self.depositos[categoria if categoria in self.depositos else cat].append(cat)
        return True

    def estado(self):
        return " ".join(f"{k[:4]}={len(v)}" for k, v in self.depositos.items())


class VisionNula(Vision):
    """Sin visión: todo es NO_SE, así que la misión no toca ningún grano (solo navega y se acerca)."""

    def clasificar_arbol(self, arbol, pose_robot_cancha, tablero=None):
        return [LecturaSoporte(s.clave, "NO_SE", 0.0) for s in cancha.soportes(arbol)]


class VisionFija(Vision):
    """SOLO PARA PROBAR EL BRAZO en el taller con un árbol cargado únicamente con granos de `categoria`.
    Nunca usar en una ronda: tomaría los verdes (−7 cada uno)."""

    def __init__(self, categoria=cancha.MADURO):
        self.cat = categoria

    def clasificar_arbol(self, arbol, pose_robot_cancha, tablero=None):
        return [LecturaSoporte(s.clave, self.cat, 1.0) for s in cancha.soportes(arbol)]


# ====================================================================== botón de inicio
class Boton:
    def esperar(self):
        """Bloquea hasta que se aprieta el botón de inicio."""


class BotonGPIO(Boton):
    """Pulsador entre el GPIO `pin` (numeración BCM) y GND, con pull-up interno."""

    def __init__(self, pin):
        from gpiozero import Button          # viene con Raspberry Pi OS (usa lgpio en la Pi 5)
        self.b = Button(pin, pull_up=True, bounce_time=0.05)

    def esperar(self):
        self.b.wait_for_press()
        self.b.wait_for_release()            # partir al SOLTAR: la mano ya no está sobre el robot


class BotonEnter(Boton):
    def esperar(self):
        input(">>> ENTER para partir (simula el botón de inicio)... ")


# ====================================================================== depósitos (compuertas)
class Depositos:
    """Compuertas traseras: k = 0 maduros (caja roja), k = 1 sobremaduros (caja azul)."""

    def __init__(self, mega, cfg_mision):
        self.mega, self.c = mega, cfg_mision

    def vaciar(self, k: int):
        self.mega.comando(f"X {k} {self.c.compuerta_abierta_deg[k]:.0f}")
        time.sleep(self.c.t_vaciado_s)
        self.mega.comando(f"X {k} {self.c.compuerta_cerrada_deg[k]:.0f}")
        time.sleep(0.4)

    def cerrar_todo(self):
        for k in (0, 1):
            self.mega.comando(f"X {k} {self.c.compuerta_cerrada_deg[k]:.0f}")


class DepositosSimulados(Depositos):
    """Como Depositos, y además mira (con la pose REAL) en qué caja cayeron los granos."""

    def __init__(self, mega, cfg_mision, brazo_sim: BrazoSimulado, escenario, pose_real_fn):
        super().__init__(mega, cfg_mision)
        self.brazo, self.esc, self.pose_real_fn = brazo_sim, escenario, pose_real_fn
        self.en_roja, self.en_azul, self.abandonados = [], [], []

    def vaciar(self, k):
        super().vaciar(k)
        cat = (cancha.MADURO, cancha.SOBREMADURO)[k]
        granos, self.brazo.depositos[cat] = self.brazo.depositos[cat], []
        x, y, th = self.pose_real_fn()
        sx = x + math.cos(th) * self.c.salida_x
        sy = y + math.sin(th) * self.c.salida_x
        if cancha.rect_caja(self.esc.zona_caja_roja).contiene(np.array(sx), np.array(sy)):
            self.en_roja += granos
        elif cancha.rect_caja(self.esc.zona_caja_azul).contiene(np.array(sx), np.array(sy)):
            self.en_azul += granos
        else:
            self.abandonados += granos
        return sx, sy

    def puntaje(self) -> int:
        en_robot = [c for v in self.brazo.depositos.values() for c in v]
        return cancha.puntaje(en_robot, self.en_roja, self.en_azul, self.abandonados)
