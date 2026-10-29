"""Líneas negras de la cancha vistas por los 4 sensores IR (QTR-1A) — su lugar en el mapa.

Cada `EventoIR` del Mega trae qué sensores entraron (blanco→negro) o salieron (negro→blanco) de una
línea y la pose de SU odometría en ese momento. Aquí:

  1. Se ubica el sensor en el MAPA en el instante del evento: pose fusionada actual ⊕ lo que el robot
     se movió desde el evento (odometría del Mega: muy precisa en unos pocos cm).
  2. Una entrada y la salida siguiente del mismo sensor forman un CRUCE. El punto medio queda sobre el
     eje de la línea (aunque se cruce en diagonal); lo recorrido sobre negro ≈ 19 mm / sen(ángulo).
  3. Si se conoce dónde está la cancha en el mapa (`fijar_cancha`), el cruce se ASOCIA a la línea
     conocida más cercana (cancha.lineas_cancha) y se calcula el RESIDUO: a qué distancia de esa
     línea cree la navegación que estaba el sensor. Residuos chicos = la pose es buena; un sesgo
     constante = algo mal medido (posición del sensor, de la cancha, odometría).
  4. Opcional (`ConfigIR.corregir_pose`): el cruce asociado corrige el EKF solo en la dirección
     perpendicular a la línea (lo único que un cruce mide de verdad).
  5. Los sensores van junto a las ruedas: si uno ENTRA a una línea del perímetro, esa rueda está pisándola
     (el reglamento obliga a reiniciar). Se registra como alerta para corregir la ruta.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

from cafebot.mundo import cancha
from cafebot.navegacion.geometria import componer, invertir, relativo, rotar, transformar


@dataclass
class CruceIR:
    """Un sensor pasó sobre una línea (entró y salió)."""
    sensor: int
    p_entra: np.ndarray        # mapa (m)
    p_sale: np.ndarray
    centro: np.ndarray         # punto medio (mapa): sobre el eje de la línea
    largo: float               # recorrido sobre negro (m)
    t: float
    linea: str | None = None   # nombre de la línea conocida asociada
    residuo: float | None = None   # m: distancia firmada del centro al eje de esa línea
    corregido: bool = False    # se usó para corregir el EKF


class DetectorLineas:
    def __init__(self, cfg_ir):
        self.cfg = cfg_ir
        self.sensores = [np.array(p, float) for p in cfg_ir.posiciones]
        self.cruces: deque[CruceIR] = deque(maxlen=400)
        self.mascara = 0
        self._entradas = {}                    # sensor -> (pose del Mega al entrar, t)
        self.lineas = cancha.lineas_cancha()
        self.cancha_en_mapa = None             # pose de la cancha en el marco del mapa
        self.descartados = 0
        self.alertas: deque = deque(maxlen=50)  # (t, sensor, línea): una rueda entró al perímetro

    def fijar_cancha(self, T_mapa_cancha):
        self.cancha_en_mapa = None if T_mapa_cancha is None else np.asarray(T_mapa_cancha, float)

    # ------------------------------------------------------------------ eventos
    def procesar(self, ev, pose_mapa, mega_ahora) -> list[tuple[CruceIR, np.ndarray]]:
        """ev: EventoIR. pose_mapa: pose fusionada AHORA. mega_ahora: pose del Mega AHORA (m, m, rad).
        Devuelve los cruces completados con el punto medio expresado en el sistema del robot AHORA
        (lo que necesita la corrección del EKF)."""
        self.mascara = ev.mascara
        mega_ev = np.array([ev.x, ev.y, ev.th])
        hechos = []
        for k in range(len(self.sensores)):
            bit = 1 << k
            if ev.subida & bit:
                self._entradas[k] = (mega_ev, ev.recibido)
                self._revisar_perimetro(k, transformar(self._sensor_desde(k, mega_ahora, mega_ev)[None, :], pose_mapa)[0],
                                        ev.recibido)
            if ev.bajada & bit and k in self._entradas:
                mega_e, t_e = self._entradas.pop(k)
                # el sensor al entrar y al salir, en el sistema del robot AHORA
                s_e = self._sensor_desde(k, mega_ahora, mega_e)
                s_s = self._sensor_desde(k, mega_ahora, mega_ev)
                largo = float(np.linalg.norm(s_s - s_e))
                if not (0.004 <= largo <= 0.25) or ev.recibido - t_e > 10.0:
                    self.descartados += 1          # ruido, o avanzó a lo largo de una línea
                    continue
                s_medio = (s_e + s_s) / 2
                pe, ps, pm = (transformar(q[None, :], pose_mapa)[0] for q in (s_e, s_s, s_medio))
                c = CruceIR(k, pe, ps, pm, largo, ev.recibido)
                self._asociar(c)
                self.cruces.append(c)
                hechos.append((c, s_medio))
        return hechos

    def _sensor_desde(self, k, mega_ahora, mega_ev) -> np.ndarray:
        """Posición del sensor k en el sistema del robot AHORA, cuando el robot estaba en mega_ev."""
        d = relativo(mega_ahora, mega_ev)
        return transformar(self.sensores[k][None, :], d)[0]

    def _revisar_perimetro(self, k, p_mapa, t):
        if self.cancha_en_mapa is None:
            return
        p = transformar(p_mapa[None, :], invertir(self.cancha_en_mapa))[0]
        for nombre, r in self.lineas:
            if nombre.startswith("perimetro") and r.contiene(np.array(p[0]), np.array(p[1]), 0.01):
                self.alertas.append((t, k, nombre))
                return

    # ------------------------------------------------------------------ líneas conocidas
    def _asociar(self, c: CruceIR):
        if self.cancha_en_mapa is None:
            return
        p = transformar(c.centro[None, :], invertir(self.cancha_en_mapa))[0]   # a la cancha
        candidatos = []
        for nombre, r in self.lineas:
            largo_x = r.ancho_x >= r.ancho_y               # eje de la línea a lo largo de x o de y
            if largo_x:
                fuera = abs(p[0] - r.cx) - r.ancho_x / 2
                d = p[1] - r.cy
            else:
                fuera = abs(p[1] - r.cy) - r.ancho_y / 2
                d = p[0] - r.cx
            if fuera > 0.03:                               # más allá de la punta de la línea
                continue
            candidatos.append((abs(d), d, nombre))
        if not candidatos:
            return
        candidatos.sort()
        dist, d, nombre = candidatos[0]
        ambiguo = len(candidatos) > 1 and candidatos[1][0] < 2 * self.cfg.asociar_m
        if dist <= self.cfg.asociar_m and not ambiguo:
            c.linea, c.residuo = nombre, float(d)

    def recta_en_mapa(self, nombre):
        """(normal unitaria, c) de la línea `nombre` en el mapa: los puntos del eje cumplen n·p = c."""
        r = dict(self.lineas)[nombre]
        T = self.cancha_en_mapa
        n_c = np.array([0.0, 1.0]) if r.ancho_x >= r.ancho_y else np.array([1.0, 0.0])
        n = rotar(n_c, T[2])
        p0 = transformar(np.array([[r.cx, r.cy]]), T)[0]
        return n, float(n @ p0)

    # ------------------------------------------------------------------ resumen
    def estadisticas(self) -> dict:
        asoc = [c for c in self.cruces if c.residuo is not None]
        res = np.array([c.residuo for c in asoc]) if asoc else np.zeros(0)
        return {"cruces": len(self.cruces), "asociados": len(asoc),
                "residuo_medio": float(res.mean()) if len(res) else None,
                "residuo_std": float(res.std()) if len(res) > 1 else None,
                "corregidos": sum(c.corregido for c in self.cruces), "descartados": self.descartados,
                "perimetro": len(self.alertas)}
