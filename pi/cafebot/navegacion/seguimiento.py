"""Seguimiento de objetos que se MUEVEN (personas, manos, un robot que pasa).

Portado del tracker del código antiguo (allí llamado "JPDA"). En rigor es asociación GNN:
agrupar puntos → asignación húngara contra las predicciones → un filtro de Kalman de
velocidad constante por objeto. Para la cancha basta. Su uso principal: marcar zonas
dinámicas para que el SLAM no las meta al mapa ni al ICP.
"""
from __future__ import annotations

from collections import deque

import numpy as np
from scipy.optimize import linear_sum_assignment

from cafebot.navegacion.landmarks import agrupar


class PistaKalman:
    _siguiente = 1

    def __init__(self, centro, radio, dt=0.1):
        self.id = PistaKalman._siguiente
        PistaKalman._siguiente += 1
        self.x = np.array([centro[0], centro[1], 0.0, 0.0])
        self.P = np.diag([0.05, 0.05, 0.8, 0.8])
        self.F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]], float)
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], float)
        self.Q = np.diag([0.01, 0.01, 0.15, 0.15])
        self.R = np.eye(2) * 0.015
        self.radio = radio
        self.aciertos = 1
        self.perdidas = 0
        self.historia = deque([np.array(centro, float)], maxlen=35)

    @property
    def pos(self):
        return self.x[:2]

    @property
    def vel(self):
        return self.x[2:]

    def predecir(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:2]

    def corregir(self, centro, radio):
        y = np.asarray(centro) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        self.radio = 0.7 * self.radio + 0.3 * radio
        self.aciertos += 1
        self.perdidas = 0
        self.historia.append(self.x[:2].copy())


class Seguidor:
    def __init__(self, eps=0.16, max_dist=0.55, max_perdidas=6):
        self.eps = eps
        self.max_dist = max_dist
        self.max_perdidas = max_perdidas
        self.pistas: list[PistaKalman] = []

    def actualizar(self, pts_mundo: np.ndarray) -> list[PistaKalman]:
        detecciones = []
        for g in agrupar(pts_mundo, self.eps):
            extension = float(np.linalg.norm(g.max(0) - g.min(0)))
            if 0.03 <= extension <= 0.85:
                detecciones.append((g.mean(0), max(extension / 2, 0.08)))
        pred = np.array([p.predecir() for p in self.pistas]) if self.pistas else np.empty((0, 2))
        usadas_p, usadas_d = set(), set()
        if len(pred) and detecciones:
            costo = np.linalg.norm(pred[:, None] - np.array([d[0] for d in detecciones])[None], axis=2)
            for r, c in zip(*linear_sum_assignment(costo)):
                if costo[r, c] <= self.max_dist:
                    self.pistas[r].corregir(*detecciones[c])
                    usadas_p.add(r)
                    usadas_d.add(c)
        for i, p in enumerate(self.pistas):
            if i not in usadas_p:
                p.perdidas += 1
        self.pistas += [PistaKalman(*d) for j, d in enumerate(detecciones) if j not in usadas_d]
        self.pistas = [p for p in self.pistas if p.perdidas <= self.max_perdidas]
        return [p for p in self.pistas if p.aciertos >= 3]

    @staticmethod
    def es_dinamica(p: PistaKalman, slam, mascara_estatica) -> bool:
        """Se mueve de verdad Y no está sobre el mapa estático (los bordes "deslizan" al barrer)."""
        return p.aciertos >= 5 and np.linalg.norm(p.vel) > 0.15 and not slam.cerca_de_estatico(p.pos, mascara_estatica)
