"""Planificador por grafo: A* sobre el mapa estático del SLAM (portado del código antiguo).

Cada celda libre (a la resolución de planificación) es un nodo con 8 vecinos; el peso de una
arista crece cerca de los obstáculos. Después se "acorta" el camino con línea de vista.
"""
from __future__ import annotations

import heapq
import math

import numpy as np
from scipy.ndimage import distance_transform_edt


class Planificador:
    def __init__(self, slam, res_plan=0.08, radio_robot=0.19, seguridad=0.10, max_ajuste=0.8):
        self.slam = slam
        self.f = max(1, int(round(res_plan / slam.res)))
        self.res = self.f * slam.res
        self.radio = radio_robot
        self.seguridad = seguridad
        self.max_ajuste = max_ajuste
        self.permitido = None
        self.costo = None
        self._idx_ajuste = None
        self._dist_ajuste = None
        self.limpiar()

    def limpiar(self):
        self.ruta = None          # waypoints acortados
        self.ruta_cruda = None    # todas las celdas del A*
        self._acum = None
        self.largo = 0.0
        self.expandidos = 0
        self.msg = "Sin ruta"

    # ------------------------------------------------------------------ celdas
    def _a_celda(self, p):
        s = self.slam
        fx = int(np.floor(p[0] / s.res + s.centro))
        fy = int(np.floor(p[1] / s.res + s.centro))
        py, px = fy // self.f, fx // self.f
        if fx < 0 or fy < 0 or py >= self.permitido.shape[0] or px >= self.permitido.shape[1]:
            return None
        return py, px

    def _a_mundo(self, py, px):
        s = self.slam
        return np.array([(px * self.f + self.f / 2 - s.centro) * s.res, (py * self.f + self.f / 2 - s.centro) * s.res])

    def construir(self):
        s, f = self.slam, self.f
        n = (s.dim // f) * f
        m = n // f
        lo = s.log_odds[:n, :n]
        ocupado = s.mascara_estatica()[:n, :n].reshape(m, f, m, f).any(axis=(1, 3))
        libre = (lo < -0.25).reshape(m, f, m, f).mean(axis=(1, 3)) >= 0.5
        dist = distance_transform_edt(~ocupado) * self.res if ocupado.any() else np.full(ocupado.shape, 99.0)
        self.permitido = libre & ~ocupado & (dist >= self.radio)
        pen = np.clip((self.radio + self.seguridad - dist) / self.seguridad, 0, 1)
        self.costo = 1.0 + 6.0 * pen
        if self.permitido.any():
            d, idx = distance_transform_edt(~self.permitido, return_indices=True)
            self._dist_ajuste, self._idx_ajuste = d * self.res, idx

    def _ajustar(self, celda):
        """Lleva un punto a la celda libre más cercana (si está a menos de max_ajuste)."""
        if celda is None:
            return None
        if self.permitido[celda]:
            return celda
        if self._idx_ajuste is None or self._dist_ajuste[celda] > self.max_ajuste:
            return None
        return int(self._idx_ajuste[0][celda]), int(self._idx_ajuste[1][celda])

    # ------------------------------------------------------------------ A*
    def _astar(self, ini, fin):
        perm, costo = self.permitido.tolist(), self.costo.tolist()
        H, W = self.permitido.shape
        r2 = math.sqrt(2)
        movs = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, r2), (-1, 1, r2), (1, -1, r2), (1, 1, r2)]

        def h(y, x):
            dy, dx = abs(fin[0] - y), abs(fin[1] - x)
            return self.res * (max(dy, dx) + (r2 - 1) * min(dy, dx))

        g, padre, cerrado = {ini: 0.0}, {}, set()
        pila = [(h(*ini), 0.0, ini)]
        exp = 0
        while pila:
            _, gc, nodo = heapq.heappop(pila)
            if nodo in cerrado:
                continue
            cerrado.add(nodo)
            exp += 1
            if nodo == fin:
                camino = [nodo]
                while nodo in padre:
                    nodo = padre[nodo]
                    camino.append(nodo)
                return camino[::-1], gc, exp
            y, x = nodo
            for dy, dx, paso in movs:
                ny, nx = y + dy, x + dx
                if not (0 <= ny < H and 0 <= nx < W) or not perm[ny][nx]:
                    continue
                if dy and dx and not (perm[y + dy][x] and perm[y][x + dx]):
                    continue
                ng = gc + paso * self.res * 0.5 * (costo[y][x] + costo[ny][nx])
                if ng < g.get((ny, nx), math.inf):
                    g[(ny, nx)] = ng
                    padre[(ny, nx)] = nodo
                    heapq.heappush(pila, (ng + h(ny, nx), ng, (ny, nx)))
        return None, math.inf, exp

    def _linea_libre(self, a, b):
        n = max(abs(b[0] - a[0]), abs(b[1] - a[1])) * 2 + 2
        ys = np.rint(np.linspace(a[0], b[0], n)).astype(int)
        xs = np.rint(np.linspace(a[1], b[1], n)).astype(int)
        return bool(self.permitido[ys, xs].all())

    def _acortar(self, celdas):
        out, i = [celdas[0]], 0
        while i < len(celdas) - 1:
            j = len(celdas) - 1
            while j > i + 1 and not self._linea_libre(celdas[i], celdas[j]):
                j -= 1
            out.append(celdas[j])
            i = j
        return out

    # ------------------------------------------------------------------ API
    def planificar(self, ini_xy, fin_xy) -> bool:
        self.limpiar()
        self.construir()
        if not self.permitido.any():
            self.msg = "Mapa sin zonas libres suficientes (mueve el robot para mapear)"
            return False
        a = self._ajustar(self._a_celda(ini_xy))
        b = self._ajustar(self._a_celda(fin_xy))
        if a is None:
            self.msg = "Origen fuera de zona libre"
            return False
        if b is None:
            self.msg = "Destino inalcanzable o zona sin mapear"
            return False
        celdas, total, self.expandidos = self._astar(a, b)
        if celdas is None:
            self.msg = "No existe ruta"
            return False
        self.ruta_cruda = np.array([self._a_mundo(*c) for c in celdas])
        self.ruta = np.array([self._a_mundo(*c) for c in self._acortar(celdas)])
        if b == self._a_celda(fin_xy):          # destino libre: terminar EN el punto pedido, no en el centro
            self.ruta[-1] = fin_xy              # de su celda (hasta 5,7 cm de diferencia con celdas de 8 cm)
            self.ruta_cruda[-1] = fin_xy
        seg = np.linalg.norm(np.diff(self.ruta_cruda, axis=0), axis=1)
        self._acum = np.concatenate(([0.0], np.cumsum(seg)))
        self.largo = float(np.linalg.norm(np.diff(self.ruta, axis=0), axis=1).sum())
        self.msg = f"OK (costo {total:.2f})"
        return True

    def punto_aproximacion(self, lm, desde_xy, holgura=0.25, medio_largo_robot=0.15):
        """Punto frente a la cara del landmark que mira al robot, para quedar a `holgura` del
        objeto con el frente del robot. Devuelve (punto, rumbo_final) o (None, None)."""
        self.construir()
        n = lm.normal_hacia(desde_xy)
        d = lm.ancho / 2 + medio_largo_robot + holgura
        for desplazamiento in (0.0, 0.10, -0.10, 0.20, -0.20):      # probar un poco a los lados
            p = lm.pos + d * n + desplazamiento * lm.eje
            c = self._a_celda(p)
            if c is not None and self.permitido[c]:
                return p, math.atan2(-n[1], -n[0])                    # mirando al objeto
        return None, None

    def restante(self, xy):
        if self.ruta_cruda is None or len(self.ruta_cruda) == 0:
            return None
        d = np.linalg.norm(self.ruta_cruda - np.asarray(xy), axis=1)
        i = int(np.argmin(d))
        return float(self._acum[-1] - self._acum[i] + d[i])
