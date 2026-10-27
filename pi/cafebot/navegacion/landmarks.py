"""Landmarks: agrupar el mapa estático, clasificar cada grupo por su FORMA y darle un nombre estable.

Cambio clave respecto del código antiguo: allí un ÁRBOL era un tronco CILÍNDRICO (ajuste de
círculo). En la cancha LARC el LiDAR, a ~8 cm del suelo, ve el PIE del árbol: una tabla plana de
~30 cm de ancho (con un soporte triangular detrás). Aquí se clasifica por largo/ancho del grupo:

    ÁRBOL     ~0,30 m de largo        (pie del tablero)
    CAJA      ~0,45 m (lado o esquina en L)
    ALBERCA   ~0,72 m de largo (o 0,20 si se ve de punta)
    PARED     > 1 m (bordes, gente alineada, mesas)
    OBJ       cualquier otra cosa

Los rangos están en CLASES y se ajustan con lo que se mida en la cancha real.
"""
from __future__ import annotations

import json

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial import cKDTree

# (clase, largo mínimo, largo máximo, ancho máximo) en metros                         [AJUSTAR]
CLASES = [
    ("ARBOL", 0.20, 0.38, 0.26),
    ("CAJA", 0.38, 0.56, 0.56),
    ("ALBERCA", 0.62, 0.85, 0.30),
]
LARGO_PARED = 1.0
PREFIJO = {"ARBOL": "ARBOL", "CAJA": "CAJA", "ALBERCA": "ALBERCA", "OBSTACULO": "OBJ"}


def agrupar(pts: np.ndarray, eps: float = 0.09, min_pts: int = 3) -> list[np.ndarray]:
    """Agrupamiento euclidiano (como DBSCAN con min_samples=1, más un mínimo por grupo)."""
    if len(pts) < min_pts:
        return []
    padre = np.arange(len(pts))

    def raiz(i):
        while padre[i] != i:
            padre[i] = padre[padre[i]]
            i = padre[i]
        return i

    for a, b in cKDTree(pts).query_pairs(eps):
        ra, rb = raiz(a), raiz(b)
        if ra != rb:
            padre[ra] = rb
    etiquetas = np.array([raiz(i) for i in range(len(pts))])
    grupos = [pts[etiquetas == e] for e in np.unique(etiquetas)]
    return [g for g in grupos if len(g) >= min_pts]


def rectangulo_envolvente(pts: np.ndarray) -> tuple[float, float, np.ndarray]:
    """(lado largo, lado corto, dirección del lado largo) del rectángulo de menor PERÍMETRO que
    envuelve los puntos (el de menor área elige la diagonal en una L). Mejor que el eje principal para una caja vista de esquina (forma de L):
    su eje principal queda en diagonal y "mide" 0,64 m en vez de 0,45."""
    q = pts - pts.mean(0)
    mejor = None
    for a in np.radians(np.arange(0.0, 90.0, 3.0)):
        u = np.array([np.cos(a), np.sin(a)])
        v = np.array([-u[1], u[0]])
        lu, lv = float(np.ptp(q @ u)), float(np.ptp(q @ v))
        if mejor is None or lu + lv < mejor[0]:
            mejor = (lu + lv, lu, lv, u, v)
    _, lu, lv, u, v = mejor
    return (lu, lv, u) if lu >= lv else (lv, lu, v)


def clasificar(pts: np.ndarray) -> dict:
    centro = pts.mean(0)
    largo, ancho, eje = rectangulo_envolvente(pts)
    info = {"centro": centro, "largo": largo, "ancho": ancho, "eje": eje, "n": len(pts),
            "radio": max(largo, ancho) / 2, "cls": "OBSTACULO"}
    if largo >= LARGO_PARED:
        info["cls"] = "PARED"
        return info
    for cls, lmin, lmax, amax in CLASES:
        if lmin <= largo <= lmax and ancho <= amax:
            info["cls"] = cls
            break
    return info


def detectar(slam, eps: float = 0.09) -> list[dict]:
    """Grupos del mapa estático del SLAM, ya clasificados."""
    return [clasificar(g) for g in agrupar(slam.puntos_estaticos(), eps)]


class Landmark:
    def __init__(self, c: dict):
        self.pos = np.array(c["centro"], float)
        self.eje = np.array(c["eje"], float)
        self.largo = c["largo"]
        self.ancho = c["ancho"]
        self.radio = c["radio"]
        self.cls = None          # clase confirmada
        self.nombre = None
        self.cls_candidata = c["cls"]
        self.racha = 1
        self.obs = 1
        self.perdido = 0

    @property
    def confirmado(self) -> bool:
        return self.nombre is not None

    def normal_hacia(self, desde_xy) -> np.ndarray:
        """Normal de la cara del objeto que mira hacia `desde_xy` (unitaria)."""
        n = np.array([-self.eje[1], self.eje[0]])
        return n if np.dot(np.asarray(desde_xy) - self.pos, n) >= 0 else -n


class RegistroLandmarks:
    """Nombres estables (ARBOL_1, CAJA_2, ...) asociados por posición en el mapa (asignación húngara)."""

    def __init__(self, dist_asoc=0.35, n_confirmar=3, max_perdido=30):
        self.items: list[Landmark] = []
        self.contadores: dict[str, int] = {}
        self.dist_asoc = dist_asoc
        self.n_confirmar = n_confirmar
        self.max_perdido = max_perdido

    def _nombrar(self, lm, cls):
        n = self.contadores.get(cls, 0) + 1
        self.contadores[cls] = n
        lm.cls, lm.nombre = cls, f"{PREFIJO[cls]}_{n}"

    def _observar(self, lm, c):
        a = 0.3
        lm.perdido = 0
        lm.obs += 1
        lm.pos = (1 - a) * lm.pos + a * c["centro"]
        eje = c["eje"] if np.dot(c["eje"], lm.eje) >= 0 else -c["eje"]
        lm.eje = (1 - a) * lm.eje + a * eje
        lm.eje /= np.linalg.norm(lm.eje)
        lm.largo = (1 - a) * lm.largo + a * c["largo"]
        lm.ancho = (1 - a) * lm.ancho + a * c["ancho"]
        lm.radio = (1 - a) * lm.radio + a * c["radio"]
        if c["cls"] == lm.cls_candidata:
            lm.racha += 1
        else:
            lm.cls_candidata, lm.racha = c["cls"], 1
        if lm.racha >= self.n_confirmar and lm.cls_candidata != lm.cls:
            self._nombrar(lm, lm.cls_candidata)

    def actualizar(self, grupos: list[dict], expirar: bool = True):
        grupos = [g for g in grupos if g["cls"] != "PARED"]
        usados_l, usados_g = set(), set()
        if self.items and grupos:
            costo = np.linalg.norm(np.array([l.pos for l in self.items])[:, None] -
                                   np.array([g["centro"] for g in grupos])[None], axis=2)
            for r, c in zip(*linear_sum_assignment(costo)):
                if costo[r, c] <= self.dist_asoc:
                    self._observar(self.items[r], grupos[c])
                    usados_l.add(r)
                    usados_g.add(c)
        for i, lm in enumerate(self.items):
            if i not in usados_l:
                lm.perdido += 1
        self.items += [Landmark(g) for j, g in enumerate(grupos) if j not in usados_g]
        if expirar:
            self.items = [l for l in self.items if l.perdido <= (self.max_perdido if l.confirmado else 3)]

    def confirmados(self) -> list[Landmark]:
        return sorted([l for l in self.items if l.confirmado], key=lambda l: (l.cls, int(l.nombre.rsplit("_", 1)[1])))

    def buscar(self, nombre) -> Landmark | None:
        return next((l for l in self.items if l.nombre == nombre), None)

    def guardar(self, ruta="landmarks.json"):
        datos = {"contadores": self.contadores,
                 "items": [{"nombre": l.nombre, "cls": l.cls, "pos": l.pos.tolist(), "eje": l.eje.tolist(),
                            "largo": l.largo, "ancho": l.ancho, "radio": l.radio} for l in self.items if l.confirmado]}
        with open(ruta, "w") as f:
            json.dump(datos, f, indent=2)

    def cargar(self, ruta="landmarks.json") -> bool:
        try:
            with open(ruta) as f:
                d = json.load(f)
            items = []
            for it in d["items"]:
                lm = Landmark({"centro": np.array(it["pos"]), "eje": np.array(it["eje"]), "largo": it["largo"],
                               "ancho": it["ancho"], "radio": it["radio"], "cls": it["cls"]})
                lm.cls, lm.nombre, lm.racha = it["cls"], it["nombre"], self.n_confirmar
                items.append(lm)
            self.contadores = {k: int(v) for k, v in d["contadores"].items()}
        except (OSError, ValueError, KeyError, TypeError):
            return False
        self.items = items
        return True
