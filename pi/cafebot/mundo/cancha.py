"""Geometría de la cancha LARC OPEN (reglamento, Apéndices A y B). Unidades: METROS.

Sistema del mundo (vista superior):
    x = 0 en el borde exterior del beneficiadero  ->  x = 3,0 en el borde exterior del cultivo
    y = 0 en un costado                            ->  y = 3,6 en el otro
    θ = 0 mirando hacia los árboles (+x), antihorario positivo.

          y=3,6 ┌──────┬────────────────┬──┬────────────────┬─────┐
                │ Z5   │                │A5│                │  C  │
                │ Z4   │                │A4│                │     │
                │ Z3   │  zona interm.  │A3│  zona interm.  │  B  │
                │ Z2   │                │A2│                │     │
                │ Z1   │                │A1│                │  A  │
          y=0   └──────┴────────────────┴──┴────────────────┴─────┘
               x=0   0,75             1,55 1,75          2,55   3,0
            beneficiadero              albercas              cultivo

El LD06 va a ~8 cm del suelo: a esa altura ve el PIE de cada árbol (tablero de 300 mm
de ancho hasta 150 mm de alto, Apéndice B), los costados de las albercas (90 mm) y de
las cajas (90 mm). Medidas marcadas [MEDIR] conviene confirmarlas en la cancha real.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

LINEA = 0.019
LARGO_X = 3.0
LARGO_Y = 3.6

# Beneficiadero: 5 sectores de 712 x 697 mm interiores
BENEF_X_INT = 0.712
BENEF_X = BENEF_X_INT + 2 * LINEA            # 0,750
SECTOR_BENEF_Y = (LARGO_Y - 6 * LINEA) / 5   # 0,6972
# Albercas: franja de 200 mm a 800 mm del beneficiadero, 5 posiciones de 720 mm
ALBERCA_X0 = BENEF_X + 0.800                 # 1,550
ALBERCA_X1 = ALBERCA_X0 + 0.200              # 1,750
ALBERCA_LARGO_Y = 0.720
# Cultivo: 450 mm (412 interiores), 3 sectores de ~1174,5 mm
CULTIVO_X0 = ALBERCA_X1 + 0.800              # 2,550 (línea entre zona intermedia y cultivo)
SECTORES_CULTIVO_Y = (1.1745, 1.175, 1.1745)
# Árbol: tablero centrado en su sector, a 100 mm de la línea más cercana al centro
ARBOL_X_FRENTE = CULTIVO_X0 + LINEA + 0.100  # cara delantera del tablero   [MEDIR]
ARBOL_ANCHO_PIE = 0.300                      # ancho del pie a la altura del LiDAR
ARBOL_ESPESOR = 0.018                        # tablero de madera            [MEDIR]
ARBOL_ALETA = 0.200                          # soporte triangular trasero   [MEDIR]
CAJA_LADO = 0.450
# Soportes de los granos (Apéndice B): cara superior a 390 / 540 / 690 mm; el centro de la
# esfera de 42 mm queda ~17 mm más arriba (hueco de 25 mm). Separación entre centros 120 mm.
ALTURA_SOPORTE = {"baja": 0.390, "media": 0.540, "alta": 0.690}
CENTRO_SOBRE_SOPORTE = 0.017
GRANO_DELANTE_M = 0.035        # centro del grano por delante de la cara del tablero   [MEDIR]
COPA_DESPLAZADA_M = 0.0        # centro de los soportes respecto del pie (+ = izquierda vista
                               # desde el robot). La figura 9 sugiere hasta ~75 mm.       [MEDIR]
_OFFSETS = {"baja": [0.24, 0.12, 0.0, -0.12, -0.24],
            "media": [0.30, 0.18, 0.06, -0.06, -0.18, -0.30],
            "alta": [0.24, 0.12, 0.0, -0.12, -0.24]}
FILAS = ("baja", "media", "alta")
MADURO, SOBREMADURO, VERDE = "MADURO", "SOBREMADURO", "VERDE"
COLORES = {MADURO: ("rojo", "naranjo", "amarillo"), SOBREMADURO: ("azul", "negro"), VERDE: ("verde",)}


@dataclass(frozen=True)
class Rect:
    cx: float
    cy: float
    ancho_x: float
    ancho_y: float

    def segmentos(self) -> np.ndarray:
        x0, x1 = self.cx - self.ancho_x / 2, self.cx + self.ancho_x / 2
        y0, y1 = self.cy - self.ancho_y / 2, self.cy + self.ancho_y / 2
        return np.array([[x0, y0, x1, y0], [x1, y0, x1, y1], [x1, y1, x0, y1], [x0, y1, x0, y0]])

    def contiene(self, x: np.ndarray, y: np.ndarray, margen: float = 0.0) -> np.ndarray:
        return ((np.abs(x - self.cx) <= self.ancho_x / 2 + margen) &
                (np.abs(y - self.cy) <= self.ancho_y / 2 + margen))


def centros_zonas_beneficiadero() -> np.ndarray:
    """y del centro de los sectores 1..5 del beneficiadero."""
    return np.array([LINEA + SECTOR_BENEF_Y / 2 + k * (SECTOR_BENEF_Y + LINEA) for k in range(5)])


ZONA_X_CENTRO = LINEA + BENEF_X_INT / 2      # 0,375


def centros_arboles() -> np.ndarray:
    ys, y = [], LINEA
    for s in SECTORES_CULTIVO_Y:
        ys.append(y + s / 2)
        y += s + LINEA
    return np.array(ys)


def rect_alberca(pos: int) -> Rect:
    """pos = 1..5"""
    return Rect((ALBERCA_X0 + ALBERCA_X1) / 2, (pos - 0.5) * ALBERCA_LARGO_Y, ALBERCA_X1 - ALBERCA_X0, ALBERCA_LARGO_Y)


def rect_caja(zona: int) -> Rect:
    """zona = 1..5 del beneficiadero; la caja va centrada en el sector."""
    return Rect(ZONA_X_CENTRO, centros_zonas_beneficiadero()[zona - 1], CAJA_LADO, CAJA_LADO)


def segmentos_arbol(y_centro: float) -> np.ndarray:
    """Pie del tablero (rectángulo delgado) + aleta trasera, a la altura del LiDAR."""
    tablero = Rect(ARBOL_X_FRENTE + ARBOL_ESPESOR / 2, y_centro, ARBOL_ESPESOR, ARBOL_ANCHO_PIE).segmentos()
    aleta = np.array([[ARBOL_X_FRENTE + ARBOL_ESPESOR, y_centro, ARBOL_X_FRENTE + ARBOL_ESPESOR + ARBOL_ALETA, y_centro]])
    return np.vstack([tablero, aleta])


def segmentos_estaticos() -> np.ndarray:
    """Lo que está SIEMPRE en el mismo lugar y el LiDAR ve: los 3 árboles."""
    return np.vstack([segmentos_arbol(y) for y in centros_arboles()])


@dataclass(frozen=True)
class Soporte:
    arbol: int          # 1..3 (A, B, C del reglamento, de y menor a mayor)
    fila: str           # baja / media / alta
    col: int            # 0 = la más a la IZQUIERDA vista desde el robot (y mayor)
    x: float            # centro del grano en el marco de la cancha (m)
    y: float
    z: float            # altura del centro del grano sobre el suelo

    @property
    def clave(self) -> tuple:
        return (self.arbol, self.fila, self.col)


def soportes(arbol: int) -> list[Soporte]:
    """Los 16 soportes del árbol `arbol` (1..3) con la posición del centro de cada grano."""
    yc = centros_arboles()[arbol - 1] + COPA_DESPLAZADA_M
    out = []
    for fila in FILAS:
        for col, dy in enumerate(_OFFSETS[fila]):
            out.append(Soporte(arbol, fila, col, ARBOL_X_FRENTE - GRANO_DELANTE_M, yc + dy,
                               ALTURA_SOPORTE[fila] + CENTRO_SOBRE_SOPORTE))
    return out


def sorteo_granos(rng: np.random.Generator) -> dict:
    """{(arbol, fila, col): (categoría, color)} según la Tabla 2 del reglamento."""
    granos = {}
    for arbol in (1, 2, 3):
        for fila in FILAS:
            if fila == "alta":
                cats = [VERDE] + [MADURO] * 3 + [SOBREMADURO]
            elif fila == "media":
                cats = [VERDE] * 2 + [MADURO] * 2 + [SOBREMADURO] * 2
            else:
                v = int(rng.integers(2, 4))
                cats = [VERDE] * v + [MADURO] + [SOBREMADURO] * (4 - v)
            rng.shuffle(cats)
            for col, c in enumerate(cats):
                granos[(arbol, fila, col)] = (c, str(rng.choice(COLORES[c])))
    return granos


def puntaje(en_robot: list[str], en_roja: list[str], en_azul: list[str], abandonados: list[str]) -> int:
    """Puntaje de una ronda (sección 8). Cada lista contiene categorías de granos."""
    p = 0
    for c in en_robot:
        p += {MADURO: 3, SOBREMADURO: 1, VERDE: -7}[c]
    for c in en_roja:
        p += {MADURO: 7, SOBREMADURO: -5, VERDE: -7}[c]
    for c in en_azul:
        p += {MADURO: -7, SOBREMADURO: 5, VERDE: -7}[c]
    p -= 7 * len(abandonados)
    return p


@dataclass
class Escenario:
    """Una ronda: dónde parte el robot, dónde están las cajas y las albercas."""
    zona_robot: int
    zona_caja_roja: int
    zona_caja_azul: int
    albercas: tuple[int, ...]   # 0, 1 o 2 posiciones (1..5)
    granos: dict = field(default_factory=dict)   # {(arbol, fila, col): (categoría, color)}

    def segmentos(self) -> np.ndarray:
        seg = [segmentos_estaticos()]
        seg += [rect_alberca(p).segmentos() for p in self.albercas]
        seg += [rect_caja(self.zona_caja_roja).segmentos(), rect_caja(self.zona_caja_azul).segmentos()]
        return np.vstack(seg)

    @staticmethod
    def sorteo(rng: np.random.Generator) -> "Escenario":
        """Como el reglamento: software para robot/cajas, dado para cada alberca (6 = no hay)."""
        z = rng.permutation(5)[:3] + 1
        a1, a2 = rng.integers(1, 7), rng.integers(1, 7)
        alb = tuple(sorted({a for a in (a1, a2) if a != 6})) if a1 != a2 else ((a1,) if a1 != 6 else ())
        return Escenario(int(z[0]), int(z[1]), int(z[2]), tuple(int(a) for a in alb), sorteo_granos(rng))
