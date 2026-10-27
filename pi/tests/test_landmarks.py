"""Clasificación de landmarks con formas de la cancha (sin hardware).
Correr desde pi/:  python3 -m tests.test_landmarks
"""
import numpy as np

from cafebot.navegacion.landmarks import clasificar


def rot(a):
    a = np.radians(a)
    return np.array([[np.cos(a), -np.sin(a)], [np.sin(a), np.cos(a)]])


def main():
    rng = np.random.default_rng(0)
    ruido = lambda p: p + rng.normal(0, 0.008, p.shape)
    t = np.linspace(0, 0.45, 25)
    caja_L = np.vstack([np.column_stack((t, 0 * t)), np.column_stack((0 * t, t))])
    pie = np.column_stack((np.zeros(30), np.linspace(0, 0.30, 30)))
    aleta = np.column_stack((np.linspace(0, 0.2, 15), np.full(15, 0.15)))
    alberca = np.vstack([np.column_stack((np.zeros(50), np.linspace(0, 0.72, 50))),
                         np.column_stack((np.linspace(0, 0.2, 10), np.zeros(10)))])
    casos = [("CAJA", caja_L), ("CAJA", np.column_stack((np.zeros(30), np.linspace(0, 0.45, 30)))),
             ("ARBOL", pie), ("ARBOL", np.vstack([pie, aleta])), ("ALBERCA", alberca),
             ("PARED", np.column_stack((np.zeros(80), np.linspace(0, 1.5, 80)))),
             ("OBSTACULO", np.column_stack((np.zeros(10), np.linspace(0, 0.12, 10))))]
    for esperado, pts in casos:
        for a in (0, 20, 45, 70):
            c = clasificar(ruido(pts @ rot(a).T))
            assert c["cls"] == esperado, (esperado, a, c["cls"], round(c["largo"], 2), round(c["ancho"], 2))
    print("OK: landmarks (caja de frente y de esquina, árbol con y sin aleta, alberca, pared, objeto)")


if __name__ == "__main__":
    main()
