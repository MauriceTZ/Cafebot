"""Las dos cajas del MISMO color (mision.cajas), sin simulador.   python3 -m tests.test_cajas

  1. "cercana": los maduros van a la caja más cercana al volver y los sobremaduros a la otra; con cajas en más de
     dos sectores (algo que el LiDAR vio mal), no apuesta. No pierde tiempo mirando el color.
  2. "guardar": no entrega (no abre compuertas ni se mueve) y no reserva tiempo para el viaje.
  3. "colores" (lo de siempre): reserva tiempo para mirar el color; un modo desconocido se rechaza.
  4. Cuánto vale cada opción (puntaje esperado, según el formato de las rondas).
"""
import math
from itertools import product

import numpy as np

from cafebot.config import cargar
from cafebot.mision.mision import Mision
from cafebot.mundo import cancha


class Nav:
    def pose(self):
        return np.zeros(3)

    def detener(self):
        pass


class Depositos:
    def __init__(self):
        self.vaciados = []

    def vaciar(self, k):
        self.vaciados.append(k)

    def cerrar_todo(self):
        pass


class Prohibido:
    """Mega / LiDAR / visión que no se deben usar en estas pruebas."""

    def __getattr__(self, nombre):
        raise AssertionError(f"no debía usar {nombre}")


def mision(modo, y_robot=2.5, zonas=(1, 4), nivel=3):
    cfg = cargar()
    cfg.mision.cajas, cfg.mision.nivel = modo, nivel
    m = Mision(cfg, Nav(), Prohibido(), Prohibido(), Prohibido(), Prohibido(), Depositos(), log=lambda t: None)
    m.inicio = np.array([2.0, y_robot, math.pi])          # volviendo de los árboles, mirando al beneficiadero
    m.zonas_caja = set(zonas)
    m.res_inicio = None
    m.recogidos = {cancha.MADURO: 3, cancha.SOBREMADURO: 2}
    m._actualizar_cajas = lambda: None                    # (necesita el SLAM)
    return m


def prueba_cercana():
    m = mision("cercana", y_robot=2.5)                    # sectores: y = 0,37 / 1,08 / 1,80 / 2,52 / 3,23
    m._asignar_por_cercania()
    assert m.cajas == {"ROJA": 4, "AZUL": 1}, m.cajas
    m = mision("cercana", y_robot=0.6)
    m._asignar_por_cercania()
    assert m.cajas == {"ROJA": 1, "AZUL": 4}, m.cajas
    m = mision("cercana", zonas=(5,))                     # solo una caja vista: los maduros a esa
    m._asignar_por_cercania()
    assert m.cajas == {"ROJA": 5}, m.cajas
    m = mision("cercana", zonas=(1, 3, 4))                # algo raro: no apuesta
    m._asignar_por_cercania()
    assert m.cajas == {}, m.cajas
    t_colores = mision("colores")._reserva_entrega()
    t_cercana = mision("cercana")._reserva_entrega()
    assert t_cercana < t_colores - 20, (t_cercana, t_colores)
    print(f"  maduros a la más cercana (desde y=2,5: sector 4; desde y=0,6: sector 1), sobremaduros a la otra; con "
          f"3 sectores no apuesta | tiempo reservado para entregar {t_cercana:.0f} s (con colores {t_colores:.0f} s)")


def prueba_guardar():
    m = mision("guardar")
    assert m._reserva_entrega() == 0.0
    m._entregar()                                         # Prohibido: si intentara moverse o mirar, falla
    assert m.depositos.vaciados == [] and m.cajas == {}
    try:
        mision("al_azar")
        raise AssertionError("debía rechazar el modo")
    except ValueError:
        pass
    print("  guardar: no se mueve, no abre compuertas, no reserva tiempo; un modo desconocido se rechaza")


def mejores(puntajes, k):
    return sum(sorted(puntajes, reverse=True)[:k])


def esperado(por_ronda, rondas, k):
    """Puntaje esperado de la suma de las k mejores de `rondas`, con por_ronda = [(puntaje, probabilidad)]."""
    total = 0.0
    for caso in product(por_ronda, repeat=rondas):
        p = np.prod([c[1] for c in caso])
        total += p * mejores([c[0] for c in caso], k)
    return total


def prueba_puntaje():
    """3 maduros (y 0 o 2 sobremaduros) por ronda, bien clasificados."""
    print("  puntaje esperado (clasificatoria = 2 mejores de 4 | final = mejor de 3):")
    for m_, s_ in ((3, 0), (3, 2)):
        guardar = 3 * m_ + s_
        apuesta = 7 * m_ + 5 * s_
        a = [(apuesta, 0.5), (-apuesta, 0.5)]
        g = [(guardar, 1.0)]
        cla_a, cla_g = esperado(a, 4, 2), esperado(g, 4, 2)
        fin_a, fin_g = esperado(a, 3, 1), esperado(g, 3, 1)
        print(f"    {m_} maduros + {s_} sobremaduros: una ronda: guardar {guardar}, apostar ±{apuesta} (promedio 0) | "
              f"clasificatoria: guardar {cla_g:.0f}, apostar {cla_a:.1f} | final: guardar {fin_g:.0f}, "
              f"apostar {fin_a:.1f}")
        assert abs(cla_a - 1.25 * apuesta) < 1e-9 and abs(fin_a - 0.75 * apuesta) < 1e-9
        assert cla_g == 2 * guardar and fin_g == guardar


def main():
    print("1) Cajas iguales: a la más cercana")
    prueba_cercana()
    print("2) Cajas iguales: guardar")
    prueba_guardar()
    print("3) Valor de cada opción")
    prueba_puntaje()
    print("OK: cajas")


if __name__ == "__main__":
    main()
