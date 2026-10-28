"""Ronda completa simulada (N2): firmware real del Mega compilado en el PC, LiDAR simulado con error de
calibración, brazo/visión/compuertas simulados. Tarda ~2 min (la simulación corre en tiempo real).

    python3 -m tests.test_mision            # semilla 1
    python3 -m tests.test_mision 3 7        # otras canchas sorteadas

Comprueba: se ubica en el sector correcto, recoge solo maduros, los vacía en la caja roja,
no abandona granos y termina antes de los 8 min.
"""
import sys
import time

from cafebot.config import cargar
from cafebot.mision.mision import Mision
from cafebot.mundo import cancha
from cafebot.navegacion.sistema import SistemaNavegacion
from tools.mision import armar_sim


def ronda(semilla: int) -> int:
    cfg = cargar()
    cfg.mision.nivel = 2
    esc, mega, lidar, brazo, vision, depositos = armar_sim(cfg, semilla, t_agarre=1.0)
    vision.p_error = 0.0      # aquí se prueba la misión, no la visión (sigue habiendo NO_SE)
    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega)
    m = Mision(cfg, nav, lidar, mega, brazo, vision, depositos, log=lambda t: None)
    fin = time.monotonic() + 3
    while time.monotonic() < fin:
        nav.actualizar()
        time.sleep(0.02)
    m.iniciar()
    while not m.terminada:
        nav.actualizar()
        time.sleep(0.02)
    lidar.detener()
    mega.cerrar()
    pts = depositos.puntaje()
    print(f"semilla {semilla}: partida {esc.zona_robot} (estimada {m.res_inicio.zona}), roja {esc.zona_caja_roja}, "
          f"albercas {esc.albercas} | {m.tiempo():.0f} s | roja {len(depositos.en_roja)} azul {len(depositos.en_azul)} "
          f"abandonados {len(depositos.abandonados)} en robot {brazo.estado()} | PUNTAJE {pts}")
    print("   último evento:", m.eventos[-1])
    assert m.res_inicio.zona == esc.zona_robot, "sector de partida mal estimado"
    assert not depositos.abandonados, "granos fuera de las cajas"
    assert all(c == cancha.MADURO for c in depositos.en_roja), "algo que no es maduro en la caja roja"
    assert m.tiempo() < cfg.mision.duracion_s
    assert "TERMINADA" in m.eventos[-1], m.eventos[-1]
    return pts


def main():
    semillas = [int(s) for s in sys.argv[1:]] or [1]
    total = [ronda(s) for s in semillas]
    print(f"OK: misión en simulación | puntajes {total}")


if __name__ == "__main__":
    main()
