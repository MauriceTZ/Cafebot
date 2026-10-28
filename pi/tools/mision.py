"""Correr la misión de una ronda, sin interfaz gráfica.

    python3 -m tools.mision --sim                    # todo simulado: cancha sorteada, brazo y visión simulados
    python3 -m tools.mision --sim --nivel 3 --semilla 7
    python3 -m tools.mision                          # robot real: Mega + LD06, botón = ENTER, sin brazo ni visión
    python3 -m tools.mision --boton gpio             # botón de inicio real (cfg.mision.pin_boton)
    python3 -m tools.mision --vision fija            # SOLO TALLER: árbol cargado solo con maduros

Para verlo en el mapa usar la interfaz: python3 -m tools.navegacion --sim  y tecla K.
"""
from __future__ import annotations

import argparse
import time

import numpy as np

from cafebot.config import cargar
from cafebot.mision import interfaces as I
from cafebot.mision.mision import Mision
from cafebot.mundo import cancha
from cafebot.navegacion.sistema import SistemaNavegacion


def armar_sim(cfg, semilla=None, t_agarre=4.0, zona=None):
    """Mega (firmware real compilado en el PC), LiDAR, brazo, visión y compuertas simulados."""
    from cafebot.drivers.lidar_simulado import LD06Simulado
    from cafebot.drivers.mega import Mega
    from tests.sim_serial import SerialSimulado, compilar
    compilar()
    rng = np.random.default_rng(semilla)
    esc = cancha.Escenario.sorteo(rng)
    if zona:
        esc.zona_robot = zona
    # El capitán lo deja "centrado y mirando a los árboles": con algo de error
    inicio = np.array([cancha.ZONA_X_CENTRO + rng.normal(0, 0.02),
                       cancha.centros_zonas_beneficiadero()[esc.zona_robot - 1] + rng.normal(0, 0.02),
                       np.radians(rng.normal(0, 2.0))])
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    lidar = LD06Simulado(cfg.geometria.config_ld06(), esc.segmentos(), inicio,
                         lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th), semilla=semilla)
    real = lambda: tuple(lidar.real)
    c = cfg.mision
    brazo = I.BrazoSimulado(esc, real, (c.brazo_x, c.brazo_y, c.brazo_z), t_agarre=t_agarre, rng=rng)
    vision = I.VisionSimulada(esc, rng)
    depositos = I.DepositosSimulados(mega, c, brazo, esc, real)
    return esc, mega, lidar, brazo, vision, depositos


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--semilla", type=int, default=None, help="(sim) sorteo de la cancha y los granos")
    ap.add_argument("--zona", type=int, default=None, help="(sim) forzar el sector de partida")
    ap.add_argument("--t-agarre", type=float, default=4.0, help="(sim) segundos por agarre")
    ap.add_argument("--nivel", type=int, default=None)
    ap.add_argument("--boton", choices=("enter", "gpio", "ya"), default=None)
    ap.add_argument("--vision", choices=("nula", "fija"), default="nula", help="(real) aún no hay visión real")
    ap.add_argument("--imu", action="store_true")
    a = ap.parse_args()
    cfg = cargar()
    if a.nivel:
        cfg.mision.nivel = a.nivel
    if cfg.mision.nivel >= 3:
        cfg.mision.max_arboles = max(cfg.mision.max_arboles, 3)
        cfg.mision.recoger = tuple(sorted(set(cfg.mision.recoger) | {cancha.SOBREMADURO}))

    gyro = None
    if a.sim:
        esc, mega, lidar, brazo, vision, depositos = armar_sim(cfg, a.semilla, a.t_agarre, a.zona)
        print(f"Cancha sorteada: partida {esc.zona_robot}, caja roja {esc.zona_caja_roja}, "
              f"azul {esc.zona_caja_azul}, albercas {esc.albercas}")
    else:
        from cafebot.drivers.ld06 import LD06
        from cafebot.drivers.mega import Mega
        mega = Mega(cfg.puertos.mega)
        print("Conectando con el Mega...")
        mega.conectar()
        mega.telemetria(20)
        lidar = LD06(cfg.puertos.lidar, cfg.geometria.config_ld06())
        brazo = I.BrazoNulo()       # la secuencia de agarre real todavía no existe
        vision = I.VisionFija() if a.vision == "fija" else I.VisionNula()
        depositos = I.Depositos(mega, cfg.mision)
        if a.vision == "fija":
            print("¡OJO! visión FIJA: toma todo como MADURO. Solo para el taller.")
        if a.imu:
            from cafebot.drivers.imu import Giroscopio
            gyro = Giroscopio(cfg.puertos.imu_bus, cfg.puertos.imu_dir)
    boton = {"enter": I.BotonEnter, "ya": I.Boton}.get(a.boton or ("ya" if a.sim else "enter"))
    boton = I.BotonGPIO(cfg.mision.pin_boton) if a.boton == "gpio" else boton()

    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega, gyro)
    m = Mision(cfg, nav, lidar, mega, brazo, vision, depositos, boton)
    # Antes del botón: dejar que el SLAM arranque (el robot quieto)
    fin = time.monotonic() + 3
    while time.monotonic() < fin:
        nav.actualizar()
        time.sleep(0.02)
    m.iniciar()
    try:
        while not m.terminada:
            nav.actualizar()
            time.sleep(0.02)
    except KeyboardInterrupt:
        m.detener()
        m.esperar(5)
    print(f"\nTiempo usado: {m.tiempo():.0f} s | recogidos (en el robot ahora): {m.recogidos}")
    if a.sim:
        print(f"Robot: {brazo.estado()} | roja {depositos.en_roja} | azul {depositos.en_azul} | "
              f"abandonados {depositos.abandonados}")
        print(f"PUNTAJE (simulado): {depositos.puntaje()}")
    lidar.detener()
    mega.cerrar()


if __name__ == "__main__":
    main()
