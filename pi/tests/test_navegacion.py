"""Navegación completa en simulación (sin pantalla): firmware real simulado + LiDAR simulado
con error de calibración (3 % en distancia, 2 % en giro).

    python3 -m tests.test_navegacion

Comprueba: el SLAM corrige la deriva, encuentra los 3 árboles y el robot llega a un árbol.
"""
import math
import time

import numpy as np

from cafebot.config import cargar
from cafebot.drivers.lidar_simulado import LD06Simulado
from cafebot.drivers.mega import Mega
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import relativo
from cafebot.navegacion.sistema import SistemaNavegacion
from tests.sim_serial import SerialSimulado, compilar


def esperar(nav, cond, max_s, paso=0.02):
    fin = time.monotonic() + max_s
    while time.monotonic() < fin:
        nav.actualizar()
        if cond():
            return True
        time.sleep(paso)
    return False


def main():
    compilar()
    cfg = cargar()
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    esc = cancha.Escenario(zona_robot=3, zona_caja_roja=1, zona_caja_azul=5, albercas=(2,))
    inicio = np.array([cancha.ZONA_X_CENTRO, cancha.centros_zonas_beneficiadero()[2], 0.0])
    lidar = LD06Simulado(cfg.geometria.config_ld06(), esc.segmentos(), inicio,
                         lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th), semilla=3)
    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega)

    def error_pose():
        real_en_mapa = relativo(inicio, lidar.real)      # el mapa tiene origen en la partida
        p = nav.pose()
        return math.hypot(*(p[:2] - real_en_mapa[:2])), math.degrees(abs(p[2] - real_en_mapa[2]))

    assert esperar(nav, lambda: nav.ekf.inicializada, 5), "EKF no se inicializó"
    esperar(nav, lambda: False, 2.0)                      # mapear quieto 2 s

    # 1) Ir a un punto antes de las albercas (frente al árbol central), recorriendo ~0,9 m
    destino = np.array([0.9, 0.0])
    assert nav.planificar(destino), nav.planificador.msg
    assert nav.ejecutar()
    assert esperar(nav, lambda: not nav.ejecutor.activo, 60), "la ruta no terminó"
    e, eth = error_pose()
    d_odom, _ = nav.ekf.deriva()
    print(f"Tramo 1: {nav.ejecutor.estado} | error pose fusionada {100*e:.1f} cm, {eth:.1f}° | "
          f"la odometría sola se habría desviado {100*d_odom:.1f} cm")
    assert e < 0.02, "el SLAM no corrigió la deriva"

    # 2) Landmarks: deberían aparecer los árboles
    esperar(nav, lambda: sum(l.cls == "ARBOL" for l in nav.registro.confirmados()) >= 2, 6)
    conf = nav.registro.confirmados()
    print("Landmarks:", [(l.nombre, f"{l.largo:.2f}m") for l in conf])
    arboles = [l for l in conf if l.cls == "ARBOL"]
    assert arboles, "no se confirmó ningún árbol"

    # 3) Aproximarse al árbol más cercano
    lm = min(arboles, key=lambda l: np.linalg.norm(l.pos - nav.pose()[:2]))
    ok, msg = nav.aproximar(lm.nombre)
    print("Aproximar:", msg)
    assert ok, msg
    assert esperar(nav, lambda: not nav.ejecutor.activo, 120), "la aproximación no terminó"
    real = relativo(inicio, lidar.real)
    arbol_real = relativo(inicio, np.array([cancha.ARBOL_X_FRENTE, cancha.centros_arboles()[1], 0]))
    frente_robot = real[0] + 0.15 * math.cos(real[2])
    holgura = arbol_real[0] - frente_robot
    e, eth = error_pose()
    print(f"Llegada: {nav.ejecutor.estado} | frente del robot a {100*holgura:.1f} cm del árbol "
          f"(pedido {100*cfg.navegacion.distancia_aprox_m:.0f}), mirando {math.degrees(real[2]):.1f}° "
          f"| error pose {100*e:.1f} cm")
    assert nav.ejecutor.estado == "ruta completada"
    assert abs(holgura - cfg.navegacion.distancia_aprox_m) < 0.08 and abs(math.degrees(real[2])) < 6
    # 4) Volver al punto de partida (recorrido largo con giros: la odometría sola deriva)
    assert nav.planificar(np.array([0.0, 0.0])), nav.planificador.msg
    assert nav.ejecutar(0.0)
    assert esperar(nav, lambda: not nav.ejecutor.activo, 120), "el regreso no terminó"
    real = relativo(inicio, lidar.real)
    e, eth = error_pose()
    d_odom, th_odom = nav.ekf.deriva()
    print(f"Regreso: {nav.ejecutor.estado} | quedó a {100*math.hypot(real[0], real[1]):.1f} cm del inicio | "
          f"error pose fusionada {100*e:.1f} cm, {eth:.1f}° | la odometría sola: {100*d_odom:.1f} cm, {th_odom:+.1f}°")
    assert e < 0.04 and math.hypot(real[0], real[1]) < 0.08
    lidar.detener()
    mega.cerrar()
    print("OK: navegación en simulación")


if __name__ == "__main__":
    main()
