"""Correr la misión de una ronda, sin interfaz gráfica.

    python3 -m tools.mision --sim                    # todo simulado: cancha sorteada, brazo y visión simulados
    python3 -m tools.mision --sim --brazo            # HÍBRIDA: ronda simulada con el BRAZO REAL (el robot quieto en la
                                                     #   mesa; el brazo hace cada agarre en el aire: docs/mision.md)
    python3 -m tools.mision --sim --nivel 3 --semilla 7
    python3 -m tools.mision                          # robot real: Mega + LD06, botón = ENTER, sin brazo ni visión
    python3 -m tools.mision --brazo --vision camara  # robot real COMPLETO (brazo XL430 + cámara)
    python3 -m tools.mision --boton gpio             # botón de inicio real (cfg.mision.pin_boton)
    python3 -m tools.mision --brazo --vision fija    # SOLO TALLER: árbol cargado solo con maduros
    python3 -m tools.mision ... --cajas cercana      # las dos cajas del mismo color (ver config: mision.cajas)
    python3 -m tools.mision --brazo --vision camara --nivel 1 --max-granos 1   # PRUEBA de cosecha: sale, va al
                                                     #   árbol, foto, línea del cultivo, un agarre y termina
    python3 -m tools.mision --brazo --vision camara --reducida --max-granos 1  # lo mismo en una cancha REDUCIDA:
                                                     #   parte mirando al árbol (<= 90 cm), sin el mapa de la cancha

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


def armar_reales(cfg, usar_brazo: bool, vision: str, mega, camara=None):
    """Brazo / visión / compuertas reales según las opciones (lo que no se pide queda nulo).
    camara: una CamaraCompartida si otra parte (la vista de tools.navegacion) también la usa."""
    if usar_brazo:
        from cafebot.arm.brazo import BrazoXL430
        from tools.brazo import buscar_u2d2
        try:
            puerto = buscar_u2d2(cfg.puertos.u2d2)
        except SystemExit as e:
            print(f"[brazo] {e}")
            puerto = cfg.puertos.u2d2
        brazo = BrazoXL430(cfg, puerto)
    else:
        brazo = I.BrazoNulo()
    if vision == "camara":
        from cafebot.vision.vision import VisionCamara
        vis = VisionCamara(cfg, camara=camara)
    elif vision == "fija":
        print("¡OJO! visión FIJA: toma todo como MADURO. Solo para el taller.")
        vis = I.VisionFija()
    else:
        vis = I.VisionNula()
    if cfg.compuertas.tipo == "dxl":           # 4 nov: Dynamixel 12 / 13 en el bus del brazo
        from cafebot.arm.compuertas import DepositosDXL, abrir_bus
        if usar_brazo:
            def bus_fn():                       # el MISMO bus del brazo: el puerto del U2D2 se abre una sola vez
                if brazo.bus is None:
                    try:
                        brazo.conectar()
                    except IOError as e:        # aunque falte un servo del brazo, las compuertas pueden servir
                        print(f"[brazo] {e}")
                if brazo.bus is None:
                    raise IOError("no se pudo abrir el U2D2")
                return brazo.bus
        else:
            bus_fn = lambda: abrir_bus(cfg)
        depositos = DepositosDXL(cfg, bus_fn)
    else:
        depositos = I.Depositos(mega, cfg.mision)
    return brazo, vis, depositos


def brazo_hibrido(cfg, brazo_sim, log=print):
    """El brazo REAL (U2D2) haciendo los agarres de una ronda SIMULADA (interfaces.BrazoHibrido)."""
    from cafebot.arm.brazo import BrazoXL430
    from tools.brazo import buscar_u2d2
    try:
        puerto = buscar_u2d2(cfg.puertos.u2d2)
    except SystemExit as e:
        log(f"[brazo] {e}")
        puerto = cfg.puertos.u2d2
    log("PRUEBA HÍBRIDA: ronda simulada con el BRAZO REAL. El robot quieto (no se mueve: el Mega es simulado), con "
        "espacio libre adelante, a los lados y atrás (la base gira hasta las canastas). Ctrl+C / ESC lo detiene "
        "después del movimiento en curso.")
    return I.BrazoHibrido(BrazoXL430(cfg, puerto, log=log), brazo_sim, log=log)


def armar_sim(cfg, semilla=None, t_agarre=4.0, zona=None, frente_arbol=None):
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
    if frente_arbol is not None:     # --reducida: frente al árbol 2, el frente a `frente_arbol` m del pie
        inicio = np.array([cancha.ARBOL_X_FRENTE - frente_arbol - cfg.geometria.largo / 2 + rng.normal(0, 0.01),
                           cancha.centros_arboles()[1] + rng.normal(0, 0.03), np.radians(rng.normal(0, 3.0))])
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    lidar = LD06Simulado(cfg.geometria.config_ld06(), esc.segmentos(), inicio,
                         lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th), semilla=semilla)
    real = lambda: tuple(lidar.real)
    from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
    mega.ir_sim = IRSimulado(mega, cfg.ir.posiciones, cancha.lineas_cancha(),
                             lambda: lidar.pose_real_ahora(pose_mega_extrapolada(mega)), semilla=semilla).iniciar()
    c = cfg.mision
    try:        # el alcance con la cinemática y el plan de agarre del brazo de verdad (sin bus: solo calcula)
        from cafebot.arm.brazo import BrazoXL430
        alcanzable = BrazoXL430(cfg, log=lambda t: None).alcanzable
    except Exception as e:
        print(f"[AVISO] alcance del brazo simulado aproximado (esfera de 38 cm): {e}")
        alcanzable = None
    brazo = I.BrazoSimulado(esc, real, (c.brazo_x, c.brazo_y, c.brazo_z), t_agarre=t_agarre, rng=rng,
                            alcanzable_fn=alcanzable)
    # Como la cámara real: los soportes fuera de la foto salen FUERA y desde la partida no ve las cajas
    vision = I.VisionSimulada(esc, rng, cfg_camara=cfg.camara, ve_cajas_al_partir=False)
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
    ap.add_argument("--brazo", action="store_true",
                    help="usar el brazo XL430 real (con --sim: prueba híbrida, ronda simulada con el brazo de verdad)")
    ap.add_argument("--vision", choices=("nula", "camara", "fija"), default="nula",
                    help="(real) camara = la real; nula = no toca granos; fija = todo MADURO (solo taller)")
    ap.add_argument("--imu", action="store_true")
    ap.add_argument("--max-granos", type=int, default=None,
                    help="PRUEBA: terminar al recoger tantos granos (p. ej. 1 = un agarre y listo)")
    ap.add_argument("--reducida", nargs="?", type=float, const=0.0, default=None, metavar="DIST_M",
                    help="cancha REDUCIDA: parte mirando al árbol, sin ubicarse con el mapa; el LiDAR mide el pie "
                         "(DIST_M: frente del robot -> pie, por si no lo ve)")
    ap.add_argument("--cajas", choices=("colores", "cercana", "guardar"), default=None,
                    help="cómo distinguir las cajas (por defecto mision.cajas de la config): colores = con la "
                         "cámara; si las dos son del mismo color: cercana = maduros a la más cercana (apuesta), "
                         "guardar = no entregar (quedan en el robot)")
    a = ap.parse_args()
    cfg = cargar()
    if a.nivel:
        cfg.mision.nivel = a.nivel
    if a.cajas:
        cfg.mision.cajas = a.cajas
    if a.max_granos is not None:
        cfg.mision.max_granos = a.max_granos
    if a.reducida is not None:
        cfg.mision.reducida, cfg.mision.reducida_dist_m = True, a.reducida
        if cfg.mision.nivel >= 2:
            print("cancha reducida: nivel 1 (los granos quedan en el robot; no hay canastas)")
            cfg.mision.nivel = 1
    if cfg.mision.nivel >= 3:
        cfg.mision.max_arboles = max(cfg.mision.max_arboles, 3)
        cfg.mision.recoger = tuple(sorted(set(cfg.mision.recoger) | {cancha.SOBREMADURO}))

    gyro = None
    if a.sim:
        esc, mega, lidar, brazo, vision, depositos = armar_sim(cfg, a.semilla, a.t_agarre, a.zona,
                                                               frente_arbol=None if a.reducida is None else (a.reducida or 0.55))
        print(f"Cancha sorteada: partida {esc.zona_robot}, caja roja {esc.zona_caja_roja}, "
              f"azul {esc.zona_caja_azul}, albercas {esc.albercas}")
        if a.brazo:
            brazo = brazo_hibrido(cfg, brazo)
    else:
        from cafebot.drivers.ld06 import LD06
        from cafebot.drivers.mega import Mega
        mega = Mega(cfg.puertos.mega)
        print("Conectando con el Mega...")
        mega.conectar()
        mega.telemetria(20)
        cc = cfg.camara          # la cámara al frente con lo calibrado (tools.cabezal calibrar), sin recargar el Mega
        for k, ang in ((2, cc.servo_pitch_centro),):    # el giro (X 3) está averiado: sin señal (5 nov)
            r = mega.comando(f"X {k} {ang}")
            if not r.startswith("OK"):
                print(f"Cámara: X {k} {ang} -> {r}")
        if cfg.ir.umbrales is not None or cfg.ir.histeresis is not None:
            mega.ir_aplicar(cfg.ir.umbrales, cfg.ir.histeresis)
        lidar = LD06(cfg.puertos.lidar, cfg.geometria.config_ld06())
        brazo, vision, depositos = armar_reales(cfg, a.brazo, a.vision, mega)
        if a.imu:
            from cafebot.drivers.imu import Giroscopio
            gyro = Giroscopio(cfg.puertos.imu_bus, cfg.puertos.imu_dir)
    boton = {"enter": I.BotonEnter, "ya": I.Boton}.get(a.boton or ("ya" if a.sim else "enter"))
    boton = I.BotonGPIO(cfg.mision.pin_boton) if a.boton == "gpio" else boton()

    lidar.iniciar()
    vision.iniciar()
    depositos.preparar()                       # compuertas conectadas y sujetas antes del botón (fuera del reloj)
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
        print(f"PUNTAJE (simulado): {depositos.puntaje()} | caja roja en el sector {esc.zona_caja_roja}, azul en el "
              f"{esc.zona_caja_azul} | la misión creyó: {m.cajas or '-'} (modo {cfg.mision.cajas})")
    print("Líneas cruzadas:", m.nav.lineas.estadisticas() if hasattr(m, "nav") else "")
    if getattr(mega, "ir_sim", None):
        mega.ir_sim.detener()
    lidar.detener()
    mega.cerrar()
    if hasattr(vision, "cerrar"):
        vision.cerrar()                        # (libcamera tiene hilos propios: que no dejen el programa colgado)
    # El brazo y las compuertas quedan con torque: el brazo en su postura de descanso y las compuertas cerradas
    # mientras los jueces levantan el robot (sin torque, una compuerta podría abrirse y botar granos).
    print("FIN: la misión terminó y el programa se cierra.")


if __name__ == "__main__":
    main()
