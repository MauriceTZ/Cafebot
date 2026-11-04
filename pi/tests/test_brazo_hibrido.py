"""Prueba HÍBRIDA (ronda simulada con el brazo real) contra el bus Dynamixel falso.   python3 -m tests.test_brazo_hibrido

  1. Un grano alcanzable: el brazo "real" hace el agarre completo aunque la garra cierre en el aire (prueba sin
     granos), va a la canasta, y el mundo simulado saca el grano del árbol y lo cuenta.
  2. Uno fuera de alcance: el brazo no se mueve y el grano sigue en el árbol.
  3. Detener (ESC / Ctrl+C, Mision.detener): el brazo no empieza otro movimiento; preparar() lo habilita de nuevo.
"""
import math
import pathlib
import tempfile

import numpy as np

from tests import bus_dxl_falso

falso = bus_dxl_falso.instalar()

from cafebot.arm import brazo as mod_brazo   # noqa: E402
from cafebot.arm.brazo import BrazoXL430   # noqa: E402
from cafebot.config import cargar   # noqa: E402
from cafebot.mision import interfaces as I   # noqa: E402
from cafebot.mision.mision import Mision   # noqa: E402
from cafebot.mundo import cancha   # noqa: E402
from cafebot.navegacion.geometria import relativo   # noqa: E402

ADDR_GOAL = 116


def metas():
    return [(s, v) for s, a, v in falso.escrituras if a == ADDR_GOAL]


def armar():
    cfg = cargar()
    cfg.brazo.t_mov_s = cfg.brazo.t_fino_s = 0.01            # sin esperas
    mod_brazo.RAIZ_PI = pathlib.Path(tempfile.mkdtemp())     # perfil de fábrica, en una carpeta temporal
    cfg.mision.brazo_x, cfg.mision.brazo_y, cfg.mision.brazo_z = 0.10, 0.0, 0.36   # alcanza las 3 filas de frente
    real = BrazoXL430(cfg, "falso", log=lambda t: None)
    real.conectar()
    falso.mem[(9, 132)] = mod_brazo.dxl.BRAZO[0].centro_pasos + 2048          # base mirando a la canasta
    real.grabar_postura("deposito_maduro")
    real.ir("trabajo")
    esc = cancha.Escenario(3, 1, 5, (), cancha.sorteo_granos(np.random.default_rng(3)))
    yt = cancha.centros_arboles()[1]
    pose = np.array([cancha.ARBOL_X_FRENTE - cfg.mision.holgura_agarre_m - cfg.geometria.largo / 2, yt, 0.0])
    sim = I.BrazoSimulado(esc, lambda: tuple(pose), (0.10, 0.0, 0.36), t_agarre=1.0, p_falla=0.0,
                          alcanzable_fn=real.alcanzable)
    log = []
    return cfg, real, sim, I.BrazoHibrido(real, sim, log=log.append), esc, pose, log


def punto(pose, sop):
    r = relativo(pose, np.array([sop.x, sop.y, 0.0]))
    return np.array([r[0], r[1], sop.z])


def main():
    cfg, real, sim, hib, esc, pose, log = armar()
    assert hib.preparar(), real.ultimo_motivo
    assert sim.t == 0.0 and real.prueba_sin_granos

    # 1) un maduro de la fila del medio, de frente (las dos columnas centrales: maduros a propósito)
    medio = [s for s in cancha.soportes(2) if s.fila == "media" and abs(s.y - cancha.centros_arboles()[1]) < 0.07]
    for s in medio:
        esc.granos[s.clave] = (cancha.MADURO, "rojo")
    sop = medio[0]
    falso.grano = False                                       # la garra cierra en el aire
    n = len(metas())
    assert hib.alcanzable(punto(pose, sop))
    assert hib.recoger(punto(pose, sop), cancha.MADURO), (real.ultimo_motivo, log)
    movidos = {s for s, _ in metas()[n:]}
    assert {4, 9, 8}.issubset(movidos), movidos               # garra, base, hombro
    assert sop.clave not in esc.granos and sim.depositos[cancha.MADURO] == [cancha.MADURO]
    print(f"  agarre completo en el aire: {len(metas()) - n} órdenes a los servos {sorted(movidos)}; el grano "
          f"{sop.fila}-{sop.col + 1} salió del árbol simulado | {hib.estado()}")

    # 2) fuera de alcance: nada se mueve (activar el torque escribe como meta la posición actual: no es moverse)
    lejos = punto(pose, sop) + np.array([0.35, 0.0, 0.0])
    antes = real.bus.pasos()
    assert not hib.recoger(lejos, cancha.MADURO) and real.bus.pasos() == antes
    print("  fuera de alcance: el brazo no se mueve")

    # 3) detener: la misión lo para y el brazo no empieza otro movimiento
    otro = medio[1]

    class Nav:
        def detener(self):
            pass

    m = Mision(cfg, Nav(), None, None, hib, I.VisionNula(), None, log=lambda t: None)
    m.detener()
    assert real.parar.is_set()
    antes = {k: v for k, v in real.bus.pasos().items() if k != real.ID_GARRA}
    assert not hib.recoger(punto(pose, otro), cancha.MADURO)
    despues = {k: v for k, v in real.bus.pasos().items() if k != real.ID_GARRA}
    assert despues == antes and otro.clave in esc.granos      # a lo más abre la garra
    assert hib.preparar() and not real.parar.is_set()
    print("  Mision.detener() para el brazo (no empieza otro movimiento); preparar() lo vuelve a habilitar")
    print("OK: prueba híbrida con el brazo")


if __name__ == "__main__":
    main()
