"""Brazo real (cafebot/arm/brazo.py) contra un bus Dynamixel falso: coordenadas del robot, posturas,
secuencia de agarre con y sin grano. Sin hardware.   python3 -m tests.test_brazo_falso
"""
import json
import math
import tempfile

import numpy as np

from tests import bus_dxl_falso

bus = bus_dxl_falso.instalar()

from cafebot.arm import brazo as mod_brazo  # noqa: E402
from cafebot.arm.brazo import BrazoXL430  # noqa: E402
from cafebot.config import cargar  # noqa: E402
from cafebot.mundo import cancha  # noqa: E402


def main():
    cfg = cargar()
    cfg.brazo.t_mov_s = cfg.brazo.t_fino_s = 0.01          # sin esperas en la prueba
    tmp = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
    tmp.write("{}")
    tmp.close()
    mod_brazo.RAIZ_PI = mod_brazo.pathlib.Path(tmp.name).parent
    cfg.brazo.archivo_posturas = mod_brazo.pathlib.Path(tmp.name).name
    b = BrazoXL430(cfg, "falso", log=lambda t: None)
    b.conectar()

    # 1) Las posturas del código antiguo se leen bien: al encender (bus en "trabajo") el ángulo coincide
    ang = b.bus.angulos()
    for s, v in b.posturas["trabajo"].items():
        assert abs(math.degrees(ang[int(s)]) - v) < 0.1, (s, v)
    p, pitch = b.directa_robot(ang)
    print(f"Postura 'trabajo': garra en x={p[0]:.3f} y={p[1]:+.3f} z={p[2]:.3f}, inclinación {pitch:.0f}°")

    # 2) IK en el sistema del robot -> directa devuelve el mismo punto
    for q in ([0.20, 0.0, 0.45], [0.18, 0.06, 0.41], [0.17, -0.05, 0.56]):
        s = b.ik(q)
        assert s is not None, q
        r, _ = b.directa_robot(s[0])
        assert np.linalg.norm(r - q) < 1e-6, (q, r)

    # 3) Alcance frente al árbol con la holgura de la misión (frente del robot a 8 cm del tablero)
    x_cara = cfg.geometria.largo / 2 + cfg.mision.holgura_agarre_m
    for fila in cancha.FILAS:
        for dy in (0.0, 0.06):
            q = [x_cara - cancha.GRANO_DELANTE_M, dy, cancha.ALTURA_SOPORTE[fila] + cancha.CENTRO_SOBRE_SOPORTE]
            plan = b.plan_agarre(q)
            print(f"  fila {fila:5s} lateral {100*dy:3.0f} cm: "
                  + (f"alcanzable, inclinación {b.directa_robot(plan[2][1])[1]:.0f}°" if plan else "FUERA DE ALCANCE"))

    # 4) Grabar posturas de depósito y agarrar
    bus.mem[(9, 132)] = 3088 + 2048                     # base girada 180° (como si mirara la tolva)
    b.grabar_postura("deposito_maduro")
    assert "deposito_maduro" in json.load(open(tmp.name))
    b.bus.torque(True)
    objetivo = [x_cara - cancha.GRANO_DELANTE_M, 0.0, cancha.ALTURA_SOPORTE["media"] + cancha.CENTRO_SOBRE_SOPORTE]
    bus.grano = False
    assert not b.recoger(objetivo, cancha.MADURO), "sin grano no debe dar recogido"
    assert "no había grano" in b.ultimo_motivo, b.ultimo_motivo
    bus.grano = True
    assert b.recoger(objetivo, cancha.MADURO), b.ultimo_motivo
    assert b.depositos[cancha.MADURO] == 1
    # quedó en la postura de espera (descanso)
    ang = b.bus.angulos()
    assert all(abs(math.degrees(ang[int(s)]) - v) < 0.1 for s, v in b.posturas[cfg.brazo.postura_espera].items())
    # 5) Un choque (la articulación no llega) aborta sin contar el grano
    real_mover = b.bus.mover
    b.bus.mover = lambda obj, t, acel_frac=0.3: real_mover({k: (v + 0.3 if k == 8 else v) for k, v in obj.items()}, t)
    assert not b.recoger(objetivo, cancha.MADURO)
    print("  choque simulado:", b.ultimo_motivo)
    b.bus.mover = real_mover
    print("Estado:", b.estado())
    print("OK: brazo contra bus falso")


if __name__ == "__main__":
    main()
