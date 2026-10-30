"""Sensores de línea (4 × QTR-1A) sin robot: firmware real simulado + lecturas analógicas simuladas.

    python3 -m tests.test_ir

  1. Detector de líneas (sin simulador): un cruce sintético se ubica sobre el eje de la línea, se asocia a la
     línea correcta de la cancha y corrige el EKF solo en la dirección perpendicular.
  2. Firmware + driver: umbrales por defecto, lecturas crudas, auto-calibración con un barrido sobre una
     línea, cruces a 150 mm/s (dónde la ubica cada sensor y qué ancho ve), umbrales enviados desde la Pi.
  3. Navegación en la cancha: el robot sale de su sector y cruza el borde del beneficiadero; los 4 sensores
     lo ven, los cruces quedan en el mapa asociados a esa línea y con residuo de pocos mm.
"""
import math
import time

import numpy as np

from cafebot.config import cargar
from cafebot.drivers.ir_simulado import IRSimulado, pose_mega_extrapolada
from cafebot.drivers.mega import EventoIR, Mega
from cafebot.mundo import cancha
from cafebot.navegacion.geometria import invertir
from cafebot.navegacion.lineas import DetectorLineas
from cafebot.navegacion.odometria import OdometriaEKF
from tests.sim_serial import SerialSimulado, compilar


def esperar(cond, max_s, paso=0.02, cada=None):
    fin = time.monotonic() + max_s
    while time.monotonic() < fin:
        if cada:
            cada()
        if cond():
            return True
        time.sleep(paso)
    return False


def prueba_detector(cfg):
    """1) Sin simulador: el robot avanza en +x por y = 1,8; la cancha coincide con el mapa."""
    det = DetectorLineas(cfg.ir)
    det.fijar_cancha(np.zeros(3))
    sx = cfg.ir.posiciones[0][0]
    x_entra, x_sale = cancha.BENEF_X - cancha.LINEA - sx, cancha.BENEF_X - sx      # robot cuando DI entra / sale
    ev1 = EventoIR(1, 1, 0, x_entra, 1.8, 0.0, 1.0)
    ev2 = EventoIR(0, 0, 1, x_sale, 1.8, 0.0, 1.2)
    ahora = np.array([x_sale + 0.05, 1.8, 0.0])        # se procesa un poco después
    det.procesar(ev1, ahora, ahora)
    hechos = det.procesar(ev2, ahora, ahora)
    c, s_medio = hechos[0]
    print(f"  cruce DI: centro x={c.centro[0]:.4f} (eje {cancha.BENEF_X - cancha.LINEA / 2:.4f}), "
          f"largo {1000 * c.largo:.1f} mm, línea '{c.linea}', residuo {1000 * c.residuo:+.1f} mm")
    assert c.linea == "borde beneficiadero" and abs(c.residuo) < 1e-6 and abs(c.largo - 0.019) < 1e-6
    # EKF 2 cm adelantado: la corrección lo trae de vuelta en x y casi no toca y (la línea no informa y)
    ekf = OdometriaEKF()
    ekf.inicializar(ahora + np.array([0.02, 0.0, 0.0]), [0, 0, 0])
    ekf.P = np.diag([0.03 ** 2, 0.03 ** 2, math.radians(1) ** 2])
    n, cc = det.recta_en_mapa(c.linea)
    assert ekf.corregir_linea(s_medio, n, cc, 0.005)
    print(f"  EKF 2 cm adelantado -> tras la línea: error x {1000 * (ekf.x[0] - ahora[0]):+.1f} mm, "
          f"y {1000 * (ekf.x[1] - ahora[1]):+.1f} mm")
    assert abs(ekf.x[0] - ahora[0]) < 0.006 and abs(ekf.x[1] - ahora[1]) < 0.002
    # Una rueda que entra al perímetro (x = 0..0,019) queda como alerta
    det.procesar(EventoIR(4, 4, 0, 0.012 - cfg.ir.posiciones[2][0], 1.8, 0.0, 1.5),
                 np.array([0.012 - cfg.ir.posiciones[2][0], 1.8, 0.0]), np.array([0.012 - cfg.ir.posiciones[2][0], 1.8, 0.0]))
    assert len(det.alertas) == 1 and det.alertas[0][2] == "perimetro x=0", det.alertas
    print(f"  alerta: {cfg.ir.nombres[det.alertas[0][1]]} entró a '{det.alertas[0][2]}'")
    # Un cruce lejos de toda línea conocida no se asocia
    det.procesar(EventoIR(1, 1, 0, 1.2, 1.8, 0.0, 2.0), ahora, ahora)
    c2, _ = det.procesar(EventoIR(0, 0, 1, 1.219, 1.8, 0.0, 2.1), ahora, ahora)[0]
    assert c2.linea is None


def prueba_firmware(cfg):
    """2) Firmware simulado con una línea a 30 cm delante del centro del robot (marco del Mega)."""
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(50)
    linea_x = 0.30
    ir = IRSimulado(mega, cfg.ir.posiciones, [cancha.Rect(linea_x, 0.0, 0.019, 3.0)],
                    lambda: pose_mega_extrapolada(mega), semilla=1).iniciar()
    try:
        est = mega.ir_estado()
        print(f"  de fábrica: umbrales {est.umbrales}, histéresis {est.histeresis}")
        assert est.umbrales == [500] * 4 and est.histeresis == 60
        mega.ir_monitor(50)
        assert esperar(lambda: mega.ir_crudo is not None and min(mega.ir_crudo[0]) > 50, 2.0)
        print(f"  lecturas sobre la lona: {mega.ir_crudo[0]}")
        assert all(100 < v < 200 for v in mega.ir_crudo[0])
        # Auto-calibración: avanzar 50 cm (los 4 sensores pasan sobre la línea) y volver
        from tools.ir import barrido
        print("  " + barrido(mega, dist_mm=500, vel=80))
        est = mega.ir_estado()
        print(f"  calibrado: blanco {est.blanco} negro {est.negro} umbrales {est.umbrales}")
        assert all(abs(u - 510) < 40 for u in est.umbrales)
        # Cruce a 150 mm/s: dónde ubica la línea cada sensor
        while not mega.ir_eventos.empty():
            mega.ir_eventos.get_nowait()
        p0 = np.array(mega.pose()) / np.array([1000.0, 1000.0, 1.0])
        assert mega.avanzar(500, 150).ok
        time.sleep(0.2)
        entra, sale = {}, {}
        while not mega.ir_eventos.empty():
            ev = mega.ir_eventos.get_nowait()
            d = ev.x - p0[0]
            for k in range(4):
                if ev.subida & (1 << k):
                    entra[k] = d
                if ev.bajada & (1 << k):
                    sale[k] = d
        for k in range(4):
            assert k in entra and k in sale, f"el sensor {k} no vio la línea"
            centro = cfg.ir.posiciones[k][0] + (entra[k] + sale[k]) / 2
            ancho = sale[k] - entra[k]
            print(f"  {['DI', 'DD', 'TI', 'TD'][k]}: línea en {1000 * centro:6.1f} mm (real {1000 * linea_x:.0f}), "
                  f"ancho visto {1000 * ancho:4.1f} mm")
            assert abs(centro - linea_x) < 0.006 and abs(ancho - 0.019) < 0.006
        assert mega.avanzar(-500, 150).ok
        # Umbrales de la Pi (robot_local.json) -> Mega
        est = mega.ir_aplicar((480, 490, 500, 510), 70)
        assert est.umbrales == [480, 490, 500, 510] and est.histeresis == 70
        assert mega.ir_guardar().startswith("OK")
        mega.ir_monitor(0)
    finally:
        ir.detener()
        mega.cerrar()


def prueba_navegacion(cfg):
    """3) En la cancha: partir del sector 3 y cruzar el borde del beneficiadero (x = 0,731..0,75 m)."""
    from cafebot.drivers.lidar_simulado import LD06Simulado
    from cafebot.navegacion.sistema import SistemaNavegacion
    mega = Mega("sim")
    mega.conectar(serial_obj=SerialSimulado())
    mega.telemetria(20)
    esc = cancha.Escenario(zona_robot=3, zona_caja_roja=1, zona_caja_azul=5, albercas=(2,))
    inicio = np.array([cancha.ZONA_X_CENTRO, cancha.centros_zonas_beneficiadero()[2], 0.0])
    lidar = LD06Simulado(cfg.geometria.config_ld06(), esc.segmentos(), inicio,
                         lambda: (mega.tele.x / 1000, mega.tele.y / 1000, mega.tele.th), semilla=2)
    ir = IRSimulado(mega, cfg.ir.posiciones, cancha.lineas_cancha(),
                    lambda: lidar.pose_real_ahora(pose_mega_extrapolada(mega)), semilla=2).iniciar()
    lidar.iniciar()
    nav = SistemaNavegacion(cfg, lidar, mega)
    try:
        assert esperar(lambda: nav.ekf.inicializada, 5, cada=nav.actualizar)
        nav.fijar_cancha(invertir(inicio))          # el mapa empieza en la pose de partida
        esperar(lambda: False, 1.0, cada=nav.actualizar)
        i = mega.avanzar(650, 150, esperar=False)
        esperar(lambda: i in mega._resultados, 15, cada=nav.actualizar)
        esperar(lambda: False, 0.5, cada=nav.actualizar)
        cr = list(nav.lineas.cruces)
        for c in cr:
            print(f"  {cfg.ir.nombres[c.sensor]}: '{c.linea}' residuo "
                  + (f"{1000 * c.residuo:+.1f} mm" if c.residuo is not None else "—") + f", largo {1000 * c.largo:.1f} mm")
        e = nav.lineas.estadisticas()
        assert len(cr) == 4 and e["asociados"] == 4, e
        assert all(c.linea == "borde beneficiadero" for c in cr)
        assert abs(e["residuo_medio"]) < 0.02
        # Volver cruzando otra vez, ahora con la corrección del EKF activada
        cfg.ir.corregir_pose = True
        i = mega.avanzar(-650, 150, esperar=False)
        esperar(lambda: i in mega._resultados, 15, cada=nav.actualizar)
        esperar(lambda: False, 0.5, cada=nav.actualizar)
        e = nav.lineas.estadisticas()
        print(f"  ida y vuelta: {e['cruces']} cruces, {e['asociados']} asociados, residuo "
              f"{1000 * e['residuo_medio']:+.1f} ± {1000 * e['residuo_std']:.1f} mm, corrigieron el EKF: {e['corregidos']}")
        assert e["cruces"] == 8 and e["asociados"] == 8 and e["corregidos"] >= 1
    finally:
        cfg.ir.corregir_pose = False
        ir.detener()
        lidar.detener()
        mega.cerrar()


def prueba_bordes(cfg):
    """4) tools.ir borde: el robot parte torcido, retrocede hasta el borde de atrás, se endereza con él SIN pasar
    la huincha (toque y giro) y va en cangrejo hasta el borde derecho; mide blanco/negro de cada sensor."""
    import argparse
    import threading
    from cafebot.drivers.ir_simulado import pose_mega_extrapolada
    from tools.ir import BORDE_SIM, buscar_bordes, conectar
    for torcido in (5.0, 14.0):
        a = argparse.Namespace(sim=True, cmd="borde", torcido=torcido, puerto=None)
        mega, ir = conectar(a, cfg)
        # Cuánto entra cada sensor (= su rueda) más allá del borde INTERIOR de cada huincha (19 mm de ancho)
        dentro = {"max": 0.0, "corriendo": True}

        def vigilar():
            while dentro["corriendo"]:
                x, y, th = pose_mega_extrapolada(mega)
                c, s = math.cos(th), math.sin(th)
                for px, py in cfg.ir.posiciones:
                    sx, sy = x + c * px - s * py, y + s * px + c * py
                    dentro["max"] = max(dentro["max"], (-BORDE_SIM[0] + 0.0095) - sx, (-BORDE_SIM[1] + 0.0095) - sy)
                time.sleep(0.003)
        threading.Thread(target=vigilar, daemon=True).start()
        try:
            res = buscar_bordes(mega, cfg, vel=80, max_mm=1000, log=lambda t: None)
            dentro["corriendo"] = False
            assert "error" not in res, res
            assert res["atras_inicial"].primero == 2, res               # TI: el robot está girado a la izquierda
            assert abs(res["torcido_inicial_deg"] + torcido) < 1.0, res   # y lo endereza girando −torcido
            assert res["derecha"] is not None
            x, y, th = mega.pose()
            print(f"  partió torcido {torcido:+.0f}°: giró {res['torcido_inicial_deg']:+.1f}°, rumbo final "
                  f"{math.degrees(th):+.1f}° | lo más que entró un sensor en la huincha: {1000 * dentro['max']:.1f} mm "
                  f"(ancho 19) | negro {res['negro']}, umbrales sugeridos {res['sugeridos']}")
            assert abs(math.degrees(th)) < 1.0
            assert dentro["max"] < 0.015, dentro                         # nunca sale de la cancha
            # quedó despegado ~4 cm de cada borde: centro a 35 - 11 - 4 cm del de atrás y 40 - 11 - 4 del derecho
            assert abs(x / 1000 + BORDE_SIM[0] - 0.1115 - 0.04) < 0.02 and abs(y / 1000 + BORDE_SIM[1] - 0.1085 - 0.04) < 0.02
            assert all(abs(res["sugeridos"][k] - 510) < 40 for k in (2, 3)) and res["negro"][0] is None
        finally:
            dentro["corriendo"] = False
            ir.detener()
            mega.cerrar()


def main():
    compilar()
    cfg = cargar()
    print("1) Detector de líneas")
    prueba_detector(cfg)
    print("2) Firmware + driver")
    prueba_firmware(cfg)
    print("3) Navegación en la cancha")
    prueba_navegacion(cfg)
    print("4) Buscar los bordes de atrás y derecho")
    prueba_bordes(cfg)
    print("OK: sensores de línea")


if __name__ == "__main__":
    main()
