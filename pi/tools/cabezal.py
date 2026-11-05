"""Probar el cabezal de la cámara: los dos servos hacen un CÍRCULO dentro de su rango y se ve lo que mira la cámara.

    python3 -m tools.cabezal                       # 2 vueltas, ventana con la cámara en vivo (en la pantalla HDMI
                                                   # si se lanza por SSH); al final un mosaico en pi/capturas/
    python3 -m tools.cabezal --vueltas 1 --periodo 10 --radio 0.5
    python3 -m tools.cabezal --sin-camara          # solo los servos
    python3 -m tools.cabezal --sim                 # sin robot: el Mega simulado (prueba el recorrido)

Rangos (5 nov, config.camara): inclinación X 2 60..100° y giro X 3 50..150°; el centro (80 / 100) = al frente. El
círculo es una elipse alrededor del centro con `--radio` × medio rango (0,9 por defecto: no toca los topes). Al
terminar (o con Ctrl+C / ESC / Q) la cámara vuelve al frente.
Si el firmware tiene el giro desactivado (AUX_ACTIVO en firmware/mega_cafebot/config.h) el Mega contesta ERR a X 3:
se avisa y solo se mueve la inclinación (sube y baja). Mientras corre, el Mega no puede estar abierto en otro programa.
"""
from __future__ import annotations

import argparse
import math
import os
import time

from cafebot.config import cargar


def puntos_circulo(cc, radio: float, pasos: int) -> list[tuple[int, int]]:
    """[(X 2, X 3)] de una vuelta de la elipse alrededor del frente, dentro de los rangos (sin repetir ángulos)."""
    r_p = radio * (cc.servo_pitch_max - cc.servo_pitch_min) / 2
    r_y = radio * (cc.servo_yaw_max - cc.servo_yaw_min) / 2
    out = []
    for i in range(pasos):
        a = 2 * math.pi * i / pasos
        p = round(cc.servo_pitch_centro + r_p * math.sin(a))
        y = round(cc.servo_yaw_centro + r_y * math.cos(a))
        p = min(cc.servo_pitch_max, max(cc.servo_pitch_min, p))
        y = min(cc.servo_yaw_max, max(cc.servo_yaw_min, y))
        if not out or out[-1] != (p, y):
            out.append((p, y))
    return out


def mosaico(fotos, columnas: int = 6, ancho: int = 320):
    """Las fotos (con su rótulo ya dibujado) en una grilla, para ver de un vistazo qué mira en cada punto."""
    import cv2
    import numpy as np
    if not fotos:
        return None
    alto = int(round(ancho * fotos[0].shape[0] / fotos[0].shape[1]))
    chicas = [cv2.resize(f, (ancho, alto)) for f in fotos]
    while len(chicas) % columnas:
        chicas.append(np.zeros_like(chicas[0]))
    filas = [np.hstack(chicas[i:i + columnas]) for i in range(0, len(chicas), columnas)]
    return np.vstack(filas)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None, help="puerto del Mega (por defecto el de la config)")
    ap.add_argument("--sim", action="store_true", help="Mega simulado (sin robot)")
    ap.add_argument("--vueltas", type=int, default=2)
    ap.add_argument("--periodo", type=float, default=8.0, help="segundos por vuelta")
    ap.add_argument("--pasos", type=int, default=24, help="puntos por vuelta")
    ap.add_argument("--radio", type=float, default=0.9, help="fracción del medio rango (0..1)")
    ap.add_argument("--sin-camara", action="store_true")
    ap.add_argument("--sin-ventana", action="store_true", help="no abrir la ventana en vivo (igual guarda el mosaico)")
    from tools.pantalla import agregar_opcion
    agregar_opcion(ap)
    a = ap.parse_args()
    cfg = cargar()
    cc = cfg.camara
    radio = min(1.0, max(0.05, a.radio))
    circulo = puntos_circulo(cc, radio, max(4, a.pasos))
    print(f"Inclinación X 2 {cc.servo_pitch_min}..{cc.servo_pitch_max} (frente {cc.servo_pitch_centro}), giro X 3 "
          f"{cc.servo_yaw_min}..{cc.servo_yaw_max} (frente {cc.servo_yaw_centro}); círculo al {100 * radio:.0f} %: "
          f"{len(circulo)} puntos por vuelta, {a.periodo:.0f} s por vuelta")

    from cafebot.drivers.mega import Mega
    if a.sim:
        from tests.sim_serial import SerialSimulado
        mega = Mega("sim")
        mega.conectar(serial_obj=SerialSimulado())
    else:
        from cafebot.drivers.puertos import resolver
        mega = Mega(resolver(a.mega or cfg.puertos.mega, "mega"))
        mega.conectar()
    cam = None
    if not a.sin_camara and not a.sim:
        try:
            from cafebot.vision.camara import abrir_para_mision
            cam = abrir_para_mision()
        except Exception as e:
            print(f"(sin cámara: {e}) — sigo solo con los servos")
    pantalla = None
    if cam is not None and not a.sin_ventana:
        try:
            from tools.pantalla import Pantalla
            pantalla = Pantalla("Cabezal de la cámara", (960, 540), a.pantalla)
        except Exception as e:
            print(f"(sin ventana: {e}) — igual guardo el mosaico")

    giro_ok = True

    def mandar(p, y):
        nonlocal giro_ok
        r = mega.comando(f"X 2 {p}")
        if not r.startswith("OK"):
            print(f"  X 2 {p}: {r}")
        if giro_ok:
            r = mega.comando(f"X 3 {y}")
            if not r.startswith("OK"):
                giro_ok = False
                print(f"  X 3 {y}: {r}\n  El giro está DESACTIVADO en el firmware: solo se mueve la inclinación. Para "
                      "activarlo: AUX_ACTIVO = { true, true, true, true } en firmware/mega_cafebot/config.h y recargar")

    fotos, video = [], None
    salir = False
    try:
        mandar(cc.servo_pitch_centro, cc.servo_yaw_centro)
        time.sleep(0.8)
        espera = a.periodo / len(circulo)
        for v in range(a.vueltas):
            for p, y in circulo:
                t0 = time.monotonic()
                mandar(p, y)
                txt = f"X 2 = {p}   X 3 = {y if giro_ok else '-'}"
                print(f"  vuelta {v + 1}: {txt}", end="\r", flush=True)
                if cam is not None:
                    import cv2
                    time.sleep(min(0.25, espera * 0.6))          # que el servo llegue antes de la foto
                    img = cam.capturar()
                    marca = img.copy()
                    cv2.putText(marca, txt, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 0), 6)
                    cv2.putText(marca, txt, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 255, 255), 2)
                    h, w = marca.shape[:2]
                    cv2.line(marca, (w // 2 - 30, h // 2), (w // 2 + 30, h // 2), (0, 255, 255), 2)   # centro
                    cv2.line(marca, (w // 2, h // 2 - 30), (w // 2, h // 2 + 30), (0, 255, 255), 2)
                    if v == 0:
                        fotos.append(marca)
                    if video is None:
                        os.makedirs("capturas", exist_ok=True)
                        ruta_video = time.strftime("capturas/cabezal_%Y%m%d_%H%M%S.mp4")
                        video = cv2.VideoWriter(ruta_video, cv2.VideoWriter_fourcc(*"mp4v"),
                                                max(1.0, 1.0 / max(espera, 0.05)), (w // 2, h // 2))
                    video.write(cv2.resize(marca, (w // 2, h // 2)))
                    if pantalla is not None:
                        salir = mostrar(pantalla, marca)
                        if salir:
                            break
                time.sleep(max(0.0, espera - (time.monotonic() - t0)))
            if salir:
                break
        print()
    except KeyboardInterrupt:
        print("\n(interrumpido)")
    finally:
        try:
            mandar(cc.servo_pitch_centro, cc.servo_yaw_centro)
            time.sleep(0.5)
            print(f"Cámara de vuelta al frente (X 2 {cc.servo_pitch_centro}"
                  + (f", X 3 {cc.servo_yaw_centro})" if giro_ok else ")"))
        finally:
            mega.cerrar()
            if cam is not None:
                cam.cerrar()
            if video is not None:
                video.release()
                print(f"Video: pi/{ruta_video}")
            if fotos:
                import cv2
                m = mosaico(fotos)
                ruta = time.strftime("capturas/cabezal_%Y%m%d_%H%M%S.jpg")
                cv2.imwrite(ruta, m)
                print(f"Mosaico (una foto por punto de la primera vuelta): pi/{ruta}")


def mostrar(pantalla, img_bgr) -> bool:
    """Dibuja la foto en la ventana (ajustada al tamaño). True si apretaron ESC / Q o cerraron la ventana."""
    import cv2
    import pygame as pg
    for ev in pg.event.get():
        if ev.type == pg.QUIT or (ev.type == pg.KEYDOWN and ev.key in (pg.K_ESCAPE, pg.K_q)):
            return True
        pantalla.tecla(ev)
    sup = pantalla.sup
    W, H = sup.get_size()
    h, w = img_bgr.shape[:2]
    e = min(W / w, H / h)
    rgb = cv2.cvtColor(cv2.resize(img_bgr, (int(w * e), int(h * e))), cv2.COLOR_BGR2RGB)
    s = pg.image.frombuffer(rgb.tobytes(), (rgb.shape[1], rgb.shape[0]), "RGB")
    sup.fill((0, 0, 0))
    sup.blit(s, ((W - s.get_width()) // 2, (H - s.get_height()) // 2))
    pg.display.flip()
    return False


if __name__ == "__main__":
    main()
