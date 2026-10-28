"""Probar y calibrar la cámara REAL (Pi Camera Module 3) para clasificar los granos del árbol.

Orden recomendado (con un árbol de prueba, el robot mirándolo de frente y la CÁMARA justo frente al centro
del árbol, como lo deja la misión; si no, decir --lateral):
    python3 -m tools.camara exposicion --dist 0.5            # 1. fija exposición y balance de blancos (se guarda)
    python3 -m tools.camara foto --frente 0.50               # 2. foto; anota a qué distancia estaba el frente del robot
    python3 -m tools.camara zonas --imagen capturas/F.jpg --ajustar
                                                             # 3. calza los círculos sobre los granos (ventana):
                                                             #    I/K inclinación  W/S altura  +/- fx  J/L lateral
                                                             #    G guarda en robot_local.json, Q sale
    python3 -m tools.camara referencias --imagen capturas/F.jpg --clase ROJO --pos alta-2 media-4
                                                             # 4. enseña colores (repetir con todas las clases:
                                                             #    ROJO NARANJO AMARILLO VERDE AZUL NEGRO VACIO)
    python3 -m tools.camara revisar                          # 5. ¿se distinguen las clases? (pares peligrosos)
    python3 -m tools.camara clasificar --frente 0.50         # 6. foto nueva + los 16 resultados (y la imagen marcada)
    python3 -m tools.camara caja --dist 0.6                  # color de una caja puesta delante (cámara hacia abajo)

Agregar --mega /dev/ttyUSB1 para que la herramienta incline la cámara (X 2) como en la misión.
Las fotos quedan en pi/capturas/ con un .json al lado (frente, lateral) para no repetir esos datos.
Nombres de soportes: alta-1..5, media-1..6, baja-1..5, numerados de IZQUIERDA a derecha en la imagen.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

from cafebot.config import cargar, guardar_local
from cafebot.mundo import cancha
from cafebot.vision import clasificador as cl
from cafebot.vision.proyeccion import ModeloCamara, tablero_frente


def _inclinar(a, cfg, que: str):
    if not a.mega:
        ang = cfg.mision.camara_arbol_deg if que == "arbol" else cfg.mision.camara_cajas_deg
        print(f"(Recuerda inclinar la cámara como en la misión: X 2 {ang:.0f} en la consola del Mega)")
        return
    from cafebot.drivers.mega import Mega
    m = Mega(a.mega)
    m.conectar()
    ang = cfg.mision.camara_arbol_deg if que == "arbol" else cfg.mision.camara_cajas_deg
    print(m.comando(f"X 2 {ang:.0f}"))
    time.sleep(0.6)
    m.cerrar()


def _camara(dist):
    from cafebot.vision.camara import Camara, cargar_ajustes
    cam = Camara(dist)
    aj = cargar_ajustes()
    if aj:
        cam.aplicar(aj)
    else:
        print("[AVISO] sin ajustes de exposición: se fijan ahora (mejor: tools.camara exposicion)")
        cam.bloquear_auto()
    return cam


def _lado(ruta):
    return os.path.splitext(ruta)[0] + ".json"


def _datos_foto(ruta, a, cfg):
    """frente/lateral: de la línea de comandos, o del .json guardado junto a la foto. Si no se dice el lateral,
    se supone la CÁMARA centrada frente al árbol (como en la misión)."""
    d = {}
    if ruta and os.path.exists(_lado(ruta)):
        d = json.load(open(_lado(ruta)))
    frente = a.frente if a.frente is not None else d.get("frente", None)
    lateral = a.lateral if a.lateral is not None else d.get("lateral", cfg.camara.y)
    if frente is None:
        raise SystemExit("¿A qué distancia del tablero estaba el FRENTE del robot? Usa --frente (m).")
    return frente, lateral


def _posiciones(cfg, frente, lateral, arbol=2):
    return ModeloCamara(cfg.camara).posiciones_soportes(arbol, tablero_frente(frente, cfg.geometria.largo, lateral))


# ---------------------------------------------------------------------------- comandos
def cmd_exposicion(a):
    from cafebot.vision.camara import Camara, guardar_ajustes
    cam = Camara(a.dist)
    try:
        aj = cam.bloquear_auto(3.0)
        guardar_ajustes(aj, a.dist)
        print(f"Ajustes fijados y guardados: {aj}")
        print("Repetir si cambia la iluminación (en la competencia: en la cancha, antes de la primera ronda).")
    finally:
        cam.cerrar()


def cmd_foto(a):
    import cv2
    cfg = cargar()
    _inclinar(a, cfg, "arbol")
    dist = cfg.geometria.largo / 2 + (a.frente or cfg.mision.distancia_foto_m) - cfg.camara.x
    cam = _camara(dist)
    os.makedirs(cl.CARPETA_CAPTURAS, exist_ok=True)
    try:
        for i in range(a.n):
            ruta = os.path.join(cl.CARPETA_CAPTURAS, time.strftime(f"%Y%m%d_%H%M%S_{a.nombre}_{i + 1}.jpg"))
            img = cam.capturar()
            cv2.imwrite(ruta, img)
            if a.frente is not None:
                json.dump({"frente": a.frente, "lateral": cfg.camara.y if a.lateral is None else a.lateral},
                          open(_lado(ruta), "w"))
            print("Guardada:", ruta)
            time.sleep(0.3)
    finally:
        cam.cerrar()


def cmd_zonas(a):
    import cv2
    cfg = cargar()
    img = cv2.imread(a.imagen)
    if img is None:
        raise SystemExit(f"No se pudo leer {a.imagen}")
    frente, lateral = _datos_foto(a.imagen, a, cfg)
    cc = cfg.camara
    if img.shape[1] != cc.ancho_px or img.shape[0] != cc.alto_px:
        print(f"[AVISO] la foto mide {img.shape[1]}×{img.shape[0]} y la config dice {cc.ancho_px}×{cc.alto_px}")
    if not a.ajustar:
        pos = _posiciones(cfg, frente, lateral)
        ruta = cl.guardar_captura(cl.dibujar(img, pos), os.path.basename(os.path.splitext(a.imagen)[0]) + "_zonas.jpg")
        print(f"{len(pos)}/16 soportes dentro de la imagen. Guardada: {ruta}")
        return
    ventana = "zonas: I/K incl  W/S altura  +/- fx  J/L lateral  G guardar  Q salir"
    cv2.namedWindow(ventana, cv2.WINDOW_NORMAL)
    while True:
        pos = _posiciones(cfg, frente, lateral)
        vista = cl.dibujar(img, pos)
        txt = (f"incl {cc.inclinacion_arbol_deg:.1f}  z {cc.z:.3f} m  fx {cc.fx_px:.0f}  "
               f"frente {frente:.3f}  lateral {lateral:+.3f}  ({len(pos)}/16)")
        cv2.putText(vista, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4)
        cv2.putText(vista, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        cv2.imshow(ventana, vista)
        k = cv2.waitKey(0) & 0xFF
        if k in (ord("q"), 27):
            break
        cambios = {ord("i"): ("inclinacion_arbol_deg", 0.5), ord("k"): ("inclinacion_arbol_deg", -0.5),
                   ord("w"): ("z", 0.005), ord("s"): ("z", -0.005),
                   ord("+"): ("fx_px", 10), ord("="): ("fx_px", 10), ord("-"): ("fx_px", -10)}
        if k in cambios:
            campo, d = cambios[k]
            setattr(cc, campo, getattr(cc, campo) + d)
        elif k == ord("j"):
            lateral += 0.005
        elif k == ord("l"):
            lateral -= 0.005
        elif k == ord("g"):
            ruta = guardar_local("camara", {"inclinacion_arbol_deg": round(cc.inclinacion_arbol_deg, 2),
                                            "z": round(cc.z, 4), "fx_px": round(cc.fx_px, 1)})
            print(f"Guardado en {ruta}")
    cv2.destroyAllWindows()


def cmd_referencias(a):
    import cv2
    cfg = cargar()
    clasif = cl.Clasificador.cargar(cl.ARCHIVO_REFERENCIAS)
    for ruta in a.imagen:
        img = cv2.imread(ruta)
        if img is None:
            raise SystemExit(f"No se pudo leer {ruta}")
        frente, lateral = _datos_foto(ruta, a, cfg)
        pos = _posiciones(cfg, frente, lateral)
        nombres = list(pos) if a.pos == ["todas"] else a.pos
        lab = cl.imagen_a_lab(img)
        for nombre in nombres:
            if nombre not in pos:
                print(f"  {nombre}: fuera de la imagen, se salta")
                continue
            x, y, r = pos[nombre]
            valor = cl.color_zona(lab, x, y, r, clasif.config)
            clasif.agregar(a.clase, valor, f"{os.path.basename(ruta)}:{nombre}")
            print(f"  {a.clase:9s} <- {nombre}  Lab=({valor[0]:.1f}, {valor[1]:.1f}, {valor[2]:.1f})")
    clasif.guardar(cl.ARCHIVO_REFERENCIAS)
    print(f"Total de muestras: {len(clasif.muestras)} en {cl.ARCHIVO_REFERENCIAS}")


def cmd_revisar(a):
    cl.cmd_revisar(argparse.Namespace(referencias=cl.ARCHIVO_REFERENCIAS))


def cmd_clasificar(a):
    import cv2
    from cafebot.vision.camara import CamaraArchivo
    from cafebot.vision.vision import VisionCamara
    cfg = cargar()
    if a.imagen:
        cam = CamaraArchivo(a.imagen)
        frente, lateral = _datos_foto(a.imagen[0], a, cfg)
    else:
        _inclinar(a, cfg, "arbol")
        frente = a.frente if a.frente is not None else cfg.mision.distancia_foto_m
        lateral = cfg.camara.y if a.lateral is None else a.lateral
        cam = _camara(cfg.geometria.largo / 2 + frente - cfg.camara.x)
    v = VisionCamara(cfg, camara=cam)
    try:
        lect = v.clasificar_arbol(2, None, tablero=tablero_frente(frente, cfg.geometria.largo, lateral),
                                  imagenes=[cam.capturar() for _ in range(len(a.imagen) if a.imagen else cfg.camara.fotos)])
    finally:
        cam.cerrar()
    for fila in cancha.FILAS[::-1]:
        print(f"  {fila:5s} " + "  ".join(f"{l.categoria[:6]:6s}{l.confianza:4.2f}" for l in lect if l.clave[1] == fila))
    res = {}
    for l in lect:
        res[l.categoria] = res.get(l.categoria, 0) + 1
    print("Resumen:", res, "| imagen marcada en", cl.CARPETA_CAPTURAS)


def cmd_caja(a):
    import numpy as np
    from cafebot.vision.camara import CamaraArchivo
    from cafebot.vision.vision import VisionCamara
    cfg = cargar()
    if not a.imagen:
        _inclinar(a, cfg, "cajas")
    cam = CamaraArchivo(a.imagen) if a.imagen else _camara(a.dist + 0.2)
    v = VisionCamara(cfg, camara=cam)
    # Pose ficticia: robot mirando a la caja del sector 3 (θ = 180°), con su frente a `dist` de la cara de la caja
    caja = cancha.rect_caja(3)
    pose = np.array([caja.cx + caja.ancho_x / 2 + a.dist + cfg.geometria.largo / 2, caja.cy + a.lateral_caja, np.pi])
    try:
        print("Color:", v.color_caja(3, pose) or "no sé")
    finally:
        cam.cerrar()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None, help="puerto del Mega para inclinar la cámara (X 2)")
    sub = ap.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("exposicion")
    s.add_argument("--dist", type=float, default=0.5, help="distancia de enfoque (m)")
    s.set_defaults(f=cmd_exposicion)
    s = sub.add_parser("foto")
    s.add_argument("--n", type=int, default=1)
    s.add_argument("--nombre", default="arbol")
    s.add_argument("--frente", type=float, default=None, help="m del frente del robot al tablero")
    s.add_argument("--lateral", type=float, default=None,
                   help="m que el centro del árbol está a la IZQUIERDA del centro del robot (por defecto: cámara centrada)")
    s.set_defaults(f=cmd_foto)
    s = sub.add_parser("zonas")
    s.add_argument("--imagen", required=True)
    s.add_argument("--frente", type=float, default=None)
    s.add_argument("--lateral", type=float, default=None)
    s.add_argument("--ajustar", action="store_true", help="ventana interactiva (escritorio/VNC)")
    s.set_defaults(f=cmd_zonas)
    s = sub.add_parser("referencias")
    s.add_argument("--imagen", nargs="+", required=True)
    s.add_argument("--clase", required=True, choices=cl.CLASES)
    s.add_argument("--pos", nargs="+", required=True, help="alta-2 media-4 ... o 'todas'")
    s.add_argument("--frente", type=float, default=None)
    s.add_argument("--lateral", type=float, default=None)
    s.set_defaults(f=cmd_referencias)
    sub.add_parser("revisar").set_defaults(f=cmd_revisar)
    s = sub.add_parser("clasificar")
    s.add_argument("--imagen", nargs="*", default=None, help="fotos ya tomadas (sin esto, saca fotos nuevas)")
    s.add_argument("--frente", type=float, default=None)
    s.add_argument("--lateral", type=float, default=None)
    s.set_defaults(f=cmd_clasificar)
    s = sub.add_parser("caja")
    s.add_argument("--dist", type=float, default=0.6, help="m del frente del robot a la caja")
    s.add_argument("--lateral-caja", type=float, default=0.0)
    s.add_argument("--imagen", nargs="*", default=None)
    s.set_defaults(f=cmd_caja)
    a = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")
    a.f(a)


if __name__ == "__main__":
    main()
