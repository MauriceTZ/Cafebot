"""Cámara (Pi Camera Module 3) con foco, exposición y balance de blancos FIJOS.

Con todo en automático, el mismo grano cambia de color entre fotos (la cámara
se ajusta según el fondo). Para clasificar por color hay que congelar todo:
  1. Foco manual: LensPosition = 1 / distancia_en_metros.
  2. Se deja que el automático se estabilice unos segundos, se leen la
     exposición, la ganancia y el balance de blancos, y se fijan.
  3. Esos valores se guardan en calibracion/camara.json y se reutilizan.

Uso en la Pi:
  python3 camara.py --calibrar --dist 0.25        # fija y guarda los ajustes
  python3 camara.py --foto pelotas --n 3          # 3 fotos con los ajustes guardados
  python3 camara.py --foto pelotas --auto         # foto en automático (comparar)
"""
from __future__ import annotations

import argparse
import json
import os
import time

# Silenciar los mensajes INFO/WARN de libcamera (solo mostrar errores).
os.environ.setdefault("LIBCAMERA_LOG_LEVELS", "*:ERROR")

ARCHIVO_AJUSTES = os.path.join("calibracion", "camara.json")
CARPETA_CAPTURAS = "capturas"
RESOLUCION = (1536, 864)


class Camara:
    def __init__(self, dist_m: float = 0.25):
        from picamera2 import Picamera2  # solo existe en la Pi
        from libcamera import controls
        self._controls = controls
        self.cam = Picamera2()
        config = self.cam.create_still_configuration(
            main={"size": RESOLUCION, "format": "RGB888"})  # OpenCV lo recibe como BGR
        self.cam.configure(config)
        self.cam.start()
        self.enfocar(dist_m)

    def enfocar(self, dist_m: float):
        self.cam.set_controls({"AfMode": self._controls.AfModeEnum.Manual,
                               "LensPosition": 1.0 / max(dist_m, 0.05)})

    def bloquear_auto(self, espera_s: float = 2.0) -> dict:
        """Deja estabilizar el automático y congela sus valores."""
        self.cam.set_controls({"AeEnable": True, "AwbEnable": True})
        time.sleep(espera_s)
        meta = self.cam.capture_metadata()
        ajustes = {
            "ExposureTime": int(meta["ExposureTime"]),
            "AnalogueGain": float(meta["AnalogueGain"]),
            "ColourGains": [float(g) for g in meta["ColourGains"]],
        }
        self.aplicar(ajustes)
        return ajustes

    def aplicar(self, ajustes: dict):
        self.cam.set_controls({
            "AeEnable": False, "AwbEnable": False,
            "ExposureTime": ajustes["ExposureTime"],
            "AnalogueGain": ajustes["AnalogueGain"],
            "ColourGains": tuple(ajustes["ColourGains"]),
        })
        time.sleep(0.5)  # unos cuadros para que se apliquen

    def capturar(self):
        """Imagen BGR (numpy) lista para OpenCV."""
        return self.cam.capture_array("main")

    def cerrar(self):
        self.cam.stop()
        self.cam.close()


def guardar_ajustes(ajustes: dict, dist_m: float):
    os.makedirs(os.path.dirname(ARCHIVO_AJUSTES), exist_ok=True)
    with open(ARCHIVO_AJUSTES, "w", encoding="utf-8") as f:
        json.dump({**ajustes, "dist_m": dist_m}, f, indent=2)


def cargar_ajustes():
    if not os.path.exists(ARCHIVO_AJUSTES):
        return None
    with open(ARCHIVO_AJUSTES, encoding="utf-8") as f:
        return json.load(f)


def main():
    import cv2
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--calibrar", action="store_true", help="fijar y guardar exposición/balance")
    p.add_argument("--dist", type=float, default=None, help="distancia de enfoque en metros")
    p.add_argument("--foto", default=None, help="nombre base de las fotos a guardar")
    p.add_argument("--n", type=int, default=1, help="cantidad de fotos")
    p.add_argument("--auto", action="store_true", help="no fijar nada (para comparar)")
    args = p.parse_args()

    guardados = cargar_ajustes()
    dist = args.dist or (guardados or {}).get("dist_m", 0.25)
    cam = Camara(dist)
    try:
        if args.calibrar:
            ajustes = cam.bloquear_auto()
            guardar_ajustes(ajustes, dist)
            print(f"Ajustes fijados y guardados en {ARCHIVO_AJUSTES}: {ajustes}")
        elif args.auto:
            time.sleep(2.0)
            print("Modo automático (sin fijar)")
        elif guardados:
            cam.aplicar(guardados)
            print(f"Usando ajustes guardados: {guardados}")
        else:
            print("[AVISO] No hay ajustes guardados: se fijan ahora (usa --calibrar para guardarlos).")
            cam.bloquear_auto()

        if args.foto:
            os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
            marca = time.strftime("%Y%m%d_%H%M%S")
            for i in range(args.n):
                ruta = os.path.join(CARPETA_CAPTURAS, f"{args.foto}_{marca}_{i + 1}.jpg")
                cv2.imwrite(ruta, cam.capturar())
                print("Guardada:", ruta)
                time.sleep(0.3)
    finally:
        cam.cerrar()


if __name__ == "__main__":
    main()
