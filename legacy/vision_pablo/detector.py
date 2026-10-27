"""Detector de pelotas (granos de 42 mm) para el ajuste fino de la pinza.

Encuentra las esferas por color saturado (o muy oscuro, para el negro) +
contornos + filtros de forma, y para cada una entrega:
  - centro y diámetro en píxeles,
  - desvío en mm respecto de un punto objetivo (por defecto, el centro de la
    imagen): es lo que el brazo debe corregir,
  - distancia estimada a la cámara (por el tamaño aparente; requiere calibrar
    la focal una vez con --calibrar-dist),
  - color según clasificador.py (NO_SE si aún no hay referencias).

Uso:
  python3 detector.py --imagen capturas/foto.jpg        # offline (también en el PC)
  python3 detector.py --camara --n 20                   # en vivo en la Pi (20 cuadros)
  python3 detector.py --camara --calibrar-dist 0.30     # 1 pelota a 30 cm exactos
  python3 detector.py --web                             # video en vivo en el navegador del PC
  python3 detector.py --camara --calibrar-color NARANJO --n 15   # enseñar un color
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass

import cv2
import numpy as np

import clasificador as C

DIAMETRO_MM = 42.0
ARCHIVO_CONFIG = os.path.join("calibracion", "detector.json")

CONFIG_DEFECTO = {
    # Máscara: píxeles muy saturados (pelotas de color) o muy oscuros (negro).
    "s_min": 150,          # saturación mínima (0-255). Piel ~100-130, pelotas >160.
    "v_min": 40,           # brillo mínimo para pelotas de color
    # Filtro estricto al calibrar colores (ver calibrar_color): pelota amarilla
    # S=214 V=182 vs caja ocre S=135-168 V=86-119 (medido el 16 oct).
    "s_min_calibracion": 185,
    "v_min_calibracion": 120,
    "v_negro": 45,         # brillo máximo para considerar "negro" (pelota negra)
    "usar_negro": False,   # activar solo con fondo claro: sombras y cables también son negros
    # Con colores calibrados: buscar pelotas por cercanía a sus colores Lab
    # reales (deja fuera café/ocre). Sin calibrar: bandas de tono de arriba.
    "segmentar_por_referencias": True,
    "factor_umbral_mascara": 1.5,  # más permisivo que el clasificador (bordes y sombras)
    "escala_mascara": 0.5,         # la máscara se calcula a media resolución (más rápido)
    # Forma
    "area_min_frac": 0.0004,   # área mínima del contorno respecto de la imagen
    "circularidad_min": 0.70,  # 4·pi·área/perímetro² (1 = círculo perfecto)
    "relleno_min": 0.65,       # área del contorno / área del círculo envolvente
    # Cámara: focal en píxeles (se calibra con --calibrar-dist)
    "focal_px": None,
}


@dataclass
class Deteccion:
    cx: float
    cy: float
    diam_px: float
    circularidad: float
    relleno: float
    clase: str = C.NO_SE
    dist_clase: float = math.inf

    @property
    def mm_por_px(self) -> float:
        return DIAMETRO_MM / self.diam_px

    def desvio_mm(self, objetivo_px) -> tuple:
        """(dx, dy) en mm: positivo = la pelota está a la derecha / abajo del objetivo."""
        k = self.mm_por_px
        return ((self.cx - objetivo_px[0]) * k, (self.cy - objetivo_px[1]) * k)

    def distancia_m(self, focal_px):
        if not focal_px:
            return None
        return focal_px * (DIAMETRO_MM / 1000.0) / self.diam_px


def cargar_config() -> dict:
    cfg = dict(CONFIG_DEFECTO)
    if os.path.exists(ARCHIVO_CONFIG):
        with open(ARCHIVO_CONFIG, encoding="utf-8") as f:
            cfg.update(json.load(f))
    return cfg


def guardar_config(cfg: dict):
    os.makedirs(os.path.dirname(ARCHIVO_CONFIG), exist_ok=True)
    with open(ARCHIVO_CONFIG, "w", encoding="utf-8") as f:
        json.dump(cfg, f, indent=2)


# Bandas de tono (H de OpenCV, 0-179). Se buscan contornos en cada banda por
# separado para que dos pelotas de distinto color que se tocan no se fusionen.
# Se solapan a propósito; los duplicados se eliminan después.
BANDAS_TONO = {
    "rojo": [(0, 10), (165, 180)],
    "naranjo": [(5, 22)],
    "amarillo": [(18, 38)],
    "verde": [(38, 90)],
    "azul": [(90, 135)],
}


def _limpiar(m):
    # Cerrar los huecos de los hoyuelos y los brillos, y quitar ruido chico.
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k, iterations=2)
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, k)


def mascaras(imagen_bgr, cfg: dict) -> dict:
    """Una máscara por banda de tono (+ "negro" si está activado)."""
    hsv = cv2.cvtColor(imagen_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
    color = (s >= cfg["s_min"]) & (v >= cfg["v_min"])
    salida = {}
    for nombre, rangos in BANDAS_TONO.items():
        en_banda = np.zeros_like(color)
        for lo, hi in rangos:
            en_banda |= (h >= lo) & (h < hi)
        salida[nombre] = _limpiar((color & en_banda).astype(np.uint8) * 255)
    if cfg["usar_negro"]:
        salida["negro"] = _limpiar((v <= cfg["v_negro"]).astype(np.uint8) * 255)
    return salida


def mascaras_por_referencias(imagen_bgr, clasif: C.Clasificador, cfg: dict) -> dict:
    """Una máscara por color CALIBRADO: un píxel entra si su color Lab está
    cerca del color real de esa pelota (promedio de sus referencias).
    A diferencia de las bandas de tono, deja fuera colores parecidos pero
    apagados, como el café/ocre del árbol o de una caja de madera.
    Se calcula en una imagen reducida (más rápido); devuelve máscaras a esa escala."""
    escala = cfg["escala_mascara"]
    chica = cv2.resize(imagen_bgr, None, fx=escala, fy=escala, interpolation=cv2.INTER_AREA)
    lab = C.imagen_a_lab(chica)
    peso_L = clasif.config["peso_L"]
    umbral = clasif.config["umbral_distancia"] * cfg["factor_umbral_mascara"]
    por_clase = {}
    for m in clasif.muestras:
        if m["clase"] != "VACIO":
            por_clase.setdefault(m["clase"], []).append(m["lab"])
    salida = {}
    k_cerrar = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    # Apertura grande: borra tiras delgadas pegadas a la pelota (bordes de
    # cajas, soportes) que inflarían el círculo y correrían el centro.
    k_abrir = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11))
    for clase, labs in por_clase.items():
        Lc, ac, bc = np.mean(labs, axis=0)
        d2 = (((lab[:, :, 0] - Lc) * peso_L) ** 2 + (lab[:, :, 1] - ac) ** 2
              + (lab[:, :, 2] - bc) ** 2)
        m = (d2 < umbral * umbral).astype(np.uint8) * 255
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k_cerrar, iterations=2)
        salida[clase] = cv2.morphologyEx(m, cv2.MORPH_OPEN, k_abrir)
    return salida


def _usar_referencias(clasif, cfg) -> bool:
    return bool(clasif and clasif.muestras and cfg.get("segmentar_por_referencias", True))


def mascara(imagen_bgr, cfg: dict, clasif: C.Clasificador = None):
    """Unión de todas las máscaras (para mirarla en una imagen)."""
    if _usar_referencias(clasif, cfg):
        alto, ancho = imagen_bgr.shape[:2]
        mascs = [cv2.resize(m, (ancho, alto), interpolation=cv2.INTER_NEAREST)
                 for m in mascaras_por_referencias(imagen_bgr, clasif, cfg).values()]
    else:
        mascs = list(mascaras(imagen_bgr, cfg).values())
    total = np.zeros(imagen_bgr.shape[:2], np.uint8)
    for m in mascs:
        total = cv2.bitwise_or(total, m)
    return total


def detectar(imagen_bgr, cfg: dict = None, clasif: C.Clasificador = None) -> list:
    """Si hay colores calibrados, busca las pelotas con esas referencias
    (mascaras_por_referencias); si no, con bandas de tono (para calibrar)."""
    cfg = cfg or cargar_config()
    alto, ancho = imagen_bgr.shape[:2]
    lab = C.imagen_a_lab(imagen_bgr) if clasif and clasif.muestras else None
    if _usar_referencias(clasif, cfg):
        escala = cfg["escala_mascara"]
        mascs = mascaras_por_referencias(imagen_bgr, clasif, cfg)
    else:
        escala = 1.0
        mascs = mascaras(imagen_bgr, cfg)
    area_min = cfg["area_min_frac"] * alto * ancho * escala * escala
    candidatas = []
    for m in mascs.values():
        contornos, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contornos:
            if cv2.contourArea(c) < area_min:
                continue
            # La envolvente convexa "rellena" las mordidas de dedos, soportes y
            # brillos: la forma se mide sobre ella.
            casco = cv2.convexHull(c)
            area = cv2.contourArea(casco)
            perim = cv2.arcLength(casco, True)
            circ = 4 * math.pi * area / (perim * perim) if perim else 0
            (x, y), r = cv2.minEnclosingCircle(casco)
            relleno = area / (math.pi * r * r) if r else 0
            if circ < cfg["circularidad_min"] or relleno < cfg["relleno_min"]:
                continue
            candidatas.append(Deteccion(x / escala, y / escala, 2 * r / escala, circ, relleno))
    # Quitar duplicados de bandas solapadas: si dos centros están más cerca que
    # medio radio, queda la de mejor relleno.
    dets = []
    for d in sorted(candidatas, key=lambda d: -d.relleno):
        if all(math.hypot(d.cx - o.cx, d.cy - o.cy) > 0.5 * o.diam_px / 2 for o in dets):
            dets.append(d)
    if lab is not None:
        for d in dets:
            res = clasif.clasificar_lab(C.color_zona(lab, d.cx, d.cy, d.diam_px / 2, clasif.config))
            d.clase, d.dist_clase = res.clase, res.distancia
    # La más grande primero (la más cercana a la pinza).
    return sorted(dets, key=lambda d: -d.diam_px)


def dibujar(imagen_bgr, dets: list, objetivo_px, focal_px=None):
    salida = imagen_bgr.copy()
    ox, oy = int(objetivo_px[0]), int(objetivo_px[1])
    cv2.drawMarker(salida, (ox, oy), (255, 255, 255), cv2.MARKER_CROSS, 30, 2)
    for i, d in enumerate(dets):
        color = C.COLOR_DIBUJO.get(d.clase, (255, 0, 255))
        centro = (int(d.cx), int(d.cy))
        cv2.circle(salida, centro, int(d.diam_px / 2), color, 3)
        cv2.circle(salida, centro, 4, color, -1)
        dx, dy = d.desvio_mm(objetivo_px)
        dist = d.distancia_m(focal_px)
        texto = f"#{i} {d.clase} d={d.diam_px:.0f}px dx={dx:+.0f}mm dy={dy:+.0f}mm"
        if dist:
            texto += f" {dist * 100:.0f}cm"
        pos = (int(d.cx - d.diam_px / 2), max(20, int(d.cy - d.diam_px / 2) - 10))
        cv2.putText(salida, texto, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4)
        cv2.putText(salida, texto, pos, cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    return salida


def informe(dets: list, objetivo_px, focal_px) -> str:
    if not dets:
        return "sin pelotas"
    partes = []
    for i, d in enumerate(dets):
        dx, dy = d.desvio_mm(objetivo_px)
        dist = d.distancia_m(focal_px)
        partes.append(f"#{i} {d.clase} centro=({d.cx:.0f},{d.cy:.0f}) diam={d.diam_px:.0f}px "
                      f"dx={dx:+.1f}mm dy={dy:+.1f}mm"
                      + (f" dist={dist * 100:.1f}cm" if dist else "")
                      + f" circ={d.circularidad:.2f}")
    return "\n  ".join(partes)


def calibrar_color(cam, cfg: dict, clasif, clase: str, n: int, marca: str):
    """Guarda el color de UNA pelota en n cuadros como referencias del clasificador.
    Conviene moverla un poco (posición, giro) para cubrir brillos y sombras."""
    print(f"Calibrando {clase}: deja UNA sola pelota {clase.lower()} a la vista y muévela un poco...")
    # Aquí todavía no se conoce el color, así que se busca por tono. Primero con
    # el filtro normal, que mide la pelota completa. Si no da exactamente una
    # pelota (p. ej. junto a una caja ocre se juntan en una sola mancha), se usa
    # uno ESTRICTO (muy saturado y brillante) que separa la pelota del café/ocre.
    # Como al calibrar hay UNA sola pelota, se ignoran las manchas que miden
    # menos de la mitad que la más grande (trocitos de caja, reflejos).
    estricto = dict(cfg)
    estricto.update(s_min=cfg["s_min_calibracion"], v_min=cfg["v_min_calibracion"])

    def sin_chicas(dets):
        if not dets:
            return dets
        mayor = max(d.diam_px for d in dets)
        return [d for d in dets if d.diam_px >= 0.5 * mayor]

    nuevas = []
    guardada = False
    for i in range(n):
        img = cam.capturar()
        dets = sin_chicas(detectar(img, cfg))
        modo = "normal"
        if len(dets) != 1:
            dets = sin_chicas(detectar(img, estricto))
            modo = "estricto"
        if len(dets) != 1:
            print(f"  cuadro {i + 1}: {len(dets)} pelotas, se descarta")
        else:
            d = dets[0]
            lab = C.color_zona(C.imagen_a_lab(img), d.cx, d.cy, d.diam_px / 2, clasif.config)
            clasif.agregar(clase, lab, f"vivo_{marca}_{i + 1}")
            nuevas.append(lab)
            print(f"  cuadro {i + 1}: Lab=({lab[0]:.1f}, {lab[1]:.1f}, {lab[2]:.1f})  [{modo}]")
            if not guardada:  # guardar una foto de respaldo para revisar después
                guardada = True
                cv2.imwrite(os.path.join(C.CARPETA_CAPTURAS, f"calib_{clase}_{marca}.jpg"), img)
        time.sleep(0.3)
    if not nuevas:
        print("[ERROR] No se guardó ninguna muestra: revisa que se vea exactamente 1 pelota.")
        return
    clasif.guardar()
    prom = np.mean(nuevas, axis=0)
    disp = np.std(nuevas, axis=0)
    print(f"{len(nuevas)} muestras de {clase} guardadas en {C.ARCHIVO_REFERENCIAS}. "
          f"Promedio Lab=({prom[0]:.1f}, {prom[1]:.1f}, {prom[2]:.1f}), "
          f"variación=({disp[0]:.1f}, {disp[1]:.1f}, {disp[2]:.1f})")
    print("Para ver si los colores se distinguen: python3 clasificador.py revisar")


def servir_web(cfg: dict, clasif, puerto: int):
    """Video en vivo con las detecciones dibujadas, para verlo en el navegador.
    Solo para pruebas en el taller (al competir, WiFi apagado)."""
    import socket
    import threading
    from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
    from camara import Camara, cargar_ajustes

    ajustes = cargar_ajustes()
    cam = Camara((ajustes or {}).get("dist_m", 0.25))
    cam.aplicar(ajustes) if ajustes else cam.bloquear_auto()
    ultimo = {"jpg": None}
    cerrojo = threading.Condition()

    def bucle_camara():
        while True:
            img = cam.capturar()
            alto, ancho = img.shape[:2]
            objetivo = (ancho / 2, alto / 2)
            dets = detectar(img, cfg, clasif)
            if dets:
                print(informe(dets, objetivo, cfg["focal_px"]), flush=True)
            dib = dibujar(img, dets, objetivo, cfg["focal_px"])
            ok, jpg = cv2.imencode(".jpg", dib, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                with cerrojo:
                    ultimo["jpg"] = jpg.tobytes()
                    cerrojo.notify_all()

    class Manejador(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path != "/video":
                pagina = (b"<html><body style='margin:0;background:#111'>"
                          b"<img src='/video' style='width:100%'></body></html>")
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.end_headers()
                self.wfile.write(pagina)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=cuadro")
            self.end_headers()
            try:
                while True:
                    with cerrojo:
                        cerrojo.wait(timeout=2)
                        jpg = ultimo["jpg"]
                    if jpg is None:
                        continue
                    self.wfile.write(b"--cuadro\r\nContent-Type: image/jpeg\r\n\r\n" + jpg + b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    threading.Thread(target=bucle_camara, daemon=True).start()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except OSError:
        ip = "<ip-de-la-pi>"
    finally:
        s.close()
    print(f"Abre en el navegador: http://{ip}:{puerto}   (Ctrl+C para terminar)", flush=True)
    try:
        ThreadingHTTPServer(("0.0.0.0", puerto), Manejador).serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cam.cerrar()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--imagen", nargs="+", help="una o más fotos guardadas")
    p.add_argument("--camara", action="store_true", help="usar la cámara en vivo (Pi)")
    p.add_argument("--n", type=int, default=10, help="cuadros a procesar con --camara")
    p.add_argument("--calibrar-dist", type=float, default=None,
                   help="distancia real (m) de UNA pelota para calcular la focal")
    p.add_argument("--mascara", action="store_true", help="guardar también la máscara")
    p.add_argument("--calibrar-color", choices=[c for c in C.CLASES if c != "VACIO"],
                   help="con UNA pelota de ese color a la vista: guarda referencias de color")
    p.add_argument("--web", action="store_true",
                   help="video en vivo con detecciones en http://<ip-de-la-pi>:PUERTO")
    p.add_argument("--puerto", type=int, default=8000)
    args = p.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")

    cfg = cargar_config()
    clasif = C.Clasificador.cargar()
    os.makedirs(C.CARPETA_CAPTURAS, exist_ok=True)

    def procesar(img, nombre):
        alto, ancho = img.shape[:2]
        objetivo = (ancho / 2, alto / 2)
        t0 = time.perf_counter()
        dets = detectar(img, cfg, clasif)
        ms = (time.perf_counter() - t0) * 1000
        print(f"{nombre}: {len(dets)} pelota(s) en {ms:.0f} ms\n  "
              + informe(dets, objetivo, cfg["focal_px"]))
        cv2.imwrite(os.path.join(C.CARPETA_CAPTURAS, f"{nombre}_det.jpg"),
                    dibujar(img, dets, objetivo, cfg["focal_px"]))
        if args.mascara:
            cv2.imwrite(os.path.join(C.CARPETA_CAPTURAS, f"{nombre}_mask.jpg"), mascara(img, cfg, clasif))
        return dets

    if args.imagen:
        for ruta in args.imagen:
            procesar(C.leer_imagen(ruta), os.path.splitext(os.path.basename(ruta))[0])
        return

    if args.web:
        servir_web(cfg, clasif, args.puerto)
        return

    if args.camara:
        from camara import Camara, cargar_ajustes
        ajustes = cargar_ajustes()
        cam = Camara((ajustes or {}).get("dist_m", 0.25))
        try:
            cam.aplicar(ajustes) if ajustes else cam.bloquear_auto()
            marca = time.strftime("%Y%m%d_%H%M%S")
            if args.calibrar_color:
                calibrar_color(cam, cfg, clasif, args.calibrar_color, args.n, marca)
                return
            n = 1 if args.calibrar_dist else args.n
            for i in range(n):
                img = cam.capturar()
                cv2.imwrite(os.path.join(C.CARPETA_CAPTURAS, f"vivo_{marca}_{i + 1}.jpg"), img)
                dets = procesar(img, f"vivo_{marca}_{i + 1}")
                time.sleep(0.2)
            if args.calibrar_dist:
                if len(dets) != 1:
                    print(f"[ERROR] Para calibrar debe verse exactamente 1 pelota (hay {len(dets)}).")
                    return
                cfg["focal_px"] = dets[0].diam_px * args.calibrar_dist / (DIAMETRO_MM / 1000.0)
                guardar_config(cfg)
                print(f"Focal calibrada: {cfg['focal_px']:.0f} px (guardada en {ARCHIVO_CONFIG})")
        finally:
            cam.cerrar()
        return

    p.print_help()


if __name__ == "__main__":
    main()
