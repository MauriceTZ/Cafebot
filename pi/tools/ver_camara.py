"""Ver la cámara en el NAVEGADOR (PC o celular, sin VNC) y calibrarla con el tablero de ajedrez.

En la Raspberry, desde pi/:
    python3 -m tools.ver_camara                         # video en vivo
    python3 -m tools.ver_camara --tablero 9x6 --cuadro 0.025
y abrir http://<ip-de-la-pi>:8000 (la herramienta muestra la dirección). Ctrl+C para terminar.

Con --tablero: dibuja las esquinas encontradas (verde = se ve entero) y, en azul, las partes de la imagen que
ya cubren las fotos guardadas. Botones: "Sacar foto" (solo si ve el tablero) y "Calcular": usa las mismas
funciones que `tools.camara intrinseca` (mismo foco fijo que la cosecha) y guarda en robot_local.json.
Consejo: 12–20 fotos a 40–80 cm, cubriendo centro, esquinas y bordes, inclinando el tablero ±30°.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

from cafebot.config import cargar, guardar_local
from cafebot.vision import clasificador as cl
from tools.camara import _camara, calibrar_intrinsecos, esquinas_tablero

PAGINA = """<!doctype html><html lang="es"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Cámara cafebot</title>
<style>body{font-family:sans-serif;margin:0;padding:12px;background:#111;color:#eee}
img{width:100%;max-width:1100px;display:block;border-radius:6px}
button{font-size:1.1em;padding:10px 18px;margin:8px 8px 0 0;border:0;border-radius:6px;background:#2a7;color:#fff}
#calc{background:#27a}#msg{margin-top:10px;white-space:pre-wrap;font-family:monospace}</style></head><body>
<img src="/video"><div>BOTONES</div><div id="msg"></div>
<script>
async function ir(u){document.getElementById('msg').textContent='...';
 const r=await fetch(u);document.getElementById('msg').textContent=await r.text();}
</script></body></html>"""
BOTONES = ('<button onclick="ir(\'/foto\')">Sacar foto</button>'
           '<button id="calc" onclick="ir(\'/calcular\')">Calcular</button>')


class Visor:
    def __init__(self, cam, tablero=None, cuadro=0.025, ancho_vista=960):
        self.cam, self.tablero, self.cuadro, self.ancho_vista = cam, tablero, cuadro, ancho_vista
        self.lock = threading.Lock()
        self.ultima = None            # foto completa (BGR)
        self.jpeg = None
        self.fotos = []               # fotos completas con el tablero
        self.cobertura = None         # máscara de lo cubierto (tamaño vista)
        self.ms = 0.0
        self.corriendo = True

    def bucle(self):
        while self.corriendo:
            t0 = time.monotonic()
            img = self.cam.capturar()
            esc = self.ancho_vista / img.shape[1]
            vista = cv2.resize(img, None, fx=esc, fy=esc, interpolation=cv2.INTER_AREA)
            texto = f"{1000 * self.ms:.0f} ms"
            if self.tablero:
                if self.cobertura is None:
                    self.cobertura = np.zeros(vista.shape[:2], np.uint8)
                azul = vista.copy()
                azul[self.cobertura > 0] = (180, 90, 0)
                vista = cv2.addWeighted(azul, 0.35, vista, 0.65, 0)
                gris = cv2.cvtColor(vista, cv2.COLOR_BGR2GRAY)
                ok, esq = cv2.findChessboardCorners(gris, self.tablero, cv2.CALIB_CB_FAST_CHECK)
                cv2.drawChessboardCorners(vista, self.tablero, esq, ok)
                texto += f" | tablero {'VISTO' if ok else 'no visto'} | fotos guardadas: {len(self.fotos)}"
            cv2.putText(vista, texto, (10, vista.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            ok, buf = cv2.imencode(".jpg", vista, [cv2.IMWRITE_JPEG_QUALITY, 80])
            with self.lock:
                self.ultima, self.jpeg = img, buf.tobytes()
            self.ms = time.monotonic() - t0

    def foto(self) -> str:
        with self.lock:
            img = None if self.ultima is None else self.ultima.copy()
        if img is None:
            return "todavía no hay imagen"
        esq = esquinas_tablero(img, *self.tablero)
        if esq is None:
            return "no se encontró el tablero entero en la foto: muévelo y prueba de nuevo"
        self.fotos.append(img)
        os.makedirs(cl.CARPETA_CAPTURAS, exist_ok=True)
        cv2.imwrite(os.path.join(cl.CARPETA_CAPTURAS, time.strftime(f"intrinseca_%H%M%S_{len(self.fotos)}.jpg")), img)
        esc = self.ancho_vista / img.shape[1]
        casco = cv2.convexHull((esq.reshape(-1, 2) * esc).astype(np.int32))
        cv2.fillConvexPoly(self.cobertura, casco, 255)
        cubierto = 100 * float((self.cobertura > 0).mean())
        return (f"foto {len(self.fotos)} guardada. Imagen cubierta: {cubierto:.0f} %. "
                f"{'Faltan fotos (mínimo 6, mejor 12-20).' if len(self.fotos) < 12 else 'Ya puedes calcular.'}")

    def calcular(self, cfg) -> str:
        if len(self.fotos) < 6:
            return f"hay {len(self.fotos)} fotos: hacen falta al menos 6 (mejor 12-20)"
        vals, rms, n = calibrar_intrinsecos(self.fotos, *self.tablero, self.cuadro, cfg.camara.ancho_px, cfg.camara.alto_px)
        campo = 2 * math.degrees(math.atan(cfg.camara.ancho_px / 2 / vals["fx_px"]))
        msg = (f"{n} fotos, error {rms:.2f} px\nfx {vals['fx_px']}  fy {vals['fy_px']}  cx {vals['cx_px']}  "
               f"cy {vals['cy_px']}\ncampo horizontal {campo:.1f}° (de fábrica 66° = fx 1183)\ndist {vals['dist']}\n")
        if rms > 1.0:
            msg += "AVISO: error alto (¿tablero doblado, fotos movidas, --tablero/--cuadro equivocados?). NO se guardó.\n"
            return msg
        if campo < 55:
            msg += "AVISO: campo angosto: la cámara está en el modo recortado del sensor.\n"
        return msg + f"Guardado en {guardar_local('camara', vals)}"


def servir(visor: Visor, cfg, puerto: int):
    botones = BOTONES if visor.tablero else ""

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _texto(self, t, tipo="text/plain; charset=utf-8"):
            b = t.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", tipo)
            self.send_header("Content-Length", str(len(b)))
            self.end_headers()
            self.wfile.write(b)

        def do_GET(self):
            if self.path == "/":
                return self._texto(PAGINA.replace("BOTONES", botones), "text/html; charset=utf-8")
            if self.path == "/foto" and visor.tablero:
                return self._texto(visor.foto())
            if self.path == "/calcular" and visor.tablero:
                try:
                    return self._texto(visor.calcular(cfg))
                except SystemExit as e:
                    return self._texto(str(e))
            if self.path == "/video":
                self.send_response(200)
                self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=cuadro")
                self.end_headers()
                try:
                    while visor.corriendo:
                        with visor.lock:
                            j = visor.jpeg
                        if j:
                            self.wfile.write(b"--cuadro\r\nContent-Type: image/jpeg\r\n\r\n" + j + b"\r\n")
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                    pass
                return
            self.send_error(404)

    srv = ThreadingHTTPServer(("", puerto), H)
    try:
        ip = socket.gethostbyname(socket.gethostname() + ".local")
    except OSError:
        ip = "<ip-de-la-pi>"
    print(f"Abre en el navegador: http://{ip}:{puerto}   (Ctrl+C para terminar)")
    srv.serve_forever()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--puerto", type=int, default=8000)
    ap.add_argument("--tablero", default=None, help="esquinas INTERIORES, p. ej. 9x6 (activa la calibración)")
    ap.add_argument("--cuadro", type=float, default=0.025, help="lado de cada cuadro en m")
    ap.add_argument("--imagen", nargs="+", default=None, help="fotos en vez de la cámara (pruebas)")
    a = ap.parse_args()
    cfg = cargar()
    if a.imagen:
        from cafebot.vision.camara import CamaraArchivo
        cam = CamaraArchivo(a.imagen)
    else:
        cam = _camara(cfg.camara.foco_m)        # el mismo foco fijo que la cosecha
    tablero = tuple(int(x) for x in a.tablero.lower().split("x")) if a.tablero else None
    visor = Visor(cam, tablero, a.cuadro)
    threading.Thread(target=visor.bucle, daemon=True).start()
    try:
        servir(visor, cfg, a.puerto)
    except KeyboardInterrupt:
        pass
    finally:
        visor.corriendo = False
        cam.cerrar()


if __name__ == "__main__":
    main()
