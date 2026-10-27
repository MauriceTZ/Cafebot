"""Mover la base con el teclado (para probar el sistema montado).

Requiere el firmware esp32_juancar cargado en la ESP32.

Uso (desde el PC, la -t es necesaria para leer las teclas):
  ssh -t cafebot "cd ~/cafebot && python3 teleop.py"
o desde una terminal en el escritorio de la Pi:
  cd ~/cafebot && python3 teleop.py

Teclas:
  ↑ / ↓      avanzar / retroceder
  ← / →      moverse de lado (izquierda / derecha)
  A / D      girar (izquierda / derecha)
  espacio    PARAR
  + / -      más / menos velocidad
  R          poner la posición en (0, 0, 0)
  Q          salir (frena antes de salir)

El movimiento sigue hasta que se presione otra tecla o espacio. El programa
repite la orden cada 100 ms: si se corta (Ctrl+C, se cae el SSH, etc.), la
ESP32 deja de recibir órdenes y frena sola en 0,5 s.
"""
import curses
import time

from esp32 import ESP32Serial

PERIODO_S = 0.10            # cada cuánto se repite la orden (la ESP32 frena a los 0,5 s)
PERIODO_ESTADO_S = 0.5      # cada cuánto se pide la posición
NIVELES = [0.2, 0.35, 0.5, 0.75, 1.0]  # fracción de VELOCIDAD_NAVEGACION


def main(pantalla):
    curses.curs_set(0)
    pantalla.nodelay(True)
    pantalla.keypad(True)
    mensajes = []
    esp = ESP32Serial(log=lambda m: mensajes.append(m))

    nivel = 1
    direccion = (0, 0, 0)  # (adelante, lado, giro) en -1, 0 o 1
    estado = "(sin datos)"
    ultimo_envio = ultimo_estado = 0.0

    def dibujar():
        pantalla.erase()
        pantalla.addstr(0, 0, "cafebot - teleoperación   (Q para salir)")
        pantalla.addstr(2, 0, "↑↓ avanzar/retroceder   ←→ de lado   A/D girar   espacio PARAR")
        pantalla.addstr(3, 0, "+/- velocidad   R posición a cero")
        v = NIVELES[nivel]
        pantalla.addstr(5, 0, f"velocidad: {v:.2f}   orden: adelante={direccion[0] * v:+.2f} "
                              f"lado={direccion[1] * v:+.2f} giro={direccion[2] * v:+.2f}")
        pantalla.addstr(6, 0, f"estado ESP32: {estado}")
        for i, m in enumerate(mensajes[-6:]):
            pantalla.addstr(8 + i, 0, m[:100])
        pantalla.refresh()

    try:
        while True:
            tecla = pantalla.getch()
            if tecla != -1:
                if tecla in (ord("q"), ord("Q")):
                    break
                elif tecla == curses.KEY_UP:
                    direccion = (1, 0, 0)
                elif tecla == curses.KEY_DOWN:
                    direccion = (-1, 0, 0)
                elif tecla == curses.KEY_LEFT:
                    direccion = (0, 1, 0)
                elif tecla == curses.KEY_RIGHT:
                    direccion = (0, -1, 0)
                elif tecla in (ord("a"), ord("A")):
                    direccion = (0, 0, 1)
                elif tecla in (ord("d"), ord("D")):
                    direccion = (0, 0, -1)
                elif tecla in (ord("+"), ord("=")):
                    nivel = min(nivel + 1, len(NIVELES) - 1)
                elif tecla in (ord("-"), ord("_")):
                    nivel = max(nivel - 1, 0)
                elif tecla in (ord("r"), ord("R")):
                    esp.comando("r", timeout=1.0)
                else:  # espacio o cualquier otra tecla: parar
                    direccion = (0, 0, 0)
                ultimo_envio = 0.0  # mandar la orden nueva de inmediato

            ahora = time.monotonic()
            if ahora - ultimo_envio >= PERIODO_S:
                # Parado se manda "v 0 0 0" y no "q": QUIETO borra la posición.
                v = NIVELES[nivel]
                a, b, c = (d * v for d in direccion)
                r = esp.comando(f"v {a:.2f} {b:.2f} {c:.2f}", timeout=1.0)
                if not r.ok:
                    mensajes.append(f"ERR {r.datos}")
                ultimo_envio = ahora
            if ahora - ultimo_estado >= PERIODO_ESTADO_S:
                antes = len(mensajes)
                esp.comando("?", timeout=1.0)
                # La línea de estado llega como mensaje informativo antes del DONE.
                nuevas = [m for m in mensajes[antes:] if "pos:" in m]
                if nuevas:
                    estado = nuevas[-1].replace("[ESP32] ", "")
                    del mensajes[antes:]
                ultimo_estado = ahora
            dibujar()
            time.sleep(0.02)
    finally:
        esp.comando("q", timeout=1.0)


if __name__ == "__main__":
    curses.wrapper(main)
