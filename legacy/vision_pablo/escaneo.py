"""Corre comandos largos de firmware/escaneo_pines y muestra todo en vivo.

Uso en la Pi:  python3 -u escaneo.py list
               python3 -u escaneo.py "enc 15000"
               python3 -u escaneo.py auto
"""
import sys

from esp32 import ESP32Serial


def main():
    esp = ESP32Serial(log=lambda m: print(m, flush=True))
    for cmd in sys.argv[1:]:
        print(f"> {cmd}", flush=True)
        r = esp.comando(cmd, timeout=120.0)
        print("DONE" if r.ok else f"ERR {r.datos}", flush=True)


if __name__ == "__main__":
    main()
