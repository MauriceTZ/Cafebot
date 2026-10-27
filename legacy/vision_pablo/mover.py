"""Manda comandos a la ESP32 desde la Pi y muestra las respuestas.
Sirve con prueba_base (y con cualquier firmware que responda DONE/ERR).

Uso:  python3 mover.py "m 0 100 1000"
      python3 mover.py "f 120 2000" "s"
      python3 mover.py "?"
"""
import sys

from esp32 import ESP32Serial


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    esp = ESP32Serial(log=lambda m: print(" ", m))
    for cmd in sys.argv[1:]:
        print(f"> {cmd}")
        r = esp.comando(cmd, timeout=8.0)
        print("  DONE" if r.ok else f"  ERR {r.datos}")


if __name__ == "__main__":
    main()
