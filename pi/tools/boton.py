"""Probar el botón de inicio en la Raspberry: el mismo código que usa la misión (interfaces.BotonGPIO).
5 nov: GPIO 16 = pin físico 36; el otro lado del botón a un GND (pin 34 o 39). Pull-up interno: no lleva resistencia.

    python3 -m tools.boton            # avisa cada vez que la misión partiría (al SOLTAR el botón). Ctrl+C sale
    python3 -m tools.boton --pin 24   # probar otro GPIO (numeración BCM)
"""
from __future__ import annotations

import argparse

from cafebot.config import cargar
from cafebot.mision.interfaces import BotonGPIO


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pin", type=int, default=None, help="GPIO BCM (por defecto mision.pin_boton)")
    a = ap.parse_args()
    pin = a.pin if a.pin is not None else cargar().mision.pin_boton
    b = BotonGPIO(pin)
    print(f"GPIO {pin}: ahora se lee {'APRETADO (0)' if b.b.is_pressed else 'suelto (1)'}. "
          "Si se lee apretado sin tocarlo: el cable va a GND por otro lado o el botón está trabado.")
    n = 0
    try:
        while True:
            b.esperar()
            n += 1
            print(f"  {n}: ¡la misión partiría ahora! (al soltar)")
    except KeyboardInterrupt:
        print()


if __name__ == "__main__":
    main()
