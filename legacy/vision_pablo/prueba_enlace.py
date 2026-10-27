"""Prueba del enlace Pi <-> ESP32 (con firmware/eco_serial cargado en la ESP32).

Uso en la Pi:  python3 prueba_enlace.py
               python3 prueba_enlace.py --puerto /dev/ttyUSB0
"""
import argparse
import time

from esp32 import ESP32Serial, buscar_puerto


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--puerto", default=None)
    p.add_argument("--veces", type=int, default=20)
    args = p.parse_args()

    puerto = args.puerto or buscar_puerto()
    print(f"Abriendo {puerto} (espera ~2 s, la ESP32 se reinicia al abrir)...")
    esp = ESP32Serial(puerto)

    tiempos, fallas = [], 0
    for _ in range(args.veces):
        t0 = time.monotonic()
        r = esp.comando("PING", timeout=1.0)
        if r.ok and r.datos == "PONG":
            tiempos.append((time.monotonic() - t0) * 1000)
        else:
            fallas += 1
            print("Falla:", r)
    if tiempos:
        print(f"PING ok {len(tiempos)}/{args.veces}, ida y vuelta: promedio "
              f"{sum(tiempos) / len(tiempos):.1f} ms, máximo {max(tiempos):.1f} ms")
    print("Comando desconocido (debe dar ERR):", esp.comando("HOLA", timeout=1.0))
    for _ in range(3):
        esp.comando("LED ON")
        time.sleep(0.3)
        esp.comando("LED OFF")
        time.sleep(0.3)
    print("El LED de la placa debió parpadear 3 veces." if fallas == 0 else
          f"Hubo {fallas} fallas: revisar cable, puerto y baudios.")


if __name__ == "__main__":
    main()
