"""Polaridad de los MOTORES y ENCODERS de las ruedas (después de desarmar y rearmar). Ruedas EN EL AIRE.

La polaridad está en el FIRMWARE (firmware/mega_cafebot/config.h: Mx_INV_MOTOR y Mx_INV_ENC): no se puede cambiar
desde Python; después de corregir config.h hay que volver a subir el firmware al Mega.

    python3 -m tools.motores              # diagnóstico: dice qué cambiar
    python3 -m tools.motores --escribir   # además lo cambia en config.h (después: subir el firmware)

1. ENCODERS: girar a mano cada rueda UNA vuelta hacia ADELANTE (la parte de arriba de la rueda hacia el frente del
   robot, donde están el LiDAR y el brazo). Con `ED` cada flanco debe sumar ~+800: si suma −800, el encoder está al
   revés (Mx_INV_ENC).
2. MOTORES: `V 60 0` durante 1 s. Con el sentido de cada encoder ya conocido, si la rueda giró hacia atrás, el motor
   está al revés (Mx_INV_MOTOR). (Si un encoder está al revés, el control se embala: la rueda gira rápido 1 s; en el
   aire no pasa nada.)
Ruedas: M1 delantera derecha, M2 delantera izquierda, M3 trasera derecha, M4 trasera izquierda.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import time

RAIZ = pathlib.Path(__file__).resolve().parents[2]
CONFIG_H = RAIZ / "firmware" / "mega_cafebot" / "config.h"
RUEDAS = {1: "delantera DERECHA", 2: "delantera IZQUIERDA", 3: "trasera DERECHA", 4: "trasera IZQUIERDA"}


def leer_banderas(texto: str) -> tuple[dict, dict]:
    mot = {int(m): v == "true" for m, v in re.findall(r"#define M([1-4])_INV_MOTOR\s+(true|false)", texto)}
    enc = {int(m): v == "1" for m, v in re.findall(r"#define M([1-4])_INV_ENC\s+([01])", texto)}
    return mot, enc


def escribir_banderas(texto: str, mot: dict, enc: dict) -> str:
    for m, v in mot.items():
        texto = re.sub(rf"(#define M{m}_INV_MOTOR\s+)(true|false)", rf"\g<1>{'true' if v else 'false'}", texto)
    for m, v in enc.items():
        texto = re.sub(rf"(#define M{m}_INV_ENC\s+)([01])", rf"\g<1>{1 if v else 0}", texto)
    return texto


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mega", default=None)
    ap.add_argument("--escribir", action="store_true", help="cambiar config.h con lo que salga")
    ap.add_argument("--vel", type=float, default=60.0)
    a = ap.parse_args()
    from cafebot.config import cargar
    from cafebot.drivers.mega import Mega
    from cafebot.drivers.puertos import resolver
    cfg = cargar()
    texto = CONFIG_H.read_text()
    mot0, enc0 = leer_banderas(texto)
    print("En config.h ahora: " + "  ".join(f"M{m}: motor {'INV' if mot0[m] else 'normal'}, encoder "
                                         f"{'INV' if enc0[m] else 'normal'}" for m in range(1, 5)))
    print("(se supone que el firmware cargado en el Mega es este config.h)\n")
    mega = Mega(resolver(a.mega or cfg.puertos.mega, "mega"))
    print("Conectando con el Mega...")
    mega.conectar()
    try:
        input("Robot con las 4 RUEDAS EN EL AIRE (sobre una caja) y nada que se pueda enredar. ENTER para seguir... ")
        mega.comando("S")
        # ---------------------------------------------------------- 1) encoders a mano
        mega.comando("ED")                     # pone en cero el diagnóstico
        input("\n1) Gira a mano CADA rueda UNA vuelta hacia ADELANTE (la parte de arriba de la rueda hacia el frente\n"
              "   del robot). Las 4, una por una. Después ENTER... ")
        r = mega.comando("ED")
        print(f"   {r}")
        flancos = {int(m): (int(s), int(b)) for m, s, b in re.findall(r"M([1-4])=(-?\d+)/(-?\d+)", r)}
        enc_al_reves, dudosos = {}, []
        for m in range(1, 5):
            s, b = flancos.get(m, (0, 0))
            tot = s + b
            if abs(s) < 150 and abs(b) < 150:
                dudosos.append(f"M{m} ({RUEDAS[m]}): casi no contó ({s}/{b}): ¿no la giraste? ¿encoder desconectado?")
                enc_al_reves[m] = None
            elif s * b < 0 and min(abs(s), abs(b)) > 150:
                dudosos.append(f"M{m} ({RUEDAS[m]}): los flancos dan signos opuestos ({s}/{b}): cable B suelto o ruido")
                enc_al_reves[m] = None
            else:
                enc_al_reves[m] = tot < 0
                print(f"   M{m} {RUEDAS[m]:20s}: {s:+5d}/{b:+5d} -> encoder "
                      + ("AL REVÉS" if tot < 0 else "bien"))
        for d in dudosos:
            print(f"   ¡OJO! {d}")
        # ---------------------------------------------------------- 2) motores con V
        input(f"\n2) Ahora las ruedas giran solas {a.vel:.0f} mm/s durante 1 s (si un encoder está al revés, esa rueda "
              "se embala: es normal). ENTER... ")
        e0 = [int(x) for x in mega.comando("E").split()[1:5]]
        fin = time.monotonic() + 1.0
        while time.monotonic() < fin:
            mega.velocidad(a.vel, 0.0)
            time.sleep(0.1)
        mega.comando("S")
        time.sleep(0.5)
        e1 = [int(x) for x in mega.comando("E").split()[1:5]]
        mot_al_reves = {}
        print()
        for m in range(1, 5):
            d = e1[m - 1] - e0[m - 1]
            inv = enc_al_reves.get(m)
            if inv is None or abs(d) < 50:
                mot_al_reves[m] = None
                print(f"   M{m} {RUEDAS[m]:20s}: encoder {d:+6d} -> no se puede saber (ver arriba / ¿el motor giró?)")
                continue
            hacia = d * (-1 if inv else 1)                # lo que giró de verdad
            mot_al_reves[m] = hacia < 0
            print(f"   M{m} {RUEDAS[m]:20s}: encoder {d:+6d} -> giró hacia {'ATRÁS: motor AL REVÉS' if hacia < 0 else 'adelante: motor bien'}"
                  + ("  (se embaló)" if abs(d) > 3000 else ""))
        # ---------------------------------------------------------- resultado
        mot1, enc1 = dict(mot0), dict(enc0)
        cambios = []
        for m in range(1, 5):
            if enc_al_reves.get(m):
                enc1[m] = not enc0[m]
                cambios.append(f"#define M{m}_INV_ENC {1 if enc1[m] else 0}")
            if mot_al_reves.get(m):
                mot1[m] = not mot0[m]
                cambios.append(f"#define M{m}_INV_MOTOR {'true' if mot1[m] else 'false'}")
        print()
        if all(enc_al_reves.get(m) and mot_al_reves.get(m) for m in range(1, 5)):
            print("¡OJO! Las 4 ruedas salen con motor Y encoder al revés: ¿la base quedó girada 180° respecto de la tapa "
                  "(el frente del robot ahora es la cola)? En ese caso NO es la polaridad: hay que volver a armarla.")
        if None in enc_al_reves.values() or None in mot_al_reves.values():
            print("Hay ruedas sin resultado: revisar lo de arriba y repetir antes de cambiar nada.")
        if not cambios:
            print("No hay nada que cambiar en la polaridad." if None not in mot_al_reves.values() else "")
            return
        print("Cambiar en firmware/mega_cafebot/config.h:\n  " + "\n  ".join(cambios))
        if a.escribir:
            nuevo = escribir_banderas(texto, mot1, enc1)
            nuevo = nuevo.replace("// Polaridad (de v0)",
                                  f"// {time.strftime('%d %b')}: tools.motores tras rearmar -> {', '.join(c.split()[1] for c in cambios)}\n"
                                  "// Polaridad (de v0)", 1)
            CONFIG_H.write_text(nuevo)
            print(f"Listo: {CONFIG_H} cambiado.")
        else:
            print("(con --escribir lo cambia solo)")
        print("\nDespués SUBIR EL FIRMWARE al Mega:")
        if shutil.which("arduino-cli"):
            print(f"  arduino-cli compile --fqbn arduino:avr:mega {CONFIG_H.parent}\n"
                  f"  arduino-cli upload -p {resolver(a.mega or cfg.puertos.mega, 'mega')} --fqbn arduino:avr:mega "
                  f"{CONFIG_H.parent}")
        else:
            print("  - desde un PC con el Arduino IDE: git pull, abrir firmware/mega_cafebot/mega_cafebot.ino, placa\n"
                  "    'Arduino Mega or Mega 2560', el puerto del Mega (conectarlo al PC) y Subir; o\n"
                  "  - en la Pi, instalar arduino-cli una vez (con internet):\n"
                  "      curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=$HOME/.local/bin sh\n"
                  "      arduino-cli core update-index && arduino-cli core install arduino:avr && arduino-cli lib install Servo\n"
                  "    y volver a correr esta herramienta (dice los comandos para compilar y subir).")
        print("Y repetir esta prueba: tiene que decir que no hay nada que cambiar.")
    finally:
        try:
            mega.comando("S")
        except Exception:
            pass
        mega.cerrar()


if __name__ == "__main__":
    main()
