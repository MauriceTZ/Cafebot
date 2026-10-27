"""Clasificador de color de granos en posiciones fijas del árbol.

Idea: el robot se detiene a distancia conocida del árbol, así que cada uno de
los 16 soportes cae siempre en la misma zona de la imagen. Para cada zona se
calcula el color mediano en espacio Lab y se compara con referencias tomadas en
una calibración en sitio (vecino más cercano). Si el color no se parece lo
suficiente a ninguna referencia, o se parece casi igual a dos clases distintas,
la respuesta es NO_SE y el robot NO toca ese grano.

Uso (en la Pi o en el PC):
  # 1. Ver dónde caen las zonas sobre una foto (y ajustar posiciones.json)
  python3 clasificador.py zonas --imagen foto.jpg

  # 2. Calibrar: agregar referencias desde fotos reales
  python3 clasificador.py calibrar --imagen foto.jpg --clase ROJO --pos baja-2
  python3 clasificador.py calibrar --imagen arbol_vacio.jpg --clase VACIO --pos todas

  # 3. Revisar que las clases calibradas se distingan entre sí
  python3 clasificador.py revisar

  # 4. Clasificar una o varias fotos (varias = todas deben coincidir)
  python3 clasificador.py clasificar --imagen f1.jpg f2.jpg f3.jpg

Para sacar fotos en la Pi sin código: rpicam-still -o foto.jpg
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np

CARPETA_CALIBRACION = "calibracion"
ARCHIVO_POSICIONES = os.path.join(CARPETA_CALIBRACION, "posiciones.json")
ARCHIVO_REFERENCIAS = os.path.join(CARPETA_CALIBRACION, "referencias.json")
CARPETA_CAPTURAS = "capturas"

CLASES = ["ROJO", "NARANJO", "AMARILLO", "VERDE", "AZUL", "NEGRO", "VACIO"]
NO_SE = "NO_SE"

CATEGORIA = {
    "ROJO": "MADURO", "NARANJO": "MADURO", "AMARILLO": "MADURO",
    "AZUL": "SOBREMADURO", "NEGRO": "SOBREMADURO",
    "VERDE": "VERDE",
    "VACIO": "VACIO",
    NO_SE: "DESCONOCIDO",
}

# Filas del árbol: alta 5, media 6, baja 5. Numeradas de izquierda a derecha
# tal como se ven en la imagen.
FILAS = {"alta": 5, "media": 6, "baja": 5}

# Parámetros de decisión (PROVISIONALES: ajustar con `revisar` tras calibrar).
CONFIG_DEFECTO = {
    "umbral_distancia": 20.0,  # más lejos que esto de toda referencia -> NO_SE
    "margen_ambiguedad": 6.0,  # 2.ª clase casi tan cerca como la 1.ª -> NO_SE
    "peso_L": 0.5,             # pesa menos la luminosidad (sombras, brillo)
    "fraccion_radio": 0.6,     # se usa solo el centro de cada zona circular
    "L_brillo_max": 95.0,      # se descartan reflejos casi blancos
}

# Colores para dibujar (BGR)
COLOR_DIBUJO = {
    "ROJO": (0, 0, 255), "NARANJO": (0, 140, 255), "AMARILLO": (0, 230, 255),
    "VERDE": (0, 200, 0), "AZUL": (255, 80, 0), "NEGRO": (60, 60, 60),
    "VACIO": (200, 200, 200), NO_SE: (255, 0, 255),
}


@dataclass
class Resultado:
    clase: str
    distancia: float
    segunda_clase: str
    segunda_distancia: float
    lab: tuple

    @property
    def categoria(self) -> str:
        return CATEGORIA[self.clase]


# ---------------------------------------------------------------- posiciones

def nombres_posiciones() -> list:
    return [f"{fila}-{i}" for fila, n in FILAS.items() for i in range(1, n + 1)]


def generar_rejilla(ancho: int, alto: int) -> dict:
    """Posiciones provisionales repartidas en la imagen, solo para empezar.
    Deben reemplazarse por las medidas reales sobre una foto del árbol."""
    posiciones = {}
    filas_y = {"alta": 0.25, "media": 0.50, "baja": 0.75}
    radio = int(min(ancho, alto) * 0.05)
    for fila, n in FILAS.items():
        for i in range(n):
            x = int(ancho * (i + 1) / (n + 1))
            posiciones[f"{fila}-{i + 1}"] = [x, int(alto * filas_y[fila]), radio]
    return posiciones


def cargar_posiciones(ruta: str, imagen=None) -> dict:
    """{nombre: [x, y, radio]} en píxeles. Si no existe el archivo, se crea
    una rejilla provisional con el tamaño de la imagen."""
    if os.path.exists(ruta):
        with open(ruta, encoding="utf-8") as f:
            return json.load(f)
    if imagen is None:
        raise FileNotFoundError(f"No existe {ruta}")
    alto, ancho = imagen.shape[:2]
    posiciones = generar_rejilla(ancho, alto)
    os.makedirs(os.path.dirname(ruta), exist_ok=True)
    with open(ruta, "w", encoding="utf-8") as f:
        json.dump(posiciones, f, indent=2)
    print(f"[AVISO] No había {ruta}: se creó una rejilla PROVISIONAL. Ajústala.")
    return posiciones


# ---------------------------------------------------------------- color

def imagen_a_lab(imagen_bgr) -> np.ndarray:
    """Lab real: L en 0..100, a y b aprox. en -127..127."""
    return cv2.cvtColor(imagen_bgr.astype(np.float32) / 255.0, cv2.COLOR_BGR2Lab)


def color_zona(lab: np.ndarray, x: int, y: int, radio: int, config: dict) -> tuple:
    """Lab mediano del centro de una zona circular, sin reflejos."""
    r = max(2, int(radio * config["fraccion_radio"]))
    mascara = np.zeros(lab.shape[:2], np.uint8)
    cv2.circle(mascara, (int(x), int(y)), r, 255, -1)
    pixeles = lab[mascara > 0]
    if len(pixeles) == 0:
        raise ValueError(f"Zona ({x},{y}) fuera de la imagen")
    sin_brillo = pixeles[pixeles[:, 0] < config["L_brillo_max"]]
    if len(sin_brillo) >= 0.3 * len(pixeles):
        pixeles = sin_brillo
    return tuple(float(v) for v in np.median(pixeles, axis=0))


def distancia(lab1, lab2, peso_L: float) -> float:
    dL = (lab1[0] - lab2[0]) * peso_L
    return math.sqrt(dL * dL + (lab1[1] - lab2[1]) ** 2 + (lab1[2] - lab2[2]) ** 2)


# ---------------------------------------------------------------- clasificador

class Clasificador:
    def __init__(self, muestras=None, config=None):
        self.muestras = muestras or []  # [{"clase", "lab", "origen"}]
        self.config = dict(CONFIG_DEFECTO)
        self.config.update(config or {})

    @classmethod
    def cargar(cls, ruta: str = ARCHIVO_REFERENCIAS) -> "Clasificador":
        if not os.path.exists(ruta):
            return cls()
        with open(ruta, encoding="utf-8") as f:
            datos = json.load(f)
        return cls(datos.get("muestras", []), datos.get("config", {}))

    def guardar(self, ruta: str = ARCHIVO_REFERENCIAS):
        os.makedirs(os.path.dirname(ruta), exist_ok=True)
        with open(ruta, "w", encoding="utf-8") as f:
            json.dump({"config": self.config, "muestras": self.muestras}, f, indent=2)

    def agregar(self, clase: str, lab: tuple, origen: str = ""):
        if clase not in CLASES:
            raise ValueError(f"Clase desconocida: {clase}. Usa una de {CLASES}")
        self.muestras.append({"clase": clase, "lab": [round(v, 2) for v in lab],
                              "origen": origen})

    def clasificar_lab(self, lab: tuple) -> Resultado:
        """Vecino más cercano por clase, con rechazo conservador."""
        mejor = {}  # clase -> menor distancia a una muestra de esa clase
        for m in self.muestras:
            d = distancia(lab, m["lab"], self.config["peso_L"])
            if d < mejor.get(m["clase"], math.inf):
                mejor[m["clase"]] = d
        if not mejor:
            return Resultado(NO_SE, math.inf, NO_SE, math.inf, lab)
        orden = sorted(mejor.items(), key=lambda kv: kv[1])
        clase1, d1 = orden[0]
        clase2, d2 = orden[1] if len(orden) > 1 else (NO_SE, math.inf)
        clase = clase1
        if d1 > self.config["umbral_distancia"]:
            clase = NO_SE
        elif d2 - d1 < self.config["margen_ambiguedad"] and CATEGORIA[clase1] != CATEGORIA[clase2]:
            # Dudar entre rojo y naranjo da igual (ambos maduros);
            # dudar entre naranjo y verde NO.
            clase = NO_SE
        return Resultado(clase, d1, clase2, d2, lab)

    def clasificar_imagen(self, imagen_bgr, posiciones: dict) -> dict:
        lab = imagen_a_lab(imagen_bgr)
        return {nombre: self.clasificar_lab(color_zona(lab, x, y, r, self.config))
                for nombre, (x, y, r) in posiciones.items()}

    def clasificar_varias(self, imagenes: list, posiciones: dict) -> dict:
        """Clasifica varias fotos del mismo árbol. Si una posición no da la
        misma clase en todas, queda NO_SE (filtro de estabilidad)."""
        por_foto = [self.clasificar_imagen(img, posiciones) for img in imagenes]
        final = {}
        for nombre in posiciones:
            resultados = [r[nombre] for r in por_foto]
            clases = {r.clase for r in resultados}
            peor = max(resultados, key=lambda r: r.distancia)
            if len(clases) == 1:
                final[nombre] = peor
            else:
                final[nombre] = Resultado(NO_SE, peor.distancia, peor.segunda_clase,
                                          peor.segunda_distancia, peor.lab)
        return final


# ---------------------------------------------------------------- dibujo

def dibujar(imagen_bgr, posiciones: dict, resultados: dict = None):
    salida = imagen_bgr.copy()
    for nombre, (x, y, r) in posiciones.items():
        res = resultados.get(nombre) if resultados else None
        color = COLOR_DIBUJO[res.clase] if res else (255, 255, 255)
        cv2.circle(salida, (x, y), r, color, 2)
        texto = nombre if res is None else f"{nombre} {res.clase} {res.distancia:.0f}"
        cv2.putText(salida, texto, (x - r, y - r - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, (0, 0, 0), 3)
        cv2.putText(salida, texto, (x - r, y - r - 6), cv2.FONT_HERSHEY_SIMPLEX,
                    0.45, color, 1)
    return salida


def guardar_captura(imagen, nombre_base: str) -> str:
    os.makedirs(CARPETA_CAPTURAS, exist_ok=True)
    ruta = os.path.join(CARPETA_CAPTURAS, nombre_base)
    cv2.imwrite(ruta, imagen)
    return ruta


def leer_imagen(ruta: str):
    imagen = cv2.imread(ruta)
    if imagen is None:
        raise FileNotFoundError(f"No se pudo leer la imagen {ruta}")
    return imagen


# ---------------------------------------------------------------- comandos

def cmd_zonas(args):
    imagen = leer_imagen(args.imagen[0])
    posiciones = cargar_posiciones(args.posiciones, imagen)
    base = os.path.splitext(os.path.basename(args.imagen[0]))[0]
    print("Guardada:", guardar_captura(dibujar(imagen, posiciones), f"{base}_zonas.jpg"))


def cmd_calibrar(args):
    clasif = Clasificador.cargar(args.referencias)
    for ruta in args.imagen:
        imagen = leer_imagen(ruta)
        posiciones = cargar_posiciones(args.posiciones, imagen)
        nombres = list(posiciones) if args.pos == ["todas"] else args.pos
        lab = imagen_a_lab(imagen)
        for nombre in nombres:
            x, y, r = posiciones[nombre]
            valor = color_zona(lab, x, y, r, clasif.config)
            clasif.agregar(args.clase, valor, f"{os.path.basename(ruta)}:{nombre}")
            print(f"{args.clase:9s} <- {ruta}:{nombre}  Lab=({valor[0]:.1f}, "
                  f"{valor[1]:.1f}, {valor[2]:.1f})")
    clasif.guardar(args.referencias)
    print(f"Total de muestras: {len(clasif.muestras)} en {args.referencias}")


def cmd_revisar(args):
    """Muestra qué tan separadas quedaron las clases calibradas."""
    clasif = Clasificador.cargar(args.referencias)
    por_clase = {}
    for m in clasif.muestras:
        por_clase.setdefault(m["clase"], []).append(m["lab"])
    print(f"{'clase':9s} n   L      a      b    (promedio)")
    for clase, labs in por_clase.items():
        p = np.mean(labs, axis=0)
        print(f"{clase:9s} {len(labs):<3d} {p[0]:5.1f}  {p[1]:6.1f} {p[2]:6.1f}")
    faltan = [c for c in CLASES if c not in por_clase]
    if faltan:
        print("FALTAN clases por calibrar:", ", ".join(faltan))
    # Distancia mínima entre muestras de clases de distinta categoría:
    # si es menor que umbral, esas clases se pueden confundir.
    peso = clasif.config["peso_L"]
    print("\nPares más cercanos entre categorías distintas (peligrosos si < "
          f"{clasif.config['umbral_distancia']:.0f}):")
    pares = []
    for i, a in enumerate(clasif.muestras):
        for b in clasif.muestras[i + 1:]:
            if CATEGORIA[a["clase"]] != CATEGORIA[b["clase"]]:
                pares.append((distancia(a["lab"], b["lab"], peso), a, b))
    for d, a, b in sorted(pares, key=lambda t: t[0])[:8]:
        print(f"  {d:6.1f}  {a['clase']}({a['origen']})  vs  {b['clase']}({b['origen']})")


def cmd_clasificar(args):
    clasif = Clasificador.cargar(args.referencias)
    if not clasif.muestras:
        print("[AVISO] No hay referencias: todo saldrá NO_SE. Corre `calibrar` primero.")
    imagenes = [leer_imagen(r) for r in args.imagen]
    posiciones = cargar_posiciones(args.posiciones, imagenes[0])
    resultados = clasif.clasificar_varias(imagenes, posiciones)
    for nombre, res in resultados.items():
        print(f"{nombre:8s} {res.clase:9s} {res.categoria:12s} d={res.distancia:6.1f}"
              f"  (2.ª: {res.segunda_clase} {res.segunda_distancia:.1f})")
    base = os.path.splitext(os.path.basename(args.imagen[0]))[0]
    ruta = guardar_captura(dibujar(imagenes[0], posiciones, resultados),
                           f"{base}_clasificado.jpg")
    print("Guardada:", ruta)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--posiciones", default=ARCHIVO_POSICIONES)
    p.add_argument("--referencias", default=ARCHIVO_REFERENCIAS)
    sub = p.add_subparsers(dest="comando", required=True)

    s = sub.add_parser("zonas", help="dibuja las zonas sobre una foto")
    s.add_argument("--imagen", nargs=1, required=True)
    s.set_defaults(f=cmd_zonas)

    s = sub.add_parser("calibrar", help="agrega referencias de color")
    s.add_argument("--imagen", nargs="+", required=True)
    s.add_argument("--clase", required=True, choices=CLASES)
    s.add_argument("--pos", nargs="+", required=True,
                   help="nombres como baja-2, o 'todas'")
    s.set_defaults(f=cmd_calibrar)

    s = sub.add_parser("revisar", help="muestra la separación entre clases")
    s.set_defaults(f=cmd_revisar)

    s = sub.add_parser("clasificar", help="clasifica una o varias fotos")
    s.add_argument("--imagen", nargs="+", required=True)
    s.set_defaults(f=cmd_clasificar)

    args = p.parse_args()
    sys.stdout.reconfigure(encoding="utf-8")  # acentos en la consola de Windows
    args.f(args)


if __name__ == "__main__":
    main()
