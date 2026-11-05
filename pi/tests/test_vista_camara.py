"""Vista de la cámara en la interfaz de navegación, SIN robot.   python3 -m tests.test_vista_camara

  1. Cámara simulada (cafebot/vision/render.py): desde la posición de la foto del árbol 2 se ven los 16 granos donde
     los proyecta el modelo de la cámara, con su color; el que saca el brazo simulado desaparece; desde la partida,
     el piso de la cancha abajo y el fondo arriba del horizonte (recorte de los polígonos detrás de la cámara).
  2. Superposición: con la pose real, los soportes caen sobre los granos dibujados; corrida 10 cm, ya no.
  3. El driver del Mega recuerda el ángulo de cada servo auxiliar (X 2 = inclinación de la cámara), como el firmware.
  4. CamaraCompartida: se abre una vez y al usarla, las capturas van de a una (misión + vista), `reciente` no saca
     fotos de más, y un error al abrir se informa y se reintenta.
  5. VistaCamara con una cámara "real" (una foto guardada) y tools.navegacion --sim con la vista abajo a la derecha.
"""
import os
import subprocess
import sys
import tempfile
import threading
import time

os.environ["SDL_VIDEODRIVER"] = "dummy"

import numpy as np   # noqa: E402

from cafebot.config import cargar   # noqa: E402
from cafebot.mundo import cancha   # noqa: E402
from cafebot.vision import render   # noqa: E402
from cafebot.vision.arbol import ModeloArbol   # noqa: E402
from cafebot.vision.camara_modelo import CamaraPinhole   # noqa: E402
from cafebot.vision.sintetica import BGR   # noqa: E402

RAIZ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def escenario(semilla=1):
    return cancha.Escenario(3, 1, 5, (2, 4), cancha.sorteo_granos(np.random.default_rng(semilla)))


def pose_foto(cfg, arbol=2):
    """Donde la misión saca la foto del árbol (mision._arbol): la cámara centrada en el árbol."""
    return np.array([cancha.ARBOL_X_FRENTE - cfg.mision.distancia_foto_m - cfg.geometria.largo / 2,
                     cancha.centros_arboles()[arbol - 1] - cfg.camara.y, 0.0])


def fraccion_color(img, uv, r, color, tol=60):
    """Fracción de los píxeles del disco (uv, r) parecidos a `color`."""
    h, w = img.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    m = (xx - uv[0]) ** 2 + (yy - uv[1]) ** 2 <= (0.8 * r) ** 2
    if not m.any():
        return 0.0
    d = np.abs(img[m].astype(int) - np.array(color)).max(axis=1)
    return float(np.mean(d < tol))


def prueba_render(cfg):
    esc = escenario()
    mundo = render.MundoCancha(esc)
    cam = CamaraPinhole.desde_config(cfg.camara, cfg.camara.inclinacion_arbol_deg).con(dist=np.zeros(5))
    pose = pose_foto(cfg)
    t0 = time.perf_counter()
    img = render.dibujar(mundo, pose, cam)
    dt = time.perf_counter() - t0
    vista = render.Vista(cam, pose)
    minimos = {"baja": 1.0, "media": 1.0, "alta": 1.0}
    for s in cancha.soportes(2):
        cat, color = esc.granos[s.clave]
        Pc = vista.a_cam([s.x, s.y, s.z])
        uv = vista.uv(Pc)[0]
        r = cam.f * 0.021 / np.linalg.norm(Pc[0])
        assert cam.dentro(uv[None, :])[0], f"{s.clave} fuera de la imagen"
        f = fraccion_color(img, uv, r, BGR[color.upper()])
        minimos[s.fila] = min(minimos[s.fila], f)
    print(f"  foto del árbol 2 ({img.shape[1]}x{img.shape[0]}, {1000 * dt:.0f} ms): los 16 granos con su color; "
          "fracción mínima del disco por fila: " + ", ".join(f"{k} {v:.0%}" for k, v in minimos.items()))
    # la fila alta se ve desde abajo: su soporte tapa la parte de abajo del grano
    assert minimos["baja"] > 0.6 and minimos["media"] > 0.5 and minimos["alta"] > 0.2, minimos

    s = cancha.soportes(2)[7]                                   # el brazo simulado se lo lleva
    color = esc.granos.pop(s.clave)[1]
    img2 = render.dibujar(mundo, pose, cam)
    Pc = vista.a_cam([s.x, s.y, s.z])
    f = fraccion_color(img2, vista.uv(Pc)[0], cam.f * 0.021 / np.linalg.norm(Pc[0]), BGR[color.upper()])
    assert f < 0.05, f
    print(f"  sacado {ModeloArbol.nombre_de(s.clave[1:])} ({color}): ya no se ve")

    # desde la partida (cámara al frente, 5° hacia arriba): piso de la cancha abajo, fondo sobre el horizonte
    partida = np.array([cancha.ZONA_X_CENTRO, cancha.centros_zonas_beneficiadero()[2], 0.0])
    cam5 = CamaraPinhole.desde_config(cfg.camara, cfg.camara.pitch_de_servo(90)).con(dist=np.zeros(5))
    img3 = render.dibujar(render.MundoCancha(escenario()), partida, cam5)
    abajo, arriba = img3[cam5.alto - 20, cam5.ancho // 2], img3[25, 25]
    assert np.abs(abajo.astype(int) - render.PISO_CANCHA).max() < 10, abajo
    assert np.abs(arriba.astype(int) - render.FONDO).max() < 10, arriba
    print("  desde la partida: piso de la cancha abajo, fondo arriba (el piso que queda detrás de la cámara se recorta)")


def prueba_superposicion(cfg):
    esc = escenario(2)
    mundo = render.MundoCancha(esc)
    cam = CamaraPinhole.desde_config(cfg.camara, cfg.camara.inclinacion_arbol_deg).con(dist=np.zeros(5))
    cam = cam.escalada(768, 432)
    pose = pose_foto(cfg)
    base = render.dibujar(mundo, pose, cam)
    vista = render.Vista(cam, pose)
    h, w = base.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w]
    granos = []      # (centro del grano dibujado, radio): donde el dibujo lo puso (prueba 1: ahí está su color)
    for s in cancha.soportes(2):
        Pc = vista.a_cam([s.x, s.y, s.z])
        granos.append((vista.uv(Pc)[0], cam.f * 0.021 / np.linalg.norm(Pc[0])))

    def desvio(pose_cancha):
        """Mediana de la distancia entre el centro de cada anillo y el centro del grano dibujado (px)."""
        img = base.copy()
        render.superponer_cancha(img, cam, pose_cancha, {})
        anillo = np.all(img == render.SIN_LEER, axis=2) & ~np.all(base == render.SIN_LEER, axis=2)
        out = []
        for c, r in granos:
            m = anillo & ((xx - c[0]) ** 2 + (yy - c[1]) ** 2 < (2.2 * r) ** 2)
            out.append(np.hypot(xx[m].mean() - c[0], yy[m].mean() - c[1]) if m.any() else 99.0)
        return float(np.median(out))

    bien = desvio(pose)
    mal = desvio(pose + np.array([0.0, 0.03, 0.0]))            # el robot CREE estar 3 cm más a la izquierda
    r = float(np.median([g[1] for g in granos]))
    print(f"  anillos centrados en los granos dibujados ({bien:.1f} px; radio ~{r:.0f} px); con la pose corrida "
          f"3 cm: {mal:.1f} px")
    assert bien < 1.0 and mal > 0.6 * r, (bien, mal, r)


def prueba_aux_mega():
    from cafebot.drivers.mega import Mega
    from tests.sim_serial import SerialSimulado, compilar
    compilar()
    m = Mega("sim")
    m.conectar(serial_obj=SerialSimulado())
    try:
        assert m.aux == {}
        assert m.comando("X 2 80").startswith("OK") and m.aux[2] == 80
        assert m.comando("X 2 250").startswith("OK") and m.aux[2] == 100       # el firmware lo deja en 60..100 (5 nov)
        assert m.comando("X 3 45").startswith("OK") and m.aux[3] == 50          # 5 nov: el giro anda, 50..150
        print(f"  servos auxiliares recordados: {m.aux}")
    finally:
        m.cerrar()


class CamaraLenta:
    """Cámara falsa: tarda en capturar y detecta si dos capturas se cruzan."""

    def __init__(self):
        self.n, self.ocupada, self.cruces = 0, False, 0

    def capturar(self):
        if self.ocupada:
            self.cruces += 1
        self.ocupada = True
        time.sleep(0.02)
        self.n += 1
        self.ocupada = False
        return np.full((864, 1536, 3), self.n % 255, np.uint8)

    def enfocar(self, d):
        pass

    def cerrar(self):
        pass


def prueba_compartida():
    from cafebot.vision.camara import CamaraCompartida
    aperturas = []
    fake = CamaraLenta()

    def abrir():
        aperturas.append(1)
        if len(aperturas) == 1:
            raise RuntimeError("Camera __init__ sequence did not complete")
        return fake

    cam = CamaraCompartida(abrir)
    assert not cam.abierta and not aperturas                       # no se abre hasta usarla
    try:
        cam.capturar()
        raise AssertionError("debía fallar")
    except RuntimeError:
        assert "did not complete" in cam.error
    hilos = [threading.Thread(target=lambda: [cam.capturar() for _ in range(10)]) for _ in range(3)]
    hilos.append(threading.Thread(target=lambda: [cam.enfocar(0.5) or cam.reciente(0.5) for _ in range(20)]))
    for h in hilos:
        h.start()
    for h in hilos:
        h.join()
    assert len(aperturas) == 2 and cam.error == "" and fake.cruces == 0, (aperturas, cam.error, fake.cruces)
    n = fake.n
    for _ in range(5):
        cam.reciente(1.0)                                          # foto fresca: no saca otra
    assert fake.n == n
    cam.cerrar()
    assert not cam.abierta
    print(f"  {fake.n} capturas desde 4 hilos sin cruzarse; abierta 1 vez (tras un error al abrir); "
          "reciente() reutiliza la última")


def prueba_vista_real(cfg):
    import pygame as pg
    from cafebot.vision.camara import CamaraArchivo, CamaraCompartida
    from tools.vista_camara import VistaCamara
    foto = os.path.join(RAIZ, "tests", "datos", "arbol_taller_29oct.jpg")
    cam = CamaraCompartida(lambda: CamaraArchivo(foto))
    pg.init()
    win = pg.display.set_mode((1200, 850))
    fuente = pg.font.SysFont("monospace", 14)
    v = VistaCamara(cfg, "grande", camara=cam)
    try:
        lecturas = None
        fin = time.monotonic() + 5
        while time.monotonic() < fin and (v._cuadro is None or v._sup is None):
            v.dibujar(win, fuente, pose_foto(cfg), lecturas)
            time.sleep(0.05)
        assert v._sup is not None, v.error
        r = v.rect
        assert r.w == 600 and r.right <= 1200 and r.bottom <= 850, r
        pix = pg.surfarray.array3d(win)[r.x:r.right, r.y:r.bottom]
        assert pix.std() > 10, "la vista quedó vacía"
        v.ciclar()                                                 # grande -> oculta
        assert v.dibujar(win, fuente) is None and v.rect is None
        print(f"  cámara 'real' (foto guardada) en un recuadro de {r.w}x{r.h}; V la oculta")
    finally:
        v.detener()
        cam.cerrar()
        pg.quit()


def _captura(*args, captura_s=6.0):
    import pygame as pg
    with tempfile.TemporaryDirectory() as d:
        png = os.path.join(d, "c.png")
        r = subprocess.run([sys.executable, "-m", "tools.navegacion", "--sim", "--pantalla", "ventana",
                            "--captura", png, "--captura-s", str(captura_s), *args],
                           capture_output=True, text=True, timeout=120)
        assert r.returncode == 0, r.stderr[-2000:]
        return pg.surfarray.array3d(pg.image.load(png))           # (ancho, alto, 3) RGB


def prueba_navegacion():
    con = _captura()
    sin = _captura("--vista-camara", "no")
    W, H = con.shape[:2]
    w = int(W * 0.30)
    h = int(round(w * 864 / 1536))
    caja = (slice(W - w - 10, W - 10), slice(H - h - 10, H - 10))
    madera = np.array(render.MADERA[::-1])                        # RGB
    hay = np.abs(con[caja].astype(int) - madera).max(axis=2) < 25
    nada = np.abs(sin[caja].astype(int) - madera).max(axis=2) < 25
    print(f"  tools.navegacion --sim: la vista ({w}x{h}) muestra los árboles desde la partida "
          f"({hay.mean():.0%} de madera); con --vista-camara no, no hay vista")
    assert hay.mean() > 0.02 and nada.mean() < 0.001, (hay.mean(), nada.mean())


def main():
    cfg = cargar()
    print("1) Cámara simulada")
    prueba_render(cfg)
    print("2) Superposición de la cancha")
    prueba_superposicion(cfg)
    print("3) Inclinación de la cámara (servo X 2) en el driver del Mega")
    prueba_aux_mega()
    print("4) Cámara compartida (misión + vista)")
    prueba_compartida()
    print("5) La vista en la interfaz")
    prueba_vista_real(cfg)
    prueba_navegacion()
    print("OK: vista de la cámara")


if __name__ == "__main__":
    main()
