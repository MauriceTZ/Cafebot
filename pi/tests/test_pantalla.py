"""Ventanas de las herramientas: pantalla completa al lanzarlas por SSH en la pantalla de la Pi, F11 y Ctrl+C.
    python3 -m tests.test_pantalla        (sin pantalla: SDL_VIDEODRIVER=dummy)

  1. Cuándo se abre en pantalla completa (SSH + pantalla de la Pi) y cuándo en ventana (escritorio, ssh -X, PC).
  2. F11 alterna y vuelve al tamaño de la ventana; Ctrl+C llega como el evento de cerrar la ventana.
  3. tools.navegacion --sim de verdad: lanzado "por SSH" abre a pantalla completa; con --pantalla ventana no; y un
     Ctrl+C con la misión en marcha cierra ordenado (detiene la misión y sale con código 0).
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ["SDL_VIDEODRIVER"] = "dummy"

from tools import pantalla as P   # noqa: E402


def prueba_deteccion():
    ssh = {"SSH_CONNECTION": "192.168.1.20 50000 192.168.1.30 22"}
    casos = [({}, False),                                          # en el escritorio de la Pi o en el PC
             ({"DISPLAY": ":0"}, False),
             (ssh, True),                                         # SSH, sin DISPLAY (Wayland): pantalla de la Pi
             ({**ssh, "DISPLAY": ":0"}, True),                    # SSH con export DISPLAY=:0
             ({**ssh, "DISPLAY": ":0.0"}, True),
             ({**ssh, "DISPLAY": "localhost:10.0"}, False),       # ssh -X: la ventana sale en el PC
             ({"SSH_TTY": "/dev/pts/0", "DISPLAY": ":1"}, True)]
    for env, esperado in casos:
        assert P.por_ssh_en_la_pi(env) == esperado, (env, esperado)
    assert P.quiere_completa("completa", {}) and not P.quiere_completa("ventana", ssh)
    assert P.quiere_completa("auto", ssh) and not P.quiere_completa("auto", {})
    print("  SSH + pantalla de la Pi = completa; escritorio, VNC, ssh -X y el PC = ventana")


def prueba_ventana():
    import pygame as pg
    anterior = signal.getsignal(signal.SIGINT)
    try:
        pan = P.Pantalla("prueba", (640, 480), "ventana", redimensionable=True)
        assert pan.sup.get_size() == (640, 480) and not pan.completa
        pg.event.clear()
        pg.event.post(pg.event.Event(pg.KEYDOWN, key=pg.K_F11, mod=0, unicode="", scancode=0))
        assert any(pan.tecla(ev) for ev in pg.event.get()) and pan.completa
        completa = pan.sup.get_size()
        assert pg.display.get_surface().get_flags() & pg.FULLSCREEN
        pan.alternar()
        assert pan.sup.get_size() == (640, 480) and not pan.completa
        print(f"  F11: ventana 640x480 -> completa {completa[0]}x{completa[1]} -> ventana 640x480")
        pg.event.clear()
        os.kill(os.getpid(), signal.SIGINT)
        time.sleep(0.05)
        assert any(ev.type == pg.QUIT for ev in pg.event.get()), "Ctrl+C no llegó como QUIT"
        assert signal.getsignal(signal.SIGINT) is signal.default_int_handler    # el segundo ya corta
        print("  Ctrl+C = cerrar la ventana")
        pg.quit()
    finally:
        signal.signal(signal.SIGINT, anterior)


def _navegacion(env_extra, *args, captura_s=4.0):
    with tempfile.TemporaryDirectory() as d:
        png = os.path.join(d, "c.png")
        env = {k: v for k, v in os.environ.items() if not k.startswith("SSH_") and k != "DISPLAY"}
        env.update(env_extra)
        r = subprocess.run([sys.executable, "-m", "tools.navegacion", "--sim", "--captura", png,
                            "--captura-s", str(captura_s), *args], env=env, capture_output=True, text=True,
                           timeout=120)
        assert r.returncode == 0, r.stderr[-2000:]
        import pygame as pg
        return pg.image.load(png).get_size()


def prueba_navegacion():
    ssh = {"SSH_CONNECTION": "192.168.1.20 50000 192.168.1.30 22"}
    completa = _navegacion(ssh)
    ventana = _navegacion(ssh, "--pantalla", "ventana")
    escritorio = _navegacion({})
    print(f"  tools.navegacion por SSH: {completa[0]}x{completa[1]} (completa); con --pantalla ventana y desde el "
          f"escritorio: {ventana[0]}x{ventana[1]}")
    assert ventana == escritorio == (1200, 850) and completa != (1200, 850)

    env = {k: v for k, v in os.environ.items() if not k.startswith("SSH_")}
    env.update(ssh)
    p = subprocess.Popen([sys.executable, "-m", "tools.navegacion", "--sim", "--mision"], env=env,
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        time.sleep(12)                          # la misión parte a los 3 s y ya va camino al primer árbol
        assert p.poll() is None, p.stdout.read()[-2000:]
        t0 = time.monotonic()
        os.killpg(p.pid, signal.SIGINT)        # como la terminal: a todo el grupo de procesos
        salida, _ = p.communicate(timeout=20)
    finally:
        if p.poll() is None:
            p.kill()
    dt = time.monotonic() - t0
    assert p.returncode == 0, salida[-2000:]
    assert "Ctrl+C: cerrando" in salida and "Traceback" not in salida and "Exception" not in salida, salida[-2000:]
    print(f"  Ctrl+C con la misión en marcha: cerró ordenado en {dt:.1f} s (código 0)")


def main():
    print("1) Cuándo pantalla completa")
    prueba_deteccion()
    print("2) F11 y Ctrl+C")
    prueba_ventana()
    print("3) tools.navegacion --sim")
    prueba_navegacion()
    print("OK: pantalla")


if __name__ == "__main__":
    main()
