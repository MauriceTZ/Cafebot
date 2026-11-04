"""Ventana pygame de las herramientas (navegación, cosecha, visor del LiDAR), con pantalla completa.

--pantalla auto (por defecto): si el programa se lanzó por SSH y se dibuja en la pantalla de la Raspberry (la del
HDMI; no reenviada al PC con ssh -X), la ventana se abre en PANTALLA COMPLETA, para no tener que maximizarla con el
mouse de la Pi. En el escritorio, por VNC o en el notebook: ventana normal.
--pantalla completa / --pantalla ventana: a la fuerza.

    F11      pantalla completa sí/no
    Ctrl+C   en la terminal (p. ej. la de SSH) = cerrar la ventana: el programa sale por su camino normal (detiene
             la misión y el robot). Un segundo Ctrl+C corta de inmediato.
"""
from __future__ import annotations

import os
import signal

OPCIONES = ("auto", "completa", "ventana")


def agregar_opcion(ap) -> None:
    ap.add_argument("--pantalla", choices=OPCIONES, default="auto",
                    help="auto (por defecto): pantalla completa si se lanzó por SSH y se ve en la pantalla de la "
                         "Raspberry; completa; ventana. F11 la alterna")


def por_ssh_en_la_pi(env=None) -> bool:
    """¿Lanzado por SSH pero dibujando en la pantalla de la propia Pi? DISPLAY vacío (Wayland) o ':0' = la pantalla
    local; 'localhost:10.0' = reenviada con ssh -X (la ventana sale en el PC: ahí va una ventana normal)."""
    env = os.environ if env is None else env
    if not any(env.get(k) for k in ("SSH_CONNECTION", "SSH_CLIENT", "SSH_TTY")):
        return False
    return env.get("DISPLAY", "").split(":")[0] in ("", "unix")


def quiere_completa(opcion: str = "auto", env=None) -> bool:
    if opcion == "completa":
        return True
    if opcion == "ventana":
        return False
    return por_ssh_en_la_pi(env)


class Pantalla:
    """La ventana de una herramienta. `sup` es la superficie donde se dibuja: tras F11 cambia (y su tamaño)."""

    def __init__(self, titulo: str, tam, opcion: str = "auto", redimensionable: bool = False):
        import pygame as pg             # aquí y no arriba: --pantalla sirve sin pygame (tools.cosecha --texto)
        self.pg = pg
        self.titulo = titulo
        self.tam = (int(tam[0]), int(tam[1]))       # tamaño de la ventana normal
        self.flags = pg.RESIZABLE if redimensionable else 0
        self.completa = quiere_completa(opcion)
        # Que no se minimice si el foco pasa a otra ventana (p. ej. un clic en el escritorio de la Pi)
        os.environ.setdefault("SDL_VIDEO_MINIMIZE_ON_FOCUS_LOSS", "0")
        pg.init()
        self.sup = self._abrir()
        _ctrl_c_cierra(pg)

    def _abrir(self):
        pg = self.pg
        if self.completa:
            sup = pg.display.set_mode((0, 0), pg.FULLSCREEN)     # (0, 0) = la resolución del escritorio
        else:
            sup = pg.display.set_mode(self.tam, self.flags)
        pg.display.set_caption(self.titulo)
        return sup

    def alternar(self):
        if not self.completa and self.flags & self.pg.RESIZABLE:
            self.tam = self.sup.get_size()            # al volver, el tamaño que tenía la ventana
        self.completa = not self.completa
        self.sup = self._abrir()
        return self.sup

    def tecla(self, ev) -> bool:
        """Atiende F11 (pantalla completa sí/no). True si el evento era ese: `sup` cambió."""
        if ev.type == self.pg.KEYDOWN and ev.key == self.pg.K_F11:
            self.alternar()
            return True
        return False


def _ctrl_c_cierra(pg):
    """Ctrl+C = el evento de cerrar la ventana, para que el programa salga por su camino normal (sin eso, la
    excepción salta en cualquier línea del bucle y no se detienen la misión ni el robot)."""
    def manejador(signum, frame):
        signal.signal(signal.SIGINT, signal.default_int_handler)     # el segundo Ctrl+C corta de inmediato
        print("\nCtrl+C: cerrando (otro Ctrl+C para cortar de inmediato)...")
        pg.event.post(pg.event.Event(pg.QUIT))
    try:
        signal.signal(signal.SIGINT, manejador)
    except ValueError:          # solo se puede desde el hilo principal
        pass
