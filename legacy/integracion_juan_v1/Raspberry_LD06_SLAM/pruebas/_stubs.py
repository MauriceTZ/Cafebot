"""Sustitutos de pygame / pyserial SOLO para correr las pruebas en un PC sin esas librerías.
En la Raspberry (donde sí están instaladas) no hace nada."""
import sys
from unittest.mock import MagicMock

for nombre in ("pygame", "pygame.gfxdraw", "serial"):
    try:
        __import__(nombre)
    except ImportError:
        sys.modules[nombre] = MagicMock()
if isinstance(sys.modules.get("serial"), MagicMock):
    class _SerialException(Exception):
        pass
    sys.modules["serial"].SerialException = _SerialException

import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
