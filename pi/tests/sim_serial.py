"""Puerto serial falso conectado al firmware REAL del Mega compilado para el PC
(firmware/test_host). Permite probar el código de la Pi sin el robot.

    from tests.sim_serial import SerialSimulado
    mega = Mega("sim"); mega.conectar(serial_obj=SerialSimulado())
"""
import os
import pathlib
import subprocess
import threading

RAIZ = pathlib.Path(__file__).resolve().parents[2]
BINARIO = "/tmp/cafebot_sim"


def compilar() -> str:
    th = RAIZ / "firmware" / "test_host"
    subprocess.run(["g++", "-std=c++17", "-O1", f"-I{th}", "-include", "Arduino.h", "-x", "c++",
                    str(th / "sim.cpp"), "-o", BINARIO], check=True)
    return BINARIO


class SerialSimulado:
    def __init__(self):
        if not os.path.exists(BINARIO):
            compilar()
        self.p = subprocess.Popen([BINARIO, "--interactivo"], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        self._buf = bytearray()
        self._cv = threading.Condition()
        threading.Thread(target=self._lector, daemon=True).start()

    def _lector(self):
        while True:
            b = self.p.stdout.read1(256)
            if not b:
                return
            with self._cv:
                self._buf += b
                self._cv.notify_all()

    def read(self, n=1):
        with self._cv:
            self._cv.wait_for(lambda: len(self._buf) > 0, 0.05)
            out = bytes(self._buf[:n])
            del self._buf[:n]
            return out

    def write(self, data: bytes):
        self.p.stdin.write(data)
        self.p.stdin.flush()

    def close(self):
        self.p.kill()
