"""Bus Dynamixel FALSO para probar el brazo sin hardware: lo que se manda como meta pasa a ser la
posición actual (servos ideales). `bus.grano = True` simula un grano entre los dedos: la garra
se detiene en ~40° en vez de cerrar del todo. Instalar ANTES de importar cafebot.arm."""
import sys
import types

ADDR_GOAL, ADDR_PRESENT = 116, 132


class Memoria:
    def __init__(self):
        self.mem = {}
        self.grano = False
        self.reboots = []
        self.escrituras = []
        for sid, pos in {9: 3074, 8: 1400, 11: 2423, 7: 194, 5: 1587, 4: 336}.items():
            self.mem[(sid, ADDR_PRESENT)] = pos       # encendido en la postura de "trabajo" del código antiguo

    def escribir(self, sid, addr, val):
        self.mem[(sid, addr)] = val
        self.escrituras.append((sid, addr, val))
        if addr == ADDR_GOAL:
            if val >= 1 << 31:
                val -= 1 << 32
            if sid == 4 and self.grano:
                val = min(val, 336 + int(40 / 360 * 4096))     # el grano no deja cerrar
            self.mem[(sid, ADDR_PRESENT)] = val & 0xFFFFFFFF

    def leer(self, sid, addr):
        defaults = {146: 38, 144: 118, 126: 50, 70: 0, 122: 0, 6: 45}
        return self.mem.get((sid, addr), defaults.get(addr, 0))


bus = Memoria()


def instalar():
    sdk = types.ModuleType("dynamixel_sdk")

    class PortHandler:
        def __init__(self, p): pass
        def openPort(self): return True
        def setBaudRate(self, b): return True
        def closePort(self): pass

    class PacketHandler:
        def __init__(self, v):
            def escribir(port, sid, addr, val):
                bus.escribir(sid, addr, val)
                return 0, 0

            def leer(port, sid, addr):
                return bus.leer(sid, addr), 0, 0
            for n in (1, 2, 4):
                setattr(self, f"write{n}ByteTxRx", escribir)
                setattr(self, f"read{n}ByteTxRx", leer)
        def getTxRxResult(self, r): return str(r)
        def ping(self, port, sid): return 1060, 0, 0
        def reboot(self, port, sid):
            bus.reboots.append(sid)
            bus.mem[(sid, 70)] = 0

    sdk.PortHandler, sdk.PacketHandler = PortHandler, PacketHandler
    sys.modules["dynamixel_sdk"] = sdk
    return bus
