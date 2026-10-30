"""Bus Dynamixel FALSO para probar el brazo sin hardware: lo que se manda como meta pasa a ser la
posición actual (servos ideales). `bus.grano = True` simula un grano entre los dedos: la garra
se detiene en ~40° en vez de cerrar del todo. Cada servo tiene su id y su velocidad (como los reales):
solo responde si el puerto está a esa velocidad; se pueden cambiar (EEPROM: id 7, baud 8).
Instalar ANTES de importar cafebot.arm."""
import sys
import types

ADDR_ID, ADDR_BAUD, ADDR_GOAL, ADDR_PRESENT = 7, 8, 116, 132
SIN_RESPUESTA = -3001          # COMM_RX_TIMEOUT del SDK
BAUD = {0: 9_600, 1: 57_600, 2: 115_200, 3: 1_000_000, 4: 2_000_000, 5: 3_000_000, 6: 4_000_000, 7: 4_500_000}
POSICION_TRABAJO = {9: 3071, 8: 2403, 11: 2492, 7: 180, 5: 1587, 4: 336}   # "trabajo" (cafebot/arm/brazo.py)


class Memoria:
    def __init__(self):
        self.mem = {}
        self.grano = False
        self.reboots = []
        self.escrituras = []
        self.baud_de = {}                      # id -> baudios a los que responde
        self.centro_garra = 336
        for sid, pos in POSICION_TRABAJO.items():
            self.agregar(sid, pos)             # encendido en la postura de "trabajo" del código antiguo

    def agregar(self, sid, pos=2048, baudios=1_000_000):
        self.mem[(sid, ADDR_PRESENT)] = pos
        self.baud_de[sid] = baudios

    def responde(self, sid, baudios) -> bool:
        return self.baud_de.get(sid) == baudios

    def escribir(self, sid, addr, val):
        self.mem[(sid, addr)] = val
        self.escrituras.append((sid, addr, val))
        if addr == ADDR_ID and val != sid:            # el servo pasa a llamarse `val`
            for (s, a), v in list(self.mem.items()):
                if s == sid:
                    self.mem[(val, a)] = v
                    del self.mem[(s, a)]
            self.baud_de[val] = self.baud_de.pop(sid)
        elif addr == ADDR_BAUD:
            self.baud_de[sid] = BAUD[val]
        elif addr == ADDR_GOAL:
            if val >= 1 << 31:
                val -= 1 << 32
            if sid == 4 and self.grano:
                val = min(val, self.centro_garra + int(40 / 360 * 4096))     # el grano no deja cerrar
            self.mem[(sid, ADDR_PRESENT)] = val & 0xFFFFFFFF

    def leer(self, sid, addr):
        defaults = {146: 38, 144: 118, 126: 50, 70: 0, 122: 0, 6: 45, 0: 1060}
        return self.mem.get((sid, addr), defaults.get(addr, 0))


bus = Memoria()


def instalar():
    sdk = types.ModuleType("dynamixel_sdk")

    class PortHandler:
        def __init__(self, p):
            self.baudios = None
        def openPort(self): return True
        def setBaudRate(self, b):
            self.baudios = b
            return True
        def closePort(self): pass

    class PacketHandler:
        def __init__(self, v):
            def escribir(port, sid, addr, val):
                if not bus.responde(sid, port.baudios):
                    return SIN_RESPUESTA, 0
                bus.escribir(sid, addr, val)
                return 0, 0

            def leer(port, sid, addr):
                if not bus.responde(sid, port.baudios):
                    return 0, SIN_RESPUESTA, 0
                return bus.leer(sid, addr), 0, 0
            for n in (1, 2, 4):
                setattr(self, f"write{n}ByteTxRx", escribir)
                setattr(self, f"read{n}ByteTxRx", leer)
        def getTxRxResult(self, r): return "sin respuesta" if r == SIN_RESPUESTA else str(r)
        def ping(self, port, sid):
            return (bus.leer(sid, 0), 0, 0) if bus.responde(sid, port.baudios) else (0, SIN_RESPUESTA, 0)
        def broadcastPing(self, port):
            datos = {sid: [bus.leer(sid, 0), bus.leer(sid, 6)] for sid in bus.baud_de if bus.responde(sid, port.baudios)}
            return datos, (0 if datos else SIN_RESPUESTA)
        def reboot(self, port, sid):
            bus.reboots.append(sid)
            bus.mem[(sid, 70)] = 0

    sdk.PortHandler, sdk.PacketHandler = PortHandler, PacketHandler
    sys.modules["dynamixel_sdk"] = sdk
    return bus
