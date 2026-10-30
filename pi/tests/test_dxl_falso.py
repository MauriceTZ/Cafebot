"""Ejercita cafebot.arm.dxl contra un bus Dynamixel falso (sin hardware).
Correr desde pi/:  python3 -m tests.test_dxl_falso
"""
import sys
import types


class BusFalso:
    def __init__(self):
        self.mem = {}

    def _rw(self, n):
        def w(port, sid, addr, val):
            self.mem[(sid, addr)] = val
            return 0, 0

        def r(port, sid, addr):
            defaults = {146: 41, 144: 118, 126: 65436, 70: 0, 122: 0, 132: 2048, 6: 45}
            return self.mem.get((sid, addr), defaults.get(addr, 0)), 0, 0
        return w, r


bus = BusFalso()
sdk = types.ModuleType("dynamixel_sdk")


class PortHandler:
    def __init__(self, p): pass
    def openPort(self): return True
    def setBaudRate(self, b): return True
    def closePort(self): pass


class PacketHandler:
    def __init__(self, v):
        for n in (1, 2, 4):
            w, r = bus._rw(n)
            setattr(self, f"write{n}ByteTxRx", w)
            setattr(self, f"read{n}ByteTxRx", r)
        self.reboots = []
    def getTxRxResult(self, r): return str(r)
    def ping(self, port, sid): return 1060, 0, 0
    def reboot(self, port, sid): self.reboots.append(sid); bus.mem[(sid, 70)] = 0


sdk.PortHandler, sdk.PacketHandler = PortHandler, PacketHandler
sys.modules["dynamixel_sdk"] = sdk

from cafebot.arm import dxl  # noqa: E402

b = dxl.BusDXL("falso")
assert b.configurar()                       # la primera vez cambia EEPROM: apaga el torque
bus.mem[(8, dxl.ADDR_TORQUE_ENABLE)] = 1
assert b.configurar() == [] and bus.mem[(8, dxl.ADDR_TORQUE_ENABLE)] == 1   # después NO lo apaga (el brazo no se cae)
assert bus.mem[(4, dxl.ADDR_PWM_LIMIT)] == 300 and bus.mem[(8, dxl.ADDR_POS_I)] == 0
assert bus.mem[(8, dxl.ADDR_DRIVE_MODE)] == 0b100 and bus.mem.get((8, dxl.ADDR_RETURN_DELAY), 0) == 0
b.torque(True)
b.mover({8: 0.5, 11: -0.3}, 1.5)
assert bus.mem[(8, dxl.ADDR_PROFILE_VEL)] == 1500 and bus.mem[(8, dxl.ADDR_PROFILE_ACC)] == 450
s = b.salud()[0]
assert s.temp_c == 41 and abs(s.volt_v - 11.8) < 1e-6 and abs(s.carga_pct + 10.0) < 1e-6
assert b.puede_trabajar()[0]
bus.mem[(8, dxl.ADDR_HW_ERROR)] = 1 << 5
ok, motivo = b.puede_trabajar()
assert not ok and "sobrecarga" in motivo, motivo
assert b.recuperar_errores() == [8] and b.puede_trabajar()[0]
# Firmware viejo: perfil por velocidad (nunca Profile Velocity = duración en ms)
b.firmware = {k: 41 for k in b.firmware}
b.configurar()
assert not b.perfil_tiempo and bus.mem[(8, dxl.ADDR_DRIVE_MODE)] == 0
b.mover({8: 1.0}, 1.0)
assert 1 <= bus.mem[(8, dxl.ADDR_PROFILE_VEL)] <= 200, bus.mem[(8, dxl.ADDR_PROFILE_VEL)]
# Cero normalizado: el hombro encendido leyendo su cero + 4096 - 10 -> cero efectivo = cero + 4096
c8 = dxl.BRAZO[1].centro_pasos
bus.mem[(8, dxl.ADDR_PRESENT_POSITION)] = c8 + 4096 - 10
b.normalizar_centros()
assert b.centro[8] == c8 + 4096, b.centro[8]
assert abs(b.angulos([8])[8] - (-1) * (-10) * 2 * 3.141592653589793 / 4096) < 1e-9
print("OK: dxl contra bus falso")
