"""Adaptador no bloqueante del brazo Dynamixel para MisionCosecha.

IMPORTANTE: requiere calibrar la relación cámara->base del brazo y probar primero sin fruto.
El puerto del brazo debe ser distinto al puerto USB del LD06 y al del Arduino Mega.

Cambios respecto de la primera integración (ver CAMBIOS_REVISION.md):
  * Los movimientos son INTERRUMPIBLES: [ESC] / cancelar() detiene el brazo en el siguiente paso de
    20 ms (antes terminaba el movimiento en curso, hasta 2 s más, avanzando hacia el fruto).
  * Al iniciar se lleva el brazo a la postura de trabajo (OFFSETS_PRENDERSE) como hace
    main_stress_test.py; la cinemática inversa (pitch_ref / factor_codo2) se calcula SIEMPRE desde
    esa postura y no desde la postura en que estuviera el brazo al encender.
  * cerrar() estaciona el brazo en la postura de reposo (OFFSETS_APAGARSE) ANTES de soltar el torque
    (antes soltaba el torque de golpe y el brazo caía por gravedad).
  * Se verifican los errores de comunicación Dynamixel y se valida la cinemática inversa.
  * El motivo de cada rechazo / fallo queda disponible en ultimo_error() y se muestra en el HUD.
"""
from __future__ import annotations
import math, os, sys, threading, time, traceback

HERE = os.path.dirname(os.path.abspath(__file__))
ARM_DIR = os.path.join(HERE, "brazo1_description")
if ARM_DIR not in sys.path:
    sys.path.insert(0, ARM_DIR)

from brazo_interfaz import BrazoBase


class _Cancelado(Exception):
    """El operador / la misión canceló el movimiento en curso."""


class BrazoDynamixel(BrazoBase):
    nombre = "Dynamixel + MuJoCo"

    def __init__(self, puerto="/dev/ttyUSB1", baud=1_000_000, mujoco=True,
                 max_dist_m=0.40, radio_obj_mm=140.0, altura_obj_mm=200.0,
                 gripper_cerrado_deg=65.0, yaw_camara_signo=1.0, offset_yaw_deg=0.0,
                 ir_a_postura_trabajo=True, max_corr_yaw_deg=45.0):
        self.puerto, self.baud = puerto, int(baud)
        self.usar_mujoco = bool(mujoco)
        self.max_dist_m = float(max_dist_m)
        self.radio_obj_m = float(radio_obj_mm) / 1000.0
        self.altura_obj_m = float(altura_obj_mm) / 1000.0
        self.grip_cerrado = math.radians(float(gripper_cerrado_deg))
        self.yaw_sign = float(yaw_camara_signo)
        self.yaw_offset = math.radians(float(offset_yaw_deg))
        self.ir_a_postura_trabajo = bool(ir_a_postura_trabajo)
        self.max_corr_yaw = math.radians(float(max_corr_yaw_deg))
        self._lock = threading.Lock()
        self._result = "LIBRE"
        self._thread = None
        self._cancel = threading.Event()
        self._error = ""
        self._ready = False
        self._cerrado = False
        self.port = None
        self.viewer = None
        self._init_hardware()

    # ------------------------------------------------------------------ inicio
    def _chk(self, comm, err, que):
        """Lanza RuntimeError si una escritura/lectura Dynamixel falló."""
        if comm != self.COMM_SUCCESS:
            raise RuntimeError(f"{que}: {self.packet.getTxRxResult(comm)}")

    def _init_hardware(self):
        try:
            import numpy as np
            import mujoco, mujoco.viewer
            from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS, GroupSyncWrite
            import main_stress_test as arm
            self.arm, self.np, self.mujoco = arm, np, mujoco
            self.COMM_SUCCESS = COMM_SUCCESS
            self.config_giro = {9: False, 8: True, 11: True, 7: True, 5: True, 4: False}
            self.port = PortHandler(self.puerto)
            self.packet = PacketHandler(2.0)

            # 1) MuJoCo primero: si el modelo falla, el brazo físico todavía no se ha tocado.
            self.model = self.data = None
            self.indices = []
            if self.usar_mujoco:
                urdf = os.path.join(ARM_DIR, "brazo1_description", "urdf", "brazo1.urdf")
                if not os.path.exists(urdf):
                    # Fallback para el diseño anidado original
                    urdf = os.path.join(ARM_DIR, "brazo1_description", "brazo1_description", "urdf", "brazo1.urdf")
                self.model = mujoco.MjModel.from_xml_path(urdf)
                self.data = mujoco.MjData(self.model)
                names = ["hombro_yaw_joint", "hombro_pitch_joint", "codo1_pitch_joint",
                         "codo2_pitch_joint", "muneca_pitch_joint"]
                for n in names:
                    jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
                    if jid < 0:
                        raise RuntimeError(f"Articulación {n} no encontrada en el modelo MuJoCo")
                    self.indices.append(self.model.jnt_qposadr[jid])
                self.viewer = mujoco.viewer.launch_passive(self.model, self.data)

            # 2) Puerto y servos
            if not self.port.openPort():
                raise RuntimeError(f"No se pudo abrir el puerto Dynamixel {self.puerto}")
            if not self.port.setBaudRate(self.baud):
                raise RuntimeError(f"No se pudo configurar {self.baud} baudios en {self.puerto}")
            self.centros = {}
            self.angulos = [0.0] * len(arm.SERVO_IDS_BRAZO)
            self.grip = 0.0
            P, A = self.port, arm
            for sid in arm.TODOS_LOS_SERVOS:
                raw, comm, err = self.packet.read4ByteTxRx(P, sid, arm.ADDR_PRESENT_POSITION)
                self._chk(comm, err, f"No responde Dynamixel ID {sid}")
                pos = arm.dxl_to_signed(raw)
                self.centros[sid] = arm.normalizar_offset_encendido(pos, arm.OFFSETS_ORTOGONAL[sid])
                w1 = lambda addr, v: self._chk(*self.packet.write1ByteTxRx(P, sid, addr, v), f"ID {sid} dir {addr}")
                w2 = lambda addr, v: self._chk(*self.packet.write2ByteTxRx(P, sid, addr, v), f"ID {sid} dir {addr}")
                w4 = lambda addr, v: self._chk(*self.packet.write4ByteTxRx(P, sid, addr, v), f"ID {sid} dir {addr}")
                w1(A.ADDR_TORQUE_ENABLE, 0)
                w1(A.ADDR_OPERATING_MODE, 4)
                if sid == arm.SERVO_ID_GRIPPER:
                    w2(A.ADDR_POSITION_P_GAIN, 600)
                    w2(A.ADDR_POSITION_I_GAIN, 0)
                    w2(A.ADDR_POSITION_D_GAIN, 1000)
                    w2(A.ADDR_GOAL_PWM, arm.PWM_MAX_GRIPPER)
                    w4(A.ADDR_PROFILE_ACCEL, 10)
                    w4(A.ADDR_PROFILE_VEL, 35)
                    self.grip = arm.dynamixel_a_rad(pos, self.centros[sid], self.config_giro[sid])
                else:
                    w2(A.ADDR_POSITION_P_GAIN, 1000)
                    w2(A.ADDR_POSITION_I_GAIN, 150)
                    w2(A.ADDR_POSITION_D_GAIN, 1200)
                    w4(A.ADDR_PROFILE_ACCEL, 40)
                    w4(A.ADDR_PROFILE_VEL, 150)
                    self.angulos[arm.SERVO_IDS_BRAZO.index(sid)] = arm.dynamixel_a_rad(
                        pos, self.centros[sid], self.config_giro[sid])
                w4(A.ADDR_GOAL_POSITION, pos & 0xFFFFFFFF)     # mantener donde está
                w1(A.ADDR_TORQUE_ENABLE, 1)
            self.sync = GroupSyncWrite(self.port, self.packet, arm.ADDR_GOAL_POSITION, 4)

            # 3) Postura de trabajo y de reposo (tablas del proyecto del brazo)
            self.reposo = arm.obtener_angulos_desde_tabla(arm.OFFSETS_APAGARSE, self.config_giro)
            if self.ir_a_postura_trabajo:
                self.home = arm.obtener_angulos_desde_tabla(arm.OFFSETS_PRENDERSE, self.config_giro)
                self._ready = True            # _move() lo exige; se revierte si algo falla
                self._move(self.home, math.radians(arm.GRIPPER_ABIERTO_DEG), 2.5)
            else:
                self.home = list(self.angulos)
            # La cinemática inversa se referencia SIEMPRE a la postura de trabajo.
            _, _, self._yaw_home, self._pitch_ref, self._factor_c2 = arm.cinematica_directa_brazo(self.home)
            self._ready = True
        except Exception as e:
            self._error = str(e)
            self._ready = False
            self._liberar(torque_off=False)
            raise RuntimeError(f"No se pudo inicializar el brazo: {e}") from e

    # ------------------------------------------------------------------ movimiento
    def _escribir(self, ang, grip):
        """Un paso: actualiza MuJoCo y manda las metas a los servos (réplica de mover_suave_a_postura)."""
        arm = self.arm
        v = self.viewer
        if v is not None and v.is_running():
            for idx, qpos_addr in enumerate(self.indices):
                self.data.qpos[qpos_addr] = ang[idx]
            self.mujoco.mj_forward(self.model, self.data)
            v.sync()
        self.sync.clearParam()
        for i, sid in enumerate(arm.SERVO_IDS_BRAZO):
            pasos = arm.rad_a_dynamixel(ang[i], self.centros[sid], self.config_giro[sid])
            self.sync.addParam(sid, list((pasos & 0xFFFFFFFF).to_bytes(4, byteorder="little")))
        sid = arm.SERVO_ID_GRIPPER
        pasos = arm.rad_a_dynamixel(grip, self.centros[sid], self.config_giro[sid])
        self.sync.addParam(sid, list((pasos & 0xFFFFFFFF).to_bytes(4, byteorder="little")))
        return self.sync.txPacket()

    def _move(self, angles, grip, duration):
        """Interpolación cosenoidal a 50 Hz como en el proyecto del brazo, pero revisando la
        cancelación en cada paso. Si se cancela, el brazo se queda donde está (sin terminar el movimiento)."""
        if not self._ready:
            raise RuntimeError("brazo no inicializado")
        ini, grip_ini = list(self.angulos), self.grip
        pasos = max(1, int(duration * 50))
        dt = duration / pasos
        fallos = 0
        for k in range(1, pasos + 1):
            if self._cancel.is_set():
                raise _Cancelado()
            t0 = time.monotonic()
            s = 0.5 * (1.0 - math.cos(math.pi * k / pasos))
            ang = [ini[i] + s * (angles[i] - ini[i]) for i in range(5)]
            g = grip_ini + s * (grip - grip_ini)
            comm = self._escribir(ang, g)
            self.angulos, self.grip = ang, g
            if comm != self.COMM_SUCCESS:
                fallos += 1
                if fallos >= 5:
                    raise RuntimeError(f"fallo de comunicación Dynamixel: {self.packet.getTxRxResult(comm)}")
            else:
                fallos = 0
            resto = dt - (time.monotonic() - t0)
            if resto > 0:
                time.sleep(resto)

    def _validar(self, q, nombre):
        """La cinemática inversa clampa en silencio; aquí se rechazan resultados no finitos o fuera de ±180°."""
        if any((not math.isfinite(a)) or abs(a) > math.pi for a in q):
            raise RuntimeError(f"cinemática inversa inválida en '{nombre}': {[round(math.degrees(a), 1) for a in q]}")
        return q

    # ------------------------------------------------------------------ API BrazoBase
    def disponible(self):
        return (self._ready and not self._cerrado and (self._thread is None or not self._thread.is_alive())
                and self._result in ("LIBRE", "OK", "FALLO"))

    def iniciar_recogida(self, grano):
        if not self.disponible():
            self._error = "brazo ocupado o no disponible"
            return False
        # No accionar el brazo si la distancia calibrada indica que el fruto está lejos.
        if grano.dist_m is None:
            self._error = ("distancia sin calibrar (falta focal_px): ejecuta "
                           "'python vision/detector.py --calibrar-dist 0.32' con un grano a 32 cm")
            return False
        if grano.dist_m > self.max_dist_m:
            self._error = f"fruto fuera de alcance: {grano.dist_m:.2f} m > {self.max_dist_m:.2f} m"
            return False
        corr = self.yaw_sign * math.atan2(grano.dx_mm / 1000.0, self.radio_obj_m)
        if abs(corr) > self.max_corr_yaw:
            self._error = (f"fruto demasiado lateral: corrección de yaw {math.degrees(corr):+.0f}° "
                           f"(máx ±{math.degrees(self.max_corr_yaw):.0f}°)")
            return False
        self._error = ""
        self._cancel.clear()
        with self._lock:
            self._result = "OCUPADO"
        self._thread = threading.Thread(target=self._pick_worker, args=(grano, corr), daemon=True)
        self._thread.start()
        return True

    def _pick_worker(self, grano, corr_yaw):
        try:
            arm = self.arm
            abierto = math.radians(arm.GRIPPER_ABIERTO_DEG)
            pitch_ref, f2 = self._pitch_ref, self._factor_c2
            r, z = self.radio_obj_m, self.altura_obj_m
            yaw = self._yaw_home + self.yaw_offset + corr_yaw
            alcance = arm.L1 + arm.L2 + arm.L3 + arm.L4 - 0.005
            ik = arm.cinematica_inversa_analitica
            target = self._validar(ik(r, z, yaw, pitch_ref, f2), "agarre")
            approach = self._validar(ik(min(r + 0.025, alcance), z + 0.025, yaw, pitch_ref, f2), "aproximación")
            # Depósito provisional según categoría: yaw girado +/-90 grados.
            side = math.radians(90.0 if grano.categoria == "MADURO" else -90.0)
            drop = self._validar(ik(r, z + 0.04, self._yaw_home + side, pitch_ref, f2), "depósito")
            # Aproximación, bajar, cerrar garra, elevar, depositar y volver a la postura de trabajo.
            self._move(approach, abierto, 2.0)
            self._move(target, abierto, 1.5)
            self._move(target, self.grip_cerrado, 0.8)
            self._move(approach, self.grip_cerrado, 1.5)
            self._move(drop, self.grip_cerrado, 1.8)
            self._move(drop, abierto, 0.8)
            self._move(self.home, abierto, 2.0)
            with self._lock:
                self._result = "OK"
        except _Cancelado:
            self._error = "recogida cancelada (brazo detenido en su posición actual)"
            with self._lock:
                self._result = "FALLO"
        except Exception as e:
            self._error = str(e)
            print("[brazo] error en la recogida:\n" + traceback.format_exc(), file=sys.stderr)
            with self._lock:
                self._result = "FALLO"

    def estado(self):
        return self._result

    def ultimo_error(self):
        return self._error

    def reconocer(self):
        with self._lock:
            if self._result in ("OK", "FALLO"):
                self._result = "LIBRE"

    def cancelar(self):
        self._cancel.set()
        # El worker revisa la cancelación en cada paso de 20 ms; se espera a que termine.
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=0.5)
        with self._lock:
            if self._result == "OCUPADO" and (t is None or not t.is_alive()):
                self._result = "FALLO"

    def deposito_lleno(self, categoria):
        # No hay sensores de capacidad instalados todavía; el conteo lo lleva MisionCosecha.
        return False

    def resumen(self):
        txt = f"{self.nombre} ({'listo' if self._ready else 'no disponible'}, {self._result})"
        if self._error:
            txt += f" | {self._error.splitlines()[0][:70]}"
        return txt

    # ------------------------------------------------------------------ cierre
    def _liberar(self, torque_off=True):
        try:
            if self.viewer is not None:
                self.viewer.close()
        except Exception:
            pass
        try:
            if self.port is not None and getattr(self.port, "is_open", False):
                if torque_off and getattr(self, "centros", None):
                    for sid in self.arm.TODOS_LOS_SERVOS:
                        self.packet.write1ByteTxRx(self.port, sid, self.arm.ADDR_TORQUE_ENABLE, 0)
                self.port.closePort()
        except Exception:
            pass

    def cerrar(self):
        """Idempotente. Estaciona el brazo en la postura de reposo ANTES de soltar el torque."""
        if self._cerrado:
            return
        self._cerrado = True
        self._cancel.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        if self._ready:
            try:
                self._cancel.clear()
                self._move(self.reposo, self.grip, 3.0)
                time.sleep(0.3)
            except Exception:
                pass
        self._ready = False
        self._liberar(torque_off=True)
