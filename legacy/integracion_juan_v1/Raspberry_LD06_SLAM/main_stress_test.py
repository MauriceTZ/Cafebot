import time
import math
import random
import sys
import select
import termios
import tty
import numpy as np
import mujoco
import mujoco.viewer
from dynamixel_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS, 
    GroupSyncWrite, GroupSyncRead
)

# ==========================================
# 1. TABLAS DE CALIBRACIÓN Y POSTURAS FIJAS (EN PASOS DE ENCODER)
# ==========================================
OFFSETS_ORTOGONAL = {
    9: 3088,
    8: 1042,
    11: 3074,
    7: 1546,
    5: 1014,
    4: 336
}

OFFSETS_PRENDERSE = {
    9: 3074,
    8: 1400,
    11: 2423,
    7: 194,
    5: 1587,
    4: 336
}

OFFSETS_APAGARSE = {
    9: 3085,
    8: 1787,
    11: 1719,
    7: 78,
    5: 1801,
    4: 336
}

# ==========================================
# 2. MAPA DE MEMORIA DYNAMIXEL (XL430-W250-T)
# ==========================================
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCEL = 108
ADDR_PROFILE_VEL = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

SERVO_IDS_BRAZO = [9, 8, 11, 7, 5]  # Base Yaw, Hombro, Codo1, Codo2, Muñeca
SERVO_ID_GRIPPER = 4                # Gripper
TODOS_LOS_SERVOS = SERVO_IDS_BRAZO + [SERVO_ID_GRIPPER]

PUERTO = "/dev/ttyUSB0"
BAUD_RATE = 1000000

# ==========================================
# 3. CONSTANTES DE ANIMACIÓN, CONTROL Y GRAVEDAD
# ==========================================
# Parámetros del ciclo automático Pick & Place
ALTURAS_OBJETO_MM = [130.0, 200.0, 250.0]  # 3 alturas posibles en mm (ajustables)
RADIO_AGARRE_MM = 140.0                   # Distancia radial donde busca el objeto (mm)
TIEMPO_MOVIMIENTO_S = 2.0                 # Duración de cada desplazamiento (s)
TIEMPO_PAUSA_ACCION_S = 0.6               # Pausa simulando "agarrar" o "soltar" (s)

# Incrementos para modo manual
PASO_Z_MM = 5.0
PASO_RADIO_MM = 5.0
PASO_YAW_DEG = 5.0

SIGNO_CODO = 1

# Dimensiones del URDF (en metros)
L1 = 0.107       # Hombro a Codo 1
L2 = 0.107       # Codo 1 a Codo 2
L3 = 0.067       # Codo 2 a Muñeca
L4 = 0.032       # Muñeca a Garra
Z_BASE = 0.0671  # Base a Hombro

# Configuración del Gripper (Servo 4 fijo en abierto durante la animación)
GRIPPER_ABIERTO_DEG = 0.0
PWM_MAX_GRIPPER = 350

# Ganancias de Lazo Cerrado y Compensación de Gravedad
K_GRAV = 0.08
K_I_EXT = 0.15
MAX_CORRECCION_RAD = math.radians(8.0)

# ==========================================
# 4. FUNCIONES DE CONVERSIÓN Y CINEMÁTICA
# ==========================================
def rad_a_dynamixel(angulo_rad, centro_fisico, invertir=False):
    direccion = -1 if invertir else 1
    pasos = int(centro_fisico + (direccion * angulo_rad * 4096 / (2 * math.pi)))
    return max(-1048575, min(1048575, pasos))

def dynamixel_a_rad(pasos_actuales, centro_fisico, invertir=False):
    direccion = -1 if invertir else 1
    return direccion * (pasos_actuales - centro_fisico) * (2 * math.pi) / 4096.0

def dxl_to_signed(val):
    if val > 0x7fffffff:
        val -= 4294967296
    return val

def normalizar_offset_encendido(pos_actual, offset_referencia):
    diff = pos_actual - offset_referencia
    diff_envuelto = ((diff + 2048) % 4096) - 2048
    return pos_actual - diff_envuelto

def obtener_angulos_desde_tabla(tabla_pasos, config_giro):
    angulos = []
    for s_id in SERVO_IDS_BRAZO:
        diff = ((tabla_pasos[s_id] - OFFSETS_ORTOGONAL[s_id] + 2048) % 4096) - 2048
        pasos_rel = OFFSETS_ORTOGONAL[s_id] + diff
        angulos.append(dynamixel_a_rad(pasos_rel, OFFSETS_ORTOGONAL[s_id], config_giro[s_id]))
    return angulos

def cinematica_directa_brazo(angulos):
    q_yaw, q_hombro, q_codo1, q_codo2, q_muneca = angulos
    t1 = q_hombro
    t12 = q_hombro + q_codo1
    t123 = q_hombro + q_codo1 + q_codo2
    pitch_global = t123 + q_muneca

    rw = L1 * math.sin(t1) + L2 * math.sin(t12) + L3 * math.sin(t123)
    zw = Z_BASE + L1 * math.cos(t1) + L2 * math.cos(t12) + L3 * math.cos(t123)

    r_garra = rw + L4 * math.sin(pitch_global)
    z_garra = zw + L4 * math.cos(pitch_global)

    delta_z = zw - Z_BASE
    d2 = rw**2 + delta_z**2
    l23_recto = L2 + L3
    c_total = (d2 - L1**2 - l23_recto**2) / (2 * L1 * l23_recto)
    c_total = max(-0.9999, min(0.9999, c_total))
    ang_flex_total = math.acos(c_total)
    factor_c2 = abs(q_codo2) / ang_flex_total if ang_flex_total > 1e-4 else 0.5

    return r_garra, z_garra, q_yaw, pitch_global, factor_c2

def cinematica_inversa_analitica(target_r, target_z, target_yaw, pitch_ref, factor_codo2):
    q_yaw = target_yaw

    Rw = target_r - L4 * math.sin(pitch_ref)
    Zw = target_z - L4 * math.cos(pitch_ref)
    delta_Z = Zw - Z_BASE

    D2 = Rw**2 + delta_Z**2
    L23_recto = L2 + L3

    c_total = (D2 - L1**2 - L23_recto**2) / (2 * L1 * L23_recto)
    c_total = max(-0.9999, min(0.9999, c_total))
    angulo_flexion_total = math.acos(c_total)

    q_codo2 = SIGNO_CODO * min(2.25, factor_codo2 * angulo_flexion_total)

    L23 = math.sqrt(L2**2 + L3**2 + 2 * L2 * L3 * math.cos(q_codo2))
    delta_interno = math.atan2(L3 * math.sin(q_codo2), L2 + L3 * math.cos(q_codo2))

    limite_max = (L1 + L23)**2 * 0.99999
    limite_min = (L1 - L23)**2 * 1.00001
    D2_clamped = max(limite_min, min(limite_max, D2))

    c_virtual = (D2_clamped - L1**2 - L23**2) / (2 * L1 * L23)
    c_virtual = max(-1.0, min(1.0, c_virtual))
    s_virtual = SIGNO_CODO * math.sqrt(1.0 - c_virtual**2)

    q_codo1_virtual = math.atan2(s_virtual, c_virtual)
    q_codo1 = q_codo1_virtual - delta_interno

    alpha = math.atan2(Rw, delta_Z)
    beta = math.atan2(L23 * s_virtual, L1 + L23 * c_virtual)
    q_hombro = alpha - beta

    q_muneca = pitch_ref - (q_hombro + q_codo1 + q_codo2)

    return [q_yaw, q_hombro, q_codo1, q_codo2, q_muneca]

def mover_suave_a_postura(
    angulos_inicio, angulos_destino, grip_inicio, grip_destino, duracion,
    puerto_conectado, centros_fisicos, config_giro, groupSyncWrite,
    model, data, viewer, indices_qpos
):
    pasos_interp = max(1, int(duracion * 50))
    dt = duracion / pasos_interp

    for paso in range(1, pasos_interp + 1):
        t_ini = time.time()
        s = 0.5 * (1.0 - math.cos(math.pi * paso / pasos_interp))

        ang_interp = [
            angulos_inicio[i] + s * (angulos_destino[i] - angulos_inicio[i])
            for i in range(5)
        ]
        grip_interp = grip_inicio + s * (grip_destino - grip_inicio)

        if viewer is not None and viewer.is_running():
            for idx, qpos_addr in enumerate(indices_qpos):
                data.qpos[qpos_addr] = ang_interp[idx]
            mujoco.mj_forward(model, data)
            viewer.sync()

        if puerto_conectado and centros_fisicos:
            groupSyncWrite.clearParam()
            for i, servo_id in enumerate(SERVO_IDS_BRAZO):
                if servo_id in centros_fisicos:
                    pasos_dxl = rad_a_dynamixel(ang_interp[i], centros_fisicos[servo_id], config_giro[servo_id])
                    groupSyncWrite.addParam(servo_id, list((pasos_dxl & 0xFFFFFFFF).to_bytes(4, byteorder='little')))

            if SERVO_ID_GRIPPER in centros_fisicos:
                pasos_grip = rad_a_dynamixel(
                    grip_interp, centros_fisicos[SERVO_ID_GRIPPER], config_giro[SERVO_ID_GRIPPER]
                )
                groupSyncWrite.addParam(SERVO_ID_GRIPPER, list((pasos_grip & 0xFFFFFFFF).to_bytes(4, byteorder='little')))

            groupSyncWrite.txPacket()

        elapsed = time.time() - t_ini
        if elapsed < dt:
            time.sleep(dt - elapsed)

# ==========================================
# 5. BUCLE PRINCIPAL CON ANIMACIÓN CÍCLICA (50 Hz)
# ==========================================
def main():
    centros_fisicos = {}
    config_giro = {9: False, 8: True, 11: True, 7: True, 5: True, 4: False}

    angulos_prenderse = obtener_angulos_desde_tabla(OFFSETS_PRENDERSE, config_giro)
    angulos_apagarse = obtener_angulos_desde_tabla(OFFSETS_APAGARSE, config_giro)

    r_ini, z_ini, yaw_ini, pitch_ref, factor_codo2 = cinematica_directa_brazo(angulos_prenderse)

    portHandler = PortHandler(PUERTO)
    packetHandler = PacketHandler(2.0)

    puerto_conectado = True
    if not portHandler.openPort() or not portHandler.setBaudRate(BAUD_RATE):
        print(f"⚠️ No se pudo abrir {PUERTO}. Ejecutando en MODO SIMULACIÓN.")
        puerto_conectado = False

    angulos_despertar = list(angulos_prenderse)
    gripper_despertar_rad = 0.0

    if puerto_conectado:
        print("Leyendo posición actual de los encoders y normalizando offsets...")

        for servo_id in TODOS_LOS_SERVOS:
            pos_raw, res, _ = packetHandler.read4ByteTxRx(portHandler, servo_id, ADDR_PRESENT_POSITION)
            if res == COMM_SUCCESS:
                pos_actual = dxl_to_signed(pos_raw)
                centros_fisicos[servo_id] = normalizar_offset_encendido(pos_actual, OFFSETS_ORTOGONAL[servo_id])

                packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 0)
                packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, 4)

                if servo_id == SERVO_ID_GRIPPER:
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_P_GAIN, 600)
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_I_GAIN, 0)
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_D_GAIN, 1000)
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_GOAL_PWM, PWM_MAX_GRIPPER)
                    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_ACCEL, 10)
                    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_VEL, 35)
                    gripper_despertar_rad = dynamixel_a_rad(
                        pos_actual, centros_fisicos[servo_id], config_giro[servo_id]
                    )
                else:
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_P_GAIN, 1000)
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_I_GAIN, 150)
                    packetHandler.write2ByteTxRx(portHandler, servo_id, ADDR_POSITION_D_GAIN, 1200)
                    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_ACCEL, 40)
                    packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_VEL, 150)
                    idx_b = SERVO_IDS_BRAZO.index(servo_id)
                    angulos_despertar[idx_b] = dynamixel_a_rad(
                        pos_actual, centros_fisicos[servo_id], config_giro[servo_id]
                    )

                packetHandler.write4ByteTxRx(
                    portHandler, servo_id, ADDR_GOAL_POSITION, pos_actual & 0xFFFFFFFF
                )
                packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, 1)
                print(
                    f"✅ Servo {servo_id:2d} OK | Lectura actual: {pos_actual:4d} | "
                    f"Cero Ortogonal Efectivo: {centros_fisicos[servo_id]:4d}"
                )
            else:
                print(f"⚠️ Servo {servo_id} NO DETECTADO.")

    URDF_PATH = "brazo1_description/brazo1_description/urdf/brazo1.urdf"
    model = mujoco.MjModel.from_xml_path(URDF_PATH)
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch_passive(model, data)

    nombres_joints = [
        "hombro_yaw_joint", "hombro_pitch_joint", 
        "codo1_pitch_joint", "codo2_pitch_joint", "muneca_pitch_joint"
    ]
    indices_qpos = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in nombres_joints]
    indices_dof  = [model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in nombres_joints]

    groupSyncWrite = None
    groupSyncRead = None
    if puerto_conectado:
        groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4)
        groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4)
        for s_id in centros_fisicos:
            groupSyncRead.addParam(s_id)

    print("\nMoviendo suavemente a la postura inicial de trabajo...")
    mover_suave_a_postura(
        angulos_inicio=angulos_despertar,
        angulos_destino=angulos_prenderse,
        grip_inicio=gripper_despertar_rad,
        grip_destino=math.radians(GRIPPER_ABIERTO_DEG),
        duracion=2.5,
        puerto_conectado=puerto_conectado,
        centros_fisicos=centros_fisicos,
        config_giro=config_giro,
        groupSyncWrite=groupSyncWrite,
        model=model,
        data=data,
        viewer=viewer,
        indices_qpos=indices_qpos
    )
    print("✅ Brazo en postura inicial. Iniciando animación cíclica Pick & Place...")

    target_r = r_ini
    target_z = z_ini
    current_yaw = yaw_ini
    gripper_fijo_rad = math.radians(GRIPPER_ABIERTO_DEG)

    angulos_reales = list(angulos_prenderse)
    error_integral = [0.0] * 5

    # Variables de la Máquina de Estados de la Animación
    modo_automatico = True
    ciclo_num = 0
    etapa_idx = -1  # Forzará la creación del primer ciclo aleatorio al entrar al bucle
    etapas_animacion = []
    t_inicio_etapa = time.time()

    print("\nControles activos durante la ejecución (50 Hz):")
    print(" [M]     : Alternar entre MODO AUTOMÁTICO (Animación) y MODO MANUAL (WASD/QE)")
    print(" [Ctrl+C]: Terminar y estacionar suavemente en posición de descanso\n")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    DT = 1.0 / 50.0  # 50 Hz
    color_esfera = np.array([1.0, 0.5, 0.0, 1.0])
    nombre_etapa = "INICIANDO"

    try:
        while viewer.is_running():
            t_inicio = time.time()

            # --- 1. LECTURA DE TECLADO NO BLOQUEANTE ---
            if select.select([sys.stdin], [], [], 0.0)[0]:
                tecla = sys.stdin.read(1).lower()
                if tecla == '\x03':
                    break
                elif tecla == 'm':
                    modo_automatico = not modo_automatico
                    if modo_automatico:
                        etapa_idx = -1  # Reiniciar un nuevo ciclo desde la posición actual
                        print("\n🔄 MODO AUTOMÁTICO (Animación Pick & Place) ACTIVADO.")
                    else:
                        nombre_etapa = "MANUAL"
                        color_esfera = np.array([1.0, 0.0, 0.0, 1.0])
                        print("\n🕹️ MODO MANUAL ACTIVADO (Usa WASD / QE).")
                elif not modo_automatico:
                    delta_z = PASO_Z_MM / 1000.0
                    delta_r = PASO_RADIO_MM / 1000.0
                    delta_yaw = math.radians(PASO_YAW_DEG)
                    if tecla == 'w': target_z += delta_z
                    elif tecla == 's': target_z -= delta_z
                    elif tecla == 'd': target_r += delta_r
                    elif tecla == 'a': target_r -= delta_r
                    elif tecla == 'q': current_yaw += delta_yaw
                    elif tecla == 'e': current_yaw -= delta_yaw
                    elif tecla == 'h':
                        target_r, target_z, current_yaw = r_ini, z_ini, yaw_ini
                        error_integral = [0.0] * 5

            # --- 2. GENERADOR DE TRAYECTORIA AUTOMÁTICA (PICK & PLACE) ---
            if modo_automatico:
                if etapa_idx == -1 or etapa_idx >= len(etapas_animacion):
                    ciclo_num += 1
                    altura_elegida_mm = random.choice(ALTURAS_OBJETO_MM)
                    giro_elegido_deg = random.choice([-90.0, 90.0])

                    r_obj = RADIO_AGARRE_MM / 1000.0
                    z_obj = altura_elegida_mm / 1000.0
                    yaw_obj = yaw_ini + math.radians(giro_elegido_deg)

                    print(
                        f"\n🎯 [Ciclo #{ciclo_num}] Altura aleatoria: {altura_elegida_mm:.0f} mm | "
                        f"Giro Yaw: {giro_elegido_deg:+.0f}°"
                    )

                    # Definir las fases del ciclo: (nombre, p_inicio, p_fin, duracion, color_rgba)
                    p_actual = (target_r, target_z, current_yaw)
                    p_agarre = (r_obj, z_obj, yaw_ini)
                    p_suelta = (r_obj, z_obj, yaw_obj)
                    p_home   = (r_ini, z_ini, yaw_ini)

                    etapas_animacion = [
                        ("BUSCANDO OBJETO", p_actual, p_agarre, TIEMPO_MOVIMIENTO_S, [1.0, 0.6, 0.0, 1.0]),
                        ("AGARRANDO",       p_agarre, p_agarre, TIEMPO_PAUSA_ACCION_S, [0.0, 1.0, 0.0, 1.0]),
                        (f"GIRANDO {giro_elegido_deg:+.0f}°", p_agarre, p_suelta, TIEMPO_MOVIMIENTO_S, [0.0, 0.8, 1.0, 1.0]),
                        ("SOLTANDO",        p_suelta, p_suelta, TIEMPO_PAUSA_ACCION_S, [1.0, 0.0, 1.0, 1.0]),
                        ("VOLVIENDO A HOME", p_suelta, p_home,  TIEMPO_MOVIMIENTO_S, [1.0, 1.0, 0.0, 1.0]),
                        ("PAUSA EN HOME",   p_home,   p_home,   0.4,                 [0.7, 0.7, 0.7, 1.0]),
                    ]
                    etapa_idx = 0
                    t_inicio_etapa = time.time()

                # Evaluar interpolación cosenoidal de la etapa actual
                nombre_etapa, p_ini_etapa, p_fin_etapa, dur_etapa, rgba = etapas_animacion[etapa_idx]
                color_esfera = np.array(rgba)
                t_transcurrido = time.time() - t_inicio_etapa

                if t_transcurrido >= dur_etapa:
                    target_r, target_z, current_yaw = p_fin_etapa
                    etapa_idx += 1
                    t_inicio_etapa = time.time()
                else:
                    s = 0.5 * (1.0 - math.cos(math.pi * (t_transcurrido / dur_etapa)))
                    target_r = p_ini_etapa[0] + s * (p_fin_etapa[0] - p_ini_etapa[0])
                    target_z = p_ini_etapa[1] + s * (p_fin_etapa[1] - p_ini_etapa[1])
                    current_yaw = p_ini_etapa[2] + s * (p_fin_etapa[2] - p_ini_etapa[2])

            # --- 3. CINEMÁTICA INVERSA ANALÍTICA (REFERENCIA) ---
            angulos_ref = cinematica_inversa_analitica(
                target_r, target_z, current_yaw, pitch_ref, factor_codo2
            )

            # --- 4. LECTURA DE ENCODERS EN LAZO CERRADO (SYNC READ) ---
            if puerto_conectado and centros_fisicos:
                res_read = groupSyncRead.txRxPacket()
                if res_read == COMM_SUCCESS:
                    for i, s_id in enumerate(SERVO_IDS_BRAZO):
                        if s_id in centros_fisicos and groupSyncRead.isAvailable(s_id, ADDR_PRESENT_POSITION, 4):
                            pos_raw = dxl_to_signed(groupSyncRead.getData(s_id, ADDR_PRESENT_POSITION, 4))
                            angulos_reales[i] = dynamixel_a_rad(pos_raw, centros_fisicos[s_id], config_giro[s_id])
            else:
                angulos_reales = list(angulos_ref)

            # --- 5. ACTUALIZAR MUJOCO Y CALCULAR PAR DE GRAVEDAD (qfrc_bias) ---
            for idx, qpos_addr in enumerate(indices_qpos):
                data.qpos[qpos_addr] = angulos_reales[idx]

            mujoco.mj_forward(model, data)

            angulos_cmd = [0.0] * 5
            for i in range(5):
                torque_grav = data.qfrc_bias[indices_dof[i]]
                comp_gravedad = K_GRAV * torque_grav

                error_pos = angulos_ref[i] - angulos_reales[i]
                error_integral[i] += error_pos * DT
                error_integral[i] = max(-MAX_CORRECCION_RAD, min(MAX_CORRECCION_RAD, error_integral[i]))

                correccion_total = max(
                    -MAX_CORRECCION_RAD,
                    min(MAX_CORRECCION_RAD, comp_gravedad + K_I_EXT * error_integral[i])
                )
                angulos_cmd[i] = angulos_ref[i] + correccion_total

            # --- 6. DIBUJAR ESFERA OBJETIVO EN MUJOCO ---
            target_x = target_r * math.cos(current_yaw)
            target_y = target_r * math.sin(current_yaw)

            with viewer.lock():
                viewer.user_scn.ngeom = 0
                g = viewer.user_scn.geoms[0]
                mujoco.mjv_initGeom(
                    g,
                    type=mujoco.mjtGeom.mjGEOM_SPHERE,
                    size=np.array([0.014, 0, 0]),
                    pos=np.array([target_x, target_y, target_z]),
                    mat=np.eye(3).flatten(),
                    rgba=color_esfera
                )
                viewer.user_scn.ngeom = 1

            viewer.sync()

            # --- 7. ENVÍO SINCRONIZADO (BRAZO EN MOVIMIENTO + GRIPPER ESTÁTICO) ---
            if puerto_conectado and centros_fisicos:
                groupSyncWrite.clearParam()
                for i, servo_id in enumerate(SERVO_IDS_BRAZO):
                    if servo_id in centros_fisicos:
                        pasos = rad_a_dynamixel(angulos_cmd[i], centros_fisicos[servo_id], config_giro[servo_id])
                        pasos_bytes = (pasos & 0xFFFFFFFF).to_bytes(4, byteorder='little')
                        groupSyncWrite.addParam(servo_id, list(pasos_bytes))

                if SERVO_ID_GRIPPER in centros_fisicos:
                    pasos_grip = rad_a_dynamixel(
                        gripper_fijo_rad,
                        centros_fisicos[SERVO_ID_GRIPPER],
                        config_giro[SERVO_ID_GRIPPER]
                    )
                    grip_bytes = (pasos_grip & 0xFFFFFFFF).to_bytes(4, byteorder='little')
                    groupSyncWrite.addParam(SERVO_ID_GRIPPER, list(grip_bytes))

                groupSyncWrite.txPacket()

            max_err_deg = max(abs(math.degrees(angulos_ref[i] - angulos_reales[i])) for i in range(5))
            sys.stdout.write(
                f"\r[{nombre_etapa:16s}] R:{target_r*1000:5.1f}mm Z:{target_z*1000:5.1f}mm "
                f"Yaw:{math.degrees(current_yaw):6.1f}° | ErrMax:{max_err_deg:4.2f}°   "
            )
            sys.stdout.flush()

            elapsed = time.time() - t_inicio
            if elapsed < DT:
                time.sleep(DT - elapsed)

    except KeyboardInterrupt:
        print("\n\nInterrupción detectada.")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

        if puerto_conectado and centros_fisicos:
            print("\nEstacionando el brazo suavemente en la postura de reposo (3 segundos)...")
            mover_suave_a_postura(
                angulos_inicio=angulos_reales,
                angulos_destino=angulos_apagarse,
                grip_inicio=gripper_fijo_rad,
                grip_destino=gripper_fijo_rad,
                duracion=3.0,
                puerto_conectado=puerto_conectado,
                centros_fisicos=centros_fisicos,
                config_giro=config_giro,
                groupSyncWrite=groupSyncWrite,
                model=model,
                data=data,
                viewer=viewer,
                indices_qpos=indices_qpos
            )
            time.sleep(0.3)
            packetHandler.write1ByteTxOnly(portHandler, 254, ADDR_TORQUE_ENABLE, 0)
            portHandler.closePort()

        viewer.close()
        print("✅ Brazo estacionado, torque liberado y puerto cerrado.")

if __name__ == "__main__":
    main()