"""Cinemática del brazo (5 GDL + garra) con cinemática inversa de MÍNIMO ESFUERZO.

Convenciones (las mismas de legacy/codigo_brazo/main.py, para reutilizar sus offsets):
  - Ángulos articulares en rad. 0 = eslabón vertical hacia arriba.
  - Plano del brazo: r = distancia horizontal desde el eje de la base, z = altura
    desde la base del brazo. El yaw de la base gira ese plano.
  - pitch = inclinación absoluta del último tramo (garra): 90° = horizontal hacia
    adelante, <90° = garra apuntando hacia arriba.

Por qué "mínimo esfuerzo": el brazo tiene 3 articulaciones de cabeceo (hombro, codo1,
codo2) para ubicar la muñeca en un plano, así que hay infinitas soluciones para el
mismo punto. La IK antigua elegía una con un reparto fijo entre codo1 y codo2. Aquí se
barre esa libertad y se elige la postura que minimiza el torque por gravedad,
normalizado por el torque máximo del XL430 (1,4 N·m a 11,1 V, manual de ROBOTIS).
Menos torque sostenido = menos corriente, menos temperatura y menos riesgo de que
salte la protección de sobrecarga (Shutdown bit 5, que apaga el torque hasta reiniciar).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81


@dataclass
class ModeloBrazo:
    # Geometría (m) - legacy/codigo_brazo/main.py
    L1: float = 0.107       # hombro -> codo1
    L2: float = 0.107       # codo1 -> codo2
    L3: float = 0.067       # codo2 -> muñeca
    L4: float = 0.032       # muñeca -> punto de agarre (el "target" de la IK)
    z_hombro: float = 0.0671  # base del brazo -> eje del hombro
    # Masas (kg). Servos: 57,2 g (manual XL430). Piezas impresas: masas del URDF.
    m_servo: float = 0.0572
    m_pieza_L1: float = 0.0097
    m_pieza_L2: float = 0.0225
    m_pieza_L3: float = 0.0225
    m_pieza_L4: float = 0.0149
    m_garra: float = 0.0572 + 0.0217   # servo de la garra + dedos
    m_carga: float = 0.012             # un grano (12 ± 4 g)
    # Torque máximo de referencia (N·m). Bloqueo a 11,1 V = 1,4; a 12 V = 1,5.
    tau_max: float = 1.4
    # Límites articulares relativos (rad): hombro, codo1, codo2, muñeca     [AJUSTAR al robot]
    limites: tuple = field(default_factory=lambda: (
        (math.radians(-100), math.radians(100)),
        (math.radians(-150), math.radians(150)),
        (-2.25, 2.25),                       # tope de descanso del codo2 (paso 78)
        (math.radians(-120), math.radians(120)),
    ))


def puntos(m: ModeloBrazo, q: np.ndarray) -> np.ndarray:
    """Posiciones (r, z) del hombro, codo1, codo2, muñeca y punto de agarre.
    q: (..., 4) = [hombro, codo1, codo2, muñeca]. Devuelve (..., 5, 2)."""
    q = np.asarray(q, dtype=float)
    t = np.cumsum(q, axis=-1)                      # ángulos absolutos de cada tramo
    largos = np.array([m.L1, m.L2, m.L3, m.L4])
    d = np.stack([np.sin(t), np.cos(t)], axis=-1) * largos[..., :, None]
    p0 = np.zeros(q.shape[:-1] + (1, 2))
    p0[..., 0, 1] = m.z_hombro
    return np.concatenate([p0, p0 + np.cumsum(d, axis=-2)], axis=-2)


def directa(m: ModeloBrazo, q) -> tuple[float, float, float]:
    """(r, z, pitch) del punto de agarre."""
    p = puntos(m, q)
    return float(p[4, 0]), float(p[4, 1]), float(np.sum(q))


def torques_gravedad(m: ModeloBrazo, q, con_carga: bool = True) -> np.ndarray:
    """Torque estático (N·m) en hombro, codo1, codo2 y muñeca. Acepta lotes (..., 4)."""
    p = puntos(m, q)
    r = p[..., 0]                       # (..., 5)
    mid = 0.5 * (r[..., :-1] + r[..., 1:])  # centro de cada tramo
    # (posición horizontal, masa, índice del primer joint que NO la carga)
    masas = [
        (mid[..., 0], m.m_pieza_L1, 1), (r[..., 1], m.m_servo, 1),
        (mid[..., 1], m.m_pieza_L2, 2), (r[..., 2], m.m_servo, 2),
        (mid[..., 2], m.m_pieza_L3, 3), (r[..., 3], m.m_servo, 3),
        (mid[..., 3], m.m_pieza_L4, 4), (r[..., 4], m.m_garra + (m.m_carga if con_carga else 0.0), 4),
    ]
    tau = np.zeros(np.shape(q))
    for j in range(4):
        rj = r[..., j]
        for x, masa, hasta in masas:
            if j < hasta:
                tau[..., j] += G * masa * (x - rj)
    return tau


def costo(m: ModeloBrazo, q, peso_suma: float = 0.25) -> np.ndarray:
    """Carga del servo MÁS exigido al cuadrado (es el que se recalienta y dispara la
    protección), más una fracción de la suma de todos (calor total, ∝ corriente²)."""
    u = (torques_gravedad(m, q) / m.tau_max) ** 2
    return np.max(u, axis=-1) + peso_suma * np.sum(u, axis=-1)


@dataclass
class SolucionIK:
    q: np.ndarray            # [hombro, codo1, codo2, muñeca] rad
    pitch: float
    costo: float
    tau: np.ndarray          # N·m por articulación
    margen_alcance: float    # m que faltan para estirar del todo (0 = singular)


def inversa(m: ModeloBrazo, r: float, z: float, pitch: float | tuple[float, float] = math.pi / 2,
            muestras: int = 181) -> SolucionIK | None:
    """IK de mínimo esfuerzo para el punto de agarre (r, z).

    pitch: ángulo fijo de la garra (rad) o un rango (min, max) para que también lo elija.
    Devuelve None si no hay solución dentro de los límites articulares.
    """
    pitches = np.linspace(pitch[0], pitch[1], 13) if isinstance(pitch, tuple) else np.array([pitch])
    # Barrido: pitch × ángulo absoluto del tramo L3 × rama del codo
    P, T3 = np.meshgrid(pitches, np.linspace(-math.pi, math.pi, muestras), indexing="ij")
    P, T3 = P.ravel(), T3.ravel()
    wr = r - m.L4 * np.sin(P)
    wz = z - m.L4 * np.cos(P)
    # Punto codo2 (inicio de L3)
    cr = wr - m.L3 * np.sin(T3)
    cz = wz - m.L3 * np.cos(T3) - m.z_hombro
    d2 = cr ** 2 + cz ** 2
    c = (d2 - m.L1 ** 2 - m.L2 ** 2) / (2 * m.L1 * m.L2)
    ok = np.abs(c) <= 1.0
    if not np.any(ok):
        return None
    P, T3, cr, cz, c = P[ok], T3[ok], cr[ok], cz[ok], c[ok]
    sols = []
    for signo in (1.0, -1.0):
        q2 = signo * np.arccos(np.clip(c, -1, 1))              # codo1 relativo
        q1 = np.arctan2(cr, cz) - np.arctan2(m.L2 * np.sin(q2), m.L1 + m.L2 * np.cos(q2))
        q3 = T3 - q1 - q2                                      # codo2 relativo
        q4 = P - T3                                            # muñeca relativa
        q = np.stack([q1, q2, q3, q4], axis=-1)
        q = (q + math.pi) % (2 * math.pi) - math.pi
        sols.append(q)
    q = np.concatenate(sols)
    lim = np.array(m.limites)
    dentro = np.all((q >= lim[:, 0]) & (q <= lim[:, 1]), axis=-1)
    if not np.any(dentro):
        return None
    q = q[dentro]
    k = costo(m, q)
    i = int(np.argmin(k))
    qi = q[i]
    alcance = math.hypot(r - m.L4 * math.sin(np.sum(qi)), z - m.L4 * math.cos(np.sum(qi)) - m.z_hombro)
    return SolucionIK(qi, float(np.sum(qi)), float(k[i]), torques_gravedad(m, qi),
                      m.L1 + m.L2 + m.L3 - alcance)


def inversa_legado(m: ModeloBrazo, r: float, z: float, pitch_ref: float, factor_codo2: float) -> np.ndarray:
    """La IK de legacy/codigo_brazo/main.py (reparto fijo codo1/codo2), para comparar."""
    Rw = r - m.L4 * math.sin(pitch_ref)
    Zw = z - m.L4 * math.cos(pitch_ref)
    dZ = Zw - m.z_hombro
    D2 = Rw ** 2 + dZ ** 2
    L23r = m.L2 + m.L3
    ct = max(-0.9999, min(0.9999, (D2 - m.L1 ** 2 - L23r ** 2) / (2 * m.L1 * L23r)))
    q3 = min(2.25, factor_codo2 * math.acos(ct))
    L23 = math.sqrt(m.L2 ** 2 + m.L3 ** 2 + 2 * m.L2 * m.L3 * math.cos(q3))
    di = math.atan2(m.L3 * math.sin(q3), m.L2 + m.L3 * math.cos(q3))
    D2c = max((m.L1 - L23) ** 2 * 1.00001, min((m.L1 + L23) ** 2 * 0.99999, D2))
    cv = max(-1.0, min(1.0, (D2c - m.L1 ** 2 - L23 ** 2) / (2 * m.L1 * L23)))
    sv = math.sqrt(1.0 - cv ** 2)
    q2 = math.atan2(sv, cv) - di
    q1 = math.atan2(Rw, dZ) - math.atan2(L23 * sv, m.L1 + L23 * cv)
    q4 = pitch_ref - (q1 + q2 + q3)
    return np.array([q1, q2, q3, q4])
