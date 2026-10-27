#pragma once
#include "config.h"

// Cinemática Directa según la postura actual de los servos:
// V_m13: velocidad media (mm/s) de los motores M1 y M3
// V_m24: velocidad media (mm/s) de los motores M2 y M4
void forw_kinematics(float V_m13, float V_m24, float& V_bx, float& W_bz) {
    if (postura_servos_actual == SERVOS_GIRO_Z) {
        // Ruedas a +-45°: giro puro sobre el eje Z sin traslación lineal
        V_bx = 0.0f;
        // Con M1,M3 (+) y M2,M4 (-) el carro gira a la izquierda (+W_bz)
        W_bz = (V_m13 - V_m24) / CINEMATICA_D_GIRO_Z;
    } else {
        // Ruedas a 0°: traslación longitudinal
        V_bx = (V_m13 + V_m24) * 0.5f;
        W_bz = (V_m13 - V_m24) / CINEMATICA_W;
    }
}

// Cinemática Inversa según la postura actual de los servos:
void inv_kinematics(float V_bx, float W_bz, float& V_m13, float& V_m24) {
    if (postura_servos_actual == SERVOS_GIRO_Z) {
        // Giro puro sobre eje Z con ruedas a +-45°
        V_m13 = +W_bz * (CINEMATICA_D_GIRO_Z * 0.5f);
        V_m24 = -W_bz * (CINEMATICA_D_GIRO_Z * 0.5f);
    } else {
        // Avance en postura recta (0°)
        V_m13 = V_bx;
        V_m24 = V_bx;
    }
}