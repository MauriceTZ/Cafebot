#pragma once
#include "config.h"

void rot_servos(int m1, int m2, int m3, int m4) {
    M1_SERVO.write(M1_SERVO_OFFSET + m1);
    M2_SERVO.write(M2_SERVO_OFFSET + m2);
    M3_SERVO.write(M3_SERVO_OFFSET + m3);
    M4_SERVO.write(M4_SERVO_OFFSET + m4);
}

// Actualiza progresivamente el ángulo de los servos siguiendo una curva cosenoidal suave
void actualizar_rampa_servos() {
    if (!servos_en_transicion) return;

    uint32_t transcurrido = millis() - t_inicio_transicion_servos;

    if (transcurrido < TIEMPO_GIRO_SERVOS_MS) {
        // Interpolación cosenoidal (arranca suave, acelera al centro y frena suave al llegar)
        float progreso = (float)transcurrido / (float)TIEMPO_GIRO_SERVOS_MS;
        float s = 0.5f * (1.0f - cos(PI * progreso));

        servo_angulo_actual = servo_angulo_inicio + s * (servo_angulo_destino - servo_angulo_inicio);
        int ang_int = (int)round(servo_angulo_actual);
        rot_servos(ang_int, -ang_int, -ang_int, ang_int);
    } else {
        // Asegurar ángulo final exacto
        servo_angulo_actual = servo_angulo_destino;
        int ang_int = (int)round(servo_angulo_destino);
        rot_servos(ang_int, -ang_int, -ang_int, ang_int);
    }
}

void aplicar_canal_motor(int pwm_val, int pwm_pin, int dir1_pin, int dir2_pin, bool invert_dir, ModoParada modo_cero) {
    pwm_val = constrain(pwm_val, -PWM_MAX, PWM_MAX);

    if (pwm_val > 0) {
        digitalWrite(dir1_pin, invert_dir ? HIGH : LOW);
        digitalWrite(dir2_pin, invert_dir ? LOW : HIGH);
        analogWrite(pwm_pin, pwm_val);
    } else if (pwm_val < 0) {
        digitalWrite(dir1_pin, invert_dir ? LOW : HIGH);
        digitalWrite(dir2_pin, invert_dir ? HIGH : LOW);
        analogWrite(pwm_pin, -pwm_val);
    } else {
        // Cuando la potencia es 0, elegir entre Freno Activo (HIGH/HIGH) o Neutro (LOW/LOW)
        if (modo_cero == PARADA_FRENO) {
            digitalWrite(dir1_pin, HIGH);
            digitalWrite(dir2_pin, HIGH);
        } else {
            digitalWrite(dir1_pin, LOW);
            digitalWrite(dir2_pin, LOW);
        }
        analogWrite(pwm_pin, 0);
    }
}

// Por defecto, cuando un motor pasa por 0 dentro del lazo de control usa PARADA_NEUTRO (sin tirones)
void power(int p_m1, int p_m2, int p_m3, int p_m4, ModoParada modo_cero = PARADA_NEUTRO) {
    // M2 tiene polaridad invertida respecto a M1, M3 y M4 según test_navegacion_2.ino
    aplicar_canal_motor(p_m1, M1_PWM, M1_DIR1, M1_DIR2, false, modo_cero);
    aplicar_canal_motor(p_m2, M2_PWM, M2_DIR1, M2_DIR2, true, modo_cero);
    aplicar_canal_motor(p_m3, M3_PWM, M3_DIR1, M3_DIR2, false, modo_cero);
    aplicar_canal_motor(p_m4, M4_PWM, M4_DIR1, M4_DIR2, false, modo_cero);
}

void motor_begin() {
    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR1, OUTPUT);
    pinMode(M1_DIR2, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR1, OUTPUT);
    pinMode(M2_DIR2, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    pinMode(M3_DIR1, OUTPUT);
    pinMode(M3_DIR2, OUTPUT);
    pinMode(M4_PWM, OUTPUT);
    pinMode(M4_DIR1, OUTPUT);
    pinMode(M4_DIR2, OUTPUT);

    // Iniciar motores en Neutro
    power(0, 0, 0, 0, PARADA_NEUTRO);

    M1_SERVO.attach(M1_SERVO_PIN);
    M2_SERVO.attach(M2_SERVO_PIN);
    M3_SERVO.attach(M3_SERVO_PIN);
    M4_SERVO.attach(M4_SERVO_PIN);

    rot_servos(0, 0, 0, 0);
    postura_servos_actual = SERVOS_RECTO;
    servo_angulo_actual = 0.0f;
}