#pragma once
#include <Servo.h>
#include <EEPROM.h>
#include "config.h"

// =====================================================================
//  Motores (puente H de la shield) y servos de dirección
// =====================================================================

enum ModoParada { PARADA_NEUTRO, PARADA_FRENO };

static const uint8_t PIN_PWM[4] = { M1_PWM, M2_PWM, M3_PWM, M4_PWM };
static const uint8_t PIN_DIR1[4] = { M1_DIR1, M2_DIR1, M3_DIR1, M4_DIR1 };
static const uint8_t PIN_DIR2[4] = { M1_DIR2, M2_DIR2, M3_DIR2, M4_DIR2 };
static const bool INV_MOTOR[4] = { M1_INV_MOTOR, M2_INV_MOTOR, M3_INV_MOTOR, M4_INV_MOTOR };
static const uint8_t PIN_SERVO[4] = { M1_SERVO_PIN, M2_SERVO_PIN, M3_SERVO_PIN, M4_SERVO_PIN };
static const uint8_t OFFSET_SERVO[4] = { M1_SERVO_OFFSET, M2_SERVO_OFFSET, M3_SERVO_OFFSET, M4_SERVO_OFFSET };

Servo servos[4];

void motor_pwm(uint8_t i, int pwm, ModoParada parada = PARADA_NEUTRO) {
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);
    bool inv = INV_MOTOR[i];
    if (pwm > 0) {
        digitalWrite(PIN_DIR1[i], inv ? HIGH : LOW);
        digitalWrite(PIN_DIR2[i], inv ? LOW : HIGH);
        analogWrite(PIN_PWM[i], pwm);
    } else if (pwm < 0) {
        digitalWrite(PIN_DIR1[i], inv ? LOW : HIGH);
        digitalWrite(PIN_DIR2[i], inv ? HIGH : LOW);
        analogWrite(PIN_PWM[i], -pwm);
    } else {
        uint8_t nivel = (parada == PARADA_FRENO) ? HIGH : LOW;
        digitalWrite(PIN_DIR1[i], nivel);
        digitalWrite(PIN_DIR2[i], nivel);
        analogWrite(PIN_PWM[i], 0);
    }
}

void motores_parar(ModoParada parada) {
    for (uint8_t i = 0; i < 4; i++) motor_pwm(i, 0, parada);
}

// ---------------------------------------------------------------------
// Dirección: cada rueda tiene un ángulo (grados, + = rueda apunta a la izquierda).
// Cambiar de ángulo requiere motores detenidos; la rampa es cosenoidal y su
// duración proporcional al mayor ángulo a recorrer.
// ---------------------------------------------------------------------
float servo_actual[4] = { 0, 0, 0, 0 };
float servo_inicio[4] = { 0, 0, 0, 0 };
float servo_destino[4] = { 0, 0, 0, 0 };
uint32_t servo_t0 = 0;
uint32_t servo_duracion = 0;
bool servo_en_rampa = false;

// Tabla de calibración: µs para -90°, 0° y +90° de cada rueda.
struct CalServos {
    uint16_t magia;
    int16_t us[4][3];
};
const uint16_t MAGIA_SERVOS = 0x5E70;
const int EEPROM_DIR_SERVOS = 32;  // la calibración IR usa las direcciones 0..~10
CalServos cal_servos;

void cal_servos_defecto() {
    cal_servos.magia = MAGIA_SERVOS;
    for (uint8_t i = 0; i < 4; i++)
        for (int8_t k = -1; k <= 1; k++)
            cal_servos.us[i][k + 1] = (int16_t)round(544 + (OFFSET_SERVO[i] + SIGNO_SERVO[i] * SERVO_ANG_CAL_DEG * k) * SERVO_US_POR_GRADO);
}

void cal_servos_cargar() {
    CalServos c;
    EEPROM.get(EEPROM_DIR_SERVOS, c);
    if (c.magia == MAGIA_SERVOS) cal_servos = c;
    else cal_servos_defecto();
}

void cal_servos_guardar() { EEPROM.put(EEPROM_DIR_SERVOS, cal_servos); }

uint16_t angulo_a_us(uint8_t i, float ang) {
    ang = constrain(ang, -SERVO_ANG_CAL_DEG, SERVO_ANG_CAL_DEG);
    const int16_t* u = cal_servos.us[i];
    float f = ang / SERVO_ANG_CAL_DEG;
    float us = (f >= 0) ? u[1] + (u[2] - u[1]) * f : u[1] + (u[1] - u[0]) * f;
    return (uint16_t)constrain((int)round(us), SERVO_US_MIN, SERVO_US_MAX);
}

void servos_escribir(const float ang[4]) {
    for (uint8_t i = 0; i < 4; i++) servos[i].writeMicroseconds(angulo_a_us(i, ang[i]));
}

// Pide un nuevo juego de ángulos. Devuelve true si ya estaban ahí.
bool servos_pedir(const float destino[4]) {
    float max_delta = 0;
    for (uint8_t i = 0; i < 4; i++) {
        float d = fabs(destino[i] - servo_destino[i]);
        if (d > max_delta) max_delta = d;
    }
    if (max_delta < 0.5f && !servo_en_rampa) return true;
    if (max_delta < 0.5f) return false;  // misma meta, rampa en curso
    for (uint8_t i = 0; i < 4; i++) {
        servo_inicio[i] = servo_actual[i];
        servo_destino[i] = destino[i];
    }
    float recorrido = 0;
    for (uint8_t i = 0; i < 4; i++) recorrido = fmaxf(recorrido, fabs(servo_destino[i] - servo_inicio[i]));
    servo_duracion = (uint32_t)fmaxf(SERVO_MS_MINIMO, recorrido * SERVO_MS_POR_GRADO);
    servo_t0 = millis();
    servo_en_rampa = true;
    return false;
}

// Avanza la rampa. Devuelve true cuando los servos llegaron y se asentaron.
bool servos_actualizar() {
    if (!servo_en_rampa) return true;
    uint32_t t = millis() - servo_t0;
    if (t < servo_duracion) {
        float s = 0.5f * (1.0f - cos(PI * (float)t / (float)servo_duracion));
        for (uint8_t i = 0; i < 4; i++)
            servo_actual[i] = servo_inicio[i] + s * (servo_destino[i] - servo_inicio[i]);
        servos_escribir(servo_actual);
        return false;
    }
    for (uint8_t i = 0; i < 4; i++) servo_actual[i] = servo_destino[i];
    servos_escribir(servo_actual);
    if (t < servo_duracion + SERVO_ASENTAMIENTO_MS) return false;
    servo_en_rampa = false;
    return true;
}

// Posturas
enum Postura : uint8_t { POSTURA_RECTO = 0, POSTURA_GIRO = 1, POSTURA_CANGREJO = 2 };
uint8_t cangrejo_espejo = CANGREJO_ESPEJO_DEF;   // patrón del cangrejo lateral (config.h, PASO 1b; comando LE)

// +1 / -1: hacia qué lado gira la rueda i al pasar a cangrejo lateral (dos y dos, para cancelar el torque)
int8_t patron_espejo(uint8_t i) {
    switch (cangrejo_espejo) {
        case 1: return ESQUINA_X[i] * ESQUINA_Y[i];
        case 2: return ESQUINA_Y[i];
        case 3: return ESQUINA_X[i];
        default: return 1;
    }
}

bool es_lateral(float a) { return fabs(fabs(a) - CANGREJO_MAX_DEG) < 0.5f; }

void angulos_postura(Postura p, float ang_cangrejo, float out[4]) {
    if (p == POSTURA_GIRO) {
        // Cada rueda perpendicular a la línea que la une con el centro del robot (rombo).
        // Para una rueda en (x, y): ángulo = -signo(x)·signo(y)·atan(L/W).
        // Ej.: delantera derecha (+,-) -> +45,8° (apunta a la izquierda).
        for (uint8_t i = 0; i < 4; i++)
            out[i] = -SIGNO_GIRO * ESQUINA_X[i] * ESQUINA_Y[i] * ANGULO_SERVOS_Z;
    } else if (p == POSTURA_CANGREJO) {
        float a = constrain(ang_cangrejo, -CANGREJO_MAX_DEG, CANGREJO_MAX_DEG);
        for (uint8_t i = 0; i < 4; i++) {
            if (!es_lateral(a)) out[i] = a;
            else if (es_lateral(servo_destino[i])) out[i] = servo_destino[i];   // ya está de lado: no se mueve
            else out[i] = patron_espejo(i) * a;                                // dos y dos: torques cancelados
        }
    } else {
        for (uint8_t i = 0; i < 4; i++) out[i] = 0;
    }
}

Servo servos_aux[N_SERVOS_AUX];

void servo_aux(uint8_t k, int grados) {
    if (k < N_SERVOS_AUX) servos_aux[k].write(constrain(grados, 0, 180));
}

void actuadores_begin() {
    for (uint8_t k = 0; k < N_SERVOS_AUX; k++) {
        servos_aux[k].attach(PIN_SERVO_AUX[k]);
        servos_aux[k].write(AUX_INICIAL_DEG[k]);
    }
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(PIN_PWM[i], OUTPUT);
        pinMode(PIN_DIR1[i], OUTPUT);
        pinMode(PIN_DIR2[i], OUTPUT);
    }
    motores_parar(PARADA_NEUTRO);
    cal_servos_cargar();
    for (uint8_t i = 0; i < 4; i++) servos[i].attach(PIN_SERVO[i], SERVO_US_MIN, SERVO_US_MAX);
    servos_escribir(servo_actual);
}
