#pragma once
#include <Arduino.h>
#include <Servo.h>
#include "pid.h"

// ==========================================
// 1. PINOUT PLACA MOEBIUS 4MOTOR & 9SERVO
// ==========================================
#define M1_ENC_A 18
#define M1_ENC_B 31
#define M1_PWM 12
#define M1_DIR1 34
#define M1_DIR2 35
#define M1_SERVO_PIN 13
#define M1_SERVO_OFFSET 87

#define M2_ENC_A 19
#define M2_ENC_B 38
#define M2_PWM 8
#define M2_DIR1 37
#define M2_DIR2 36
#define M2_SERVO_PIN 10
#define M2_SERVO_OFFSET 87

#define M3_ENC_A 3
#define M3_ENC_B 49
#define M3_PWM 9
#define M3_DIR1 43
#define M3_DIR2 42
#define M3_SERVO_PIN 45
#define M3_SERVO_OFFSET 90

#define M4_ENC_A 2
#define M4_ENC_B A1
#define M4_PWM 5
#define M4_DIR1 A4
#define M4_DIR2 A5
#define M4_SERVO_PIN 6
#define M4_SERVO_OFFSET 89

Servo M1_SERVO, M2_SERVO, M3_SERVO, M4_SERVO;

volatile int32_t m1_counter = 0;
volatile int32_t m2_counter = 0;
volatile int32_t m3_counter = 0;
volatile int32_t m4_counter = 0;

// ==========================================
// 2. PARÁMETROS CINEMÁTICOS Y DE SERVOS
// ==========================================
#define TIMER_TRIGGER_FREQUENCY 60.0f
const float dt = 1.0f / TIMER_TRIGGER_FREQUENCY;

#define PULSOS_POR_REVOLUCION 800.0f
#define CINEMATICA_W 217.0f              // Ancho entre ruedas (mm)
#define CINEMATICA_L 223.0f              // Largo entre eje delantero y trasero (mm) - Ajusta a tu chasis
#define CINEMATICA_DIAMETRO_RUEDA 53.0f  // Diámetro de rueda (mm)
const float CIRCUNFERENCIA_RUEDA = PI * CINEMATICA_DIAMETRO_RUEDA;

// Diámetro de la circunferencia de giro y ángulo tangente exacto
const float CINEMATICA_D_GIRO_Z = sqrt((CINEMATICA_W * CINEMATICA_W) + (CINEMATICA_L * CINEMATICA_L));
const float ANGULO_SERVOS_Z = atan2(CINEMATICA_L, CINEMATICA_W) * 180.0f / PI;

// Duración de la rampa suave de los servos (ms)
#define TIEMPO_GIRO_SERVOS_MS 1500
// Pausa adicional tras terminar de mover los servos para estabilizar antes de traccionar (ms)
#define TIEMPO_ASENTAMIENTO_SERVOS_MS 100

enum PosturaServos {
    SERVOS_RECTO,  // 0 grados
    SERVOS_GIRO_Z  // +-ANGULO_SERVOS_Z grados
};

enum ModoParada {
    PARADA_NEUTRO,  // DIR1=LOW, DIR2=LOW (Rueda libre / poca resistencia)
    PARADA_FRENO    // DIR1=HIGH, DIR2=HIGH (Freno electromagnético corto)
};

PosturaServos postura_servos_actual = SERVOS_RECTO;
bool servos_en_transicion = false;
uint32_t t_inicio_transicion_servos = 0;

// Variables para la interpolación suave de los servos
float servo_angulo_actual = 0.0f;
float servo_angulo_inicio = 0.0f;
float servo_angulo_destino = 0.0f;

// ==========================================
// 3. PARÁMETROS PID Y NAVEGACIÓN
// ==========================================
#define PWM_MAX 255
#define PWM_POR_REVOLUCION (255.0f * 0.285f)

#define VELOCIDAD_NAVEGACION 100.0f//200.0f  // mm/s en línea recta
#define VELOCIDAD_ANGULAR_MAX 0.5f//1.5f   // rad/s girando sobre eje Z

#define KP_DISTANCIA 1.5f
#define KP_ANGULO 2.2f
#define TOLERANCIA_DISTANCIA_MM 15.0f   // 15 mm
#define TOLERANCIA_ANGULO_RAD 0.04f    // ~2.3 grados
#define UMBRAL_DESVIO_RUMBO_RAD 0.18f  // ~10 grados (si se desvía más al avanzar, reorienta en Z)

PID m1_pid{ 0.3f, 0.06f, 0.0f };
PID m2_pid{ 0.3f, 0.06f, 0.0f };
PID m3_pid{ 0.3f, 0.06f, 0.0f };
PID m4_pid{ 0.3f, 0.06f, 0.0f };

int32_t m1_pwm = 0, m2_pwm = 0, m3_pwm = 0, m4_pwm = 0;

// ==========================================
// 4. ESTADO DEL CARRO Y COLA DE NAVEGACIÓN
// ==========================================
struct Pose2D {
    float x;  // mm
    float y;  // mm
    float w;  // rad [-PI, PI]
};

struct VelDiff {
    float vx;  // mm/s
    float wz;  // rad/s
};

Pose2D posicion_carro_actual = { 0.0f, 0.0f, 0.0f };
Pose2D posicion_carro_objetivo = { 0.0f, 0.0f, 0.0f };
bool hay_objetivo_activo = false;

enum FaseWaypoint {
    FASE_APUNTAR_RUMBO,
    FASE_AVANZAR_RECTO,
    FASE_ALINEAR_W_FINAL
};

FaseWaypoint fase_actual = FASE_APUNTAR_RUMBO;

VelDiff velocidad_carro_actual = { 0.0f, 0.0f };
VelDiff velocidad_carro_objetivo = { 0.0f, 0.0f };

enum MODO {
    QUIETO,
    NAVEGACION_RUTA,
    CONTROL_SERIAL_VEL
};

MODO modo_actual = QUIETO;
bool telemetria_activa = false;

#define LARGO_COLA_NAVEGACION 50

struct WaypointQueue {
    float buffer[LARGO_COLA_NAVEGACION][3];
    uint8_t head = 0;
    uint8_t tail = 0;
    uint8_t count = 0;
};

typedef WaypointQueue* QueueHandle_t;
QueueHandle_t cola_navegacion = NULL;

QueueHandle_t xQueueCreate(uint8_t /*length*/, size_t /*item_size*/) {
    static WaypointQueue q;
    q.head = 0;
    q.tail = 0;
    q.count = 0;
    return &q;
}

bool xQueueSend(QueueHandle_t q, const float item[3], uint32_t /*timeout*/ = 0) {
    if (q == NULL || q->count >= LARGO_COLA_NAVEGACION) return false;
    q->buffer[q->tail][0] = item[0];
    q->buffer[q->tail][1] = item[1];
    q->buffer[q->tail][2] = item[2];
    q->tail = (q->tail + 1) % LARGO_COLA_NAVEGACION;
    q->count++;
    return true;
}

bool xQueueReceive(QueueHandle_t q, float item[3], uint32_t /*timeout*/ = 0) {
    if (q == NULL || q->count == 0) return false;
    item[0] = q->buffer[q->head][0];
    item[1] = q->buffer[q->head][1];
    item[2] = q->buffer[q->head][2];
    q->head = (q->head + 1) % LARGO_COLA_NAVEGACION;
    q->count--;
    return true;
}

void xQueueReset(QueueHandle_t q) {
    if (q != NULL) {
        q->head = 0;
        q->tail = 0;
        q->count = 0;
    }
}

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float wrapAngle(float angle) {
    while (angle > PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}