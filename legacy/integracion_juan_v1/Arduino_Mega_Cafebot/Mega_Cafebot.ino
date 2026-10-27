#include "config.h"
#include "encoder.h"
#include "motor.h"
#include "cinematica.h"
#include "control.h"
#include "serial_cmd.h"

uint32_t ultimo_tick_control_us = 0;
uint32_t ultimo_tick_telemetria_ms = 0;
const uint32_t PERIODO_CONTROL_US = (uint32_t)(1000000.0f / TIMER_TRIGGER_FREQUENCY);  // 16666 us (60 Hz)

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(15);  // Evita bloqueos al leer comandos seriales

    encoder_begin();
    motor_begin();
    control_begin();

    ultimo_tick_control_us = micros();
    ultimo_comando_vel_ms = millis();
    modo_actual = QUIETO;
    detener_motores_y_pids(PARADA_NEUTRO);
    imprimir_ayuda();
    Serial.println(F("READY: control de motores activo; odometria global en Raspberry."));
}

void loop() {
    // 1. Lazo de control determinista a 60 Hz (PID/velocidades; la odometría global corre en Raspberry)
    uint32_t ahora_us = micros();
    if (ahora_us - ultimo_tick_control_us >= PERIODO_CONTROL_US) {
        ultimo_tick_control_us += PERIODO_CONTROL_US;
        control_carro();
    }

    // 2. Lectura de comandos por puerto Serial (Rutas, Posición X Y W, Stop, Reset)
    procesar_comando_serial();

    // 3. Envío opcional de telemetría a 5 Hz (cuando se activa con 'M 1')
    uint32_t ahora_ms = millis();
    if (telemetria_activa && (ahora_ms - ultimo_tick_telemetria_ms >= 200)) {
        ultimo_tick_telemetria_ms = ahora_ms;
        imprimir_posicion();
    }
}