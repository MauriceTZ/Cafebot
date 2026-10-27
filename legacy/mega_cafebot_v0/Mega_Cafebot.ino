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
    // motor_begin();
    // control_begin();

    // ultimo_tick_control_us = micros();
    // imprimir_ayuda();
    // delay(5000);
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
    // cargar_ruta_prueba();
}

void loop() {
    Serial.println(m1_counter);
    delay(1000 / 20);
    // // 1. Lazo de control determinista a 60 Hz (Odometría + PID + Navegación)
    // uint32_t ahora_us = micros();
    // if (ahora_us - ultimo_tick_control_us >= PERIODO_CONTROL_US) {
    //     ultimo_tick_control_us += PERIODO_CONTROL_US;
    //     control_carro();
    // }

    // // 2. Lectura de comandos por puerto Serial (Rutas, Posición X Y W, Stop, Reset)
    // procesar_comando_serial();

    // // 3. Envío opcional de telemetría a 5 Hz (cuando se activa con 'M 1')
    // uint32_t ahora_ms = millis();
    // if (telemetria_activa && (ahora_ms - ultimo_tick_telemetria_ms >= 200)) {
    //     ultimo_tick_telemetria_ms = ahora_ms;
    //     imprimir_posicion();
    // }
}