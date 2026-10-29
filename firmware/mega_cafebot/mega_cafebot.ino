// =====================================================================
//  Cafebot - firmware Arduino Mega 2560 (base móvil con ruedas orientables)
//
//  Responsabilidades del Mega (los "reflejos"):
//    - PI de velocidad por rueda + rampa de aceleración
//    - Servos de dirección (recto / giro en el sitio / cangrejo)
//    - Odometría y movimientos básicos (avanzar, girar, cangrejo, waypoint)
//    - 4 sensores de línea QTR-1A: umbrales, calibración, eventos de cruce y freno opcional
//    - Watchdog: si la Raspberry deja de mandar V/K, se detiene
//  La Raspberry decide QUÉ hacer; el Mega decide CÓMO mover las ruedas.
//
//  Protocolo: docs/protocolo_mega.md  (115200 baudios, texto, '\n')
// =====================================================================
#include "config.h"
#include "encoders.h"
#include "actuadores.h"
#include "ir.h"
#include "control.h"
#include "protocolo.h"

const uint32_t PERIODO_CONTROL_US = (uint32_t)(1000000.0f / TIMER_TRIGGER_FREQUENCY);
uint32_t ultimo_control_us = 0;

void setup() {
    Serial.begin(SERIAL_BAUD);
    actuadores_begin();
    encoders_begin();
    ir_begin();
    control_begin();
    ultimo_control_us = micros();
    Serial.println(F("READY " FW_VERSION));
}

void loop() {
    ir_actualizar();  // un sensor por ms (cada uno a 250 Hz)

    uint32_t ahora = micros();
    if (ahora - ultimo_control_us >= PERIODO_CONTROL_US) {
        ultimo_control_us += PERIODO_CONTROL_US;
        // Si el loop se atrasó mucho (p. ej. Serial lleno), no intentar "recuperar" ticks
        if (ahora - ultimo_control_us > PERIODO_CONTROL_US) ultimo_control_us = ahora;
        control_tick();
    }

    protocolo_actualizar();
}
