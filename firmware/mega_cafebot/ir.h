#pragma once
#include <EEPROM.h>
#include "config.h"

// =====================================================================
//  Sensores IR de borde: 2 digitales (delante) + 2 analógicos (atrás)
//  bit 0 = DI, bit 1 = DD, bit 2 = TI, bit 3 = TD   (1 = ve NEGRO)
// =====================================================================

struct CalibIR {
    uint16_t magia;
    int16_t blanco[2];  // lectura ADC sobre lona blanca (TI, TD)
    int16_t negro[2];   // lectura ADC sobre línea negra
};
const uint16_t MAGIA_IR = 0xCAFE;
CalibIR calib_ir = { MAGIA_IR, { 200, 200 }, { 800, 800 } };

uint8_t ir_mascara = 0;        // estado filtrado actual
uint8_t ir_flancos_subida = 0; // bits que pasaron de blanco a negro (se consumen)
uint8_t ir_flancos_bajada = 0; // bits que pasaron de negro a blanco (se consumen)
int16_t ir_analog[2] = { 0, 0 };
uint32_t ir_ultimo_us = 0;
uint8_t ir_cuenta[4] = { 0, 0, 0, 0 };
#define IR_MUESTRAS_CONFIRMAR 2   // 2 lecturas seguidas (2 ms) para cambiar de estado

// Umbral con histéresis: se pasa a "negro" al cruzar el 60 % entre blanco y negro y se
// vuelve a "blanco" bajo el 40 %. Funciona aunque el sensor dé valores invertidos.
bool analog_es_negro(uint8_t k, int16_t v, bool estaba_negro) {
    float b = calib_ir.blanco[k], n = calib_ir.negro[k];
    if (fabs(n - b) < 20) return false;  // sin calibrar o sensor muerto
    float frac = (v - b) / (n - b);       // 0 = blanco, 1 = negro
    return estaba_negro ? (frac > 0.40f) : (frac > 0.60f);
}

void ir_cargar_eeprom() {
    CalibIR c;
    EEPROM.get(0, c);
    if (c.magia == MAGIA_IR) calib_ir = c;
}

void ir_guardar_eeprom() { EEPROM.put(0, calib_ir); }

void ir_begin() {
    pinMode(IR_DI_PIN, INPUT);
    pinMode(IR_DD_PIN, INPUT);
    ir_cargar_eeprom();
}

// Promedio de varias lecturas para calibrar (k = 0 TI, 1 TD)
int16_t ir_promedio(uint8_t k) {
    uint8_t pin = (k == 0) ? IR_TI_PIN : IR_TD_PIN;
    int32_t s = 0;
    for (uint8_t i = 0; i < 32; i++) s += analogRead(pin);
    return (int16_t)(s / 32);
}

void ir_calibrar_blanco() {
    calib_ir.blanco[0] = ir_promedio(0);
    calib_ir.blanco[1] = ir_promedio(1);
}
// Negro: se calibra cada sensor por separado poniéndolo sobre la línea.
void ir_calibrar_negro(uint8_t k) { calib_ir.negro[k] = ir_promedio(k); }

void ir_actualizar() {
    uint32_t ahora = micros();
    if (ahora - ir_ultimo_us < IR_PERIODO_US) return;
    ir_ultimo_us = ahora;

    bool crudo[4];
    crudo[0] = (digitalRead(IR_DI_PIN) == (IR_DIGITAL_NEGRO_ES_HIGH ? HIGH : LOW));
    crudo[1] = (digitalRead(IR_DD_PIN) == (IR_DIGITAL_NEGRO_ES_HIGH ? HIGH : LOW));
    ir_analog[0] = analogRead(IR_TI_PIN);
    ir_analog[1] = analogRead(IR_TD_PIN);
    crudo[2] = analog_es_negro(0, ir_analog[0], ir_mascara & 0x04);
    crudo[3] = analog_es_negro(1, ir_analog[1], ir_mascara & 0x08);

    for (uint8_t i = 0; i < 4; i++) {
        bool actual = ir_mascara & (1 << i);
        if (crudo[i] != actual) {
            if (++ir_cuenta[i] >= IR_MUESTRAS_CONFIRMAR) {
                ir_cuenta[i] = 0;
                ir_mascara ^= (1 << i);
                if (crudo[i]) ir_flancos_subida |= (1 << i);
                else ir_flancos_bajada |= (1 << i);
            }
        } else {
            ir_cuenta[i] = 0;
        }
    }
}
