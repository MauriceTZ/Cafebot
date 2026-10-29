#pragma once
#include <EEPROM.h>
#include "config.h"

// =====================================================================
//  Sensores IR de línea: 4 × QTR-1A analógicos (ver config.h, sección 1b)
//  bit 0 = DI, bit 1 = DD, bit 2 = TI, bit 3 = TD   (1 = ve NEGRO)
//
//  Por sensor se guardan: nivel BLANCO, nivel NEGRO y UMBRAL (por defecto, el punto medio)
//  + una histéresis común. Calibración (docs/ir.md):
//    IW      blanco de los 4 (robot sobre la lona)
//    IN [k]  negro del sensor k, o de los 4 (sobre la línea)
//    IA 1/0  automática: registrar mín/máx mientras el robot pasa sobre una línea; al terminar
//            blanco = mín, negro = máx, umbral = punto medio
//    IU k v  fijar a mano el umbral del sensor k;  IH v  histéresis;  IG guardar;  ID fábrica
// =====================================================================

struct CalibIR {
    uint16_t magia;
    int16_t blanco[IR_N];
    int16_t negro[IR_N];
    int16_t umbral[IR_N];
    int16_t histeresis;
};
const uint16_t MAGIA_IR = 0xCA04;  // distinta a la versión anterior (2 analógicos): no se mezclan
const int EEPROM_DIR_IR = 0;       // 0..27 (la tabla de servos empieza en 32)
CalibIR calib_ir;

uint8_t ir_mascara = 0;            // estado filtrado actual
uint8_t ir_flancos_subida = 0;     // bits que pasaron de blanco a negro (los consume control_tick)
uint8_t ir_flancos_bajada = 0;     // bits que pasaron de negro a blanco
uint8_t ir_mascara_freno = 0x0F;   // qué sensores activan el freno por borde (IS 1 [mascara])
uint32_t ir_t_flanco_us = 0;       // cuándo se confirmó el primer flanco aún no informado
// Desde que el sensor cruza el umbral hasta que se confirma pasan, en promedio, medio período de
// muestreo de ese sensor + las lecturas de confirmación. El evento se corrige con este retraso.
const uint32_t IR_RETRASO_US = (uint32_t)(IR_MUESTRAS_CONFIRMAR - 1) * IR_N * IR_PERIODO_US + IR_N * IR_PERIODO_US / 2;
int16_t ir_valor[IR_N] = { 0, 0, 0, 0 };
uint8_t ir_cuenta[IR_N] = { 0, 0, 0, 0 };
uint8_t ir_siguiente = 0;          // round-robin
uint32_t ir_ultimo_us = 0;
// Auto-calibración
bool ir_auto = false;
int16_t ir_min[IR_N], ir_max[IR_N];

void ir_calib_defecto() {
    calib_ir.magia = MAGIA_IR;
    for (uint8_t k = 0; k < IR_N; k++) {
        calib_ir.blanco[k] = IR_NEGRO_ES_ALTO ? IR_BLANCO_DEF : IR_NEGRO_DEF;
        calib_ir.negro[k] = IR_NEGRO_ES_ALTO ? IR_NEGRO_DEF : IR_BLANCO_DEF;
        calib_ir.umbral[k] = (IR_BLANCO_DEF + IR_NEGRO_DEF) / 2;
    }
    calib_ir.histeresis = IR_HISTERESIS_DEF;
}

void ir_cargar_eeprom() {
    CalibIR c;
    EEPROM.get(EEPROM_DIR_IR, c);
    if (c.magia == MAGIA_IR) calib_ir = c;
    else ir_calib_defecto();
}

void ir_guardar_eeprom() { EEPROM.put(EEPROM_DIR_IR, calib_ir); }

// El negro da lecturas más ALTAS que el blanco (QTR-1A). Si la calibración dice lo contrario,
// el sensor está invertido y se compara al revés.
bool ir_negro_alto(uint8_t k) { return calib_ir.negro[k] >= calib_ir.blanco[k]; }

bool ir_es_negro(uint8_t k, int16_t v, bool estaba_negro) {
    int16_t u = calib_ir.umbral[k], h = calib_ir.histeresis / 2;
    if (!ir_negro_alto(k)) { v = -v; u = -u; }
    return estaba_negro ? (v > u - h) : (v > u + h);
}

// Lectura de un canal. El QTR-1A tiene una salida de alta impedancia (resistencia de 47 kΩ):
// tras cambiar de canal, la primera conversión del ADC arrastra el voltaje del canal anterior.
// Por eso se descarta una lectura.
int16_t ir_leer(uint8_t k) {
    analogRead(IR_PINES[k]);
    return analogRead(IR_PINES[k]);
}

int16_t ir_promedio(uint8_t k) {
    int32_t s = 0;
    for (uint8_t i = 0; i < 32; i++) s += ir_leer(k);
    return (int16_t)(s / 32);
}

// Recalcula el umbral del sensor k como el punto medio. Devuelve false si el contraste es bajo.
bool ir_umbral_medio(uint8_t k) {
    int16_t b = calib_ir.blanco[k], n = calib_ir.negro[k];
    if (abs(n - b) < IR_CONTRASTE_MIN) return false;
    calib_ir.umbral[k] = (b + n) / 2;
    return true;
}

void ir_calibrar_blanco() {
    for (uint8_t k = 0; k < IR_N; k++) {
        calib_ir.blanco[k] = ir_promedio(k);
        ir_umbral_medio(k);
    }
}

// Devuelve la lectura medida; solo se guarda si se distingue del blanco (si no, el sensor no
// estaba sobre la línea, está muy alto o desconectado).
int16_t ir_calibrar_negro(uint8_t k, bool* ok) {
    int16_t v = ir_promedio(k);
    *ok = abs(v - calib_ir.blanco[k]) >= IR_CONTRASTE_MIN;
    if (*ok) {
        calib_ir.negro[k] = v;
        ir_umbral_medio(k);
    }
    return v;
}

void ir_auto_iniciar() {
    for (uint8_t k = 0; k < IR_N; k++) { ir_min[k] = 1023; ir_max[k] = 0; }
    ir_auto = true;
}

// Termina la auto-calibración. Devuelve la máscara de sensores calibrados (contraste suficiente).
uint8_t ir_auto_terminar() {
    ir_auto = false;
    uint8_t ok = 0;
    for (uint8_t k = 0; k < IR_N; k++) {
        if (ir_max[k] - ir_min[k] < IR_CONTRASTE_MIN) continue;   // no vio la línea (o está muy alto)
        calib_ir.blanco[k] = IR_NEGRO_ES_ALTO ? ir_min[k] : ir_max[k];
        calib_ir.negro[k] = IR_NEGRO_ES_ALTO ? ir_max[k] : ir_min[k];
        ir_umbral_medio(k);
        ok |= (1 << k);
    }
    return ok;
}

void ir_begin() {
    for (uint8_t k = 0; k < IR_N; k++) pinMode(IR_PINES[k], INPUT);
    ir_cargar_eeprom();
}

void ir_actualizar() {
    uint32_t ahora = micros();
    if (ahora - ir_ultimo_us < IR_PERIODO_US) return;
    ir_ultimo_us = ahora;

    uint8_t k = ir_siguiente;
    ir_siguiente = (ir_siguiente + 1) % IR_N;
    int16_t v = ir_leer(k);
    ir_valor[k] = v;
    if (ir_auto) {
        if (v < ir_min[k]) ir_min[k] = v;
        if (v > ir_max[k]) ir_max[k] = v;
    }
    bool actual = ir_mascara & (1 << k);
    bool crudo = ir_es_negro(k, v, actual);
    if (crudo != actual) {
        if (++ir_cuenta[k] >= IR_MUESTRAS_CONFIRMAR) {
            ir_cuenta[k] = 0;
            if (!(ir_flancos_subida | ir_flancos_bajada)) ir_t_flanco_us = ahora;
            ir_mascara ^= (1 << k);
            if (crudo) ir_flancos_subida |= (1 << k);
            else ir_flancos_bajada |= (1 << k);
        }
    } else {
        ir_cuenta[k] = 0;
    }
}
