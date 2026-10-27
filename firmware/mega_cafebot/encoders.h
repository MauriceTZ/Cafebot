#pragma once
#include "config.h"

#include <EEPROM.h>

// Un contador acumulado por rueda. El lazo de control calcula el delta de cada
// ciclo restando la lectura anterior, así no hay dos contadores que se desincronicen.
volatile int32_t enc_total[4] = { 0, 0, 0, 0 };
// Diagnóstico: lo que aporta cada flanco de A por separado (comando ED).
// En cuadratura sana, girando hacia adelante, ambos suman con el MISMO signo.
volatile int32_t enc_diag_sube[4] = { 0, 0, 0, 0 };
volatile int32_t enc_diag_baja[4] = { 0, 0, 0, 0 };
volatile bool enc_x2 = ENCODER_X2;

float mm_por_cuenta() {
    return CIRCUNFERENCIA_RUEDA / (PULSOS_POR_REVOLUCION * (enc_x2 ? 2.0f : 1.0f));
}

// Siempre se engancha en CHANGE; en modo x1 los flancos de bajada solo van al diagnóstico.
// En el flanco de subida (A=1) la condición ((A==B) ^ INV) es exactamente la de v0 (B ^ INV).
template<uint8_t A_PIN, uint8_t B_PIN, uint8_t IDX, uint8_t INV>
void isr_encoder() {
    bool a = digitalRead(A_PIN);
    bool b = digitalRead(B_PIN);
    int8_t d = ((a == b) ^ INV) ? 1 : -1;
    if (a) {
        enc_diag_sube[IDX] += d;
        enc_total[IDX] += d;
    } else {
        enc_diag_baja[IDX] += d;
        if (enc_x2) enc_total[IDX] += d;
    }
}

#define MODO_ISR_ENC CHANGE
const int EEPROM_DIR_ENC = 64;
const uint8_t MAGIA_ENC = 0xE7;

void encoders_cargar_modo() {
    uint8_t m = EEPROM.read(EEPROM_DIR_ENC);
    uint8_t v = EEPROM.read(EEPROM_DIR_ENC + 1);
    if (m == MAGIA_ENC && (v == 1 || v == 2)) enc_x2 = (v == 2);
}

void encoders_guardar_modo() {
    EEPROM.update(EEPROM_DIR_ENC, MAGIA_ENC);
    EEPROM.update(EEPROM_DIR_ENC + 1, enc_x2 ? 2 : 1);
}

void encoders_diag(int32_t sube[4], int32_t baja[4], bool reset) {
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) {
        sube[i] = enc_diag_sube[i];
        baja[i] = enc_diag_baja[i];
        if (reset) { enc_diag_sube[i] = 0; enc_diag_baja[i] = 0; }
    }
    interrupts();
}

void encoders_begin() {
    encoders_cargar_modo();
    pinMode(M1_ENC_A, INPUT_PULLUP); pinMode(M1_ENC_B, INPUT_PULLUP);
    pinMode(M2_ENC_A, INPUT_PULLUP); pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP); pinMode(M3_ENC_B, INPUT_PULLUP);
    pinMode(M4_ENC_A, INPUT_PULLUP); pinMode(M4_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), isr_encoder<M1_ENC_A, M1_ENC_B, 0, M1_INV_ENC>, MODO_ISR_ENC);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), isr_encoder<M2_ENC_A, M2_ENC_B, 1, M2_INV_ENC>, MODO_ISR_ENC);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), isr_encoder<M3_ENC_A, M3_ENC_B, 2, M3_INV_ENC>, MODO_ISR_ENC);
    attachInterrupt(digitalPinToInterrupt(M4_ENC_A), isr_encoder<M4_ENC_A, M4_ENC_B, 3, M4_INV_ENC>, MODO_ISR_ENC);
}

// Copia atómica de los 4 contadores.
void encoders_leer(int32_t out[4]) {
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) out[i] = enc_total[i];
    interrupts();
}

void encoders_reset() {
    noInterrupts();
    for (uint8_t i = 0; i < 4; i++) enc_total[i] = 0;
    interrupts();
}
