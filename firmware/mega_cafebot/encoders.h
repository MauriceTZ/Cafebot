#pragma once
#include "config.h"

// Un contador acumulado por rueda. El lazo de control calcula el delta de cada
// ciclo restando la lectura anterior, así no hay dos contadores que se desincronicen.
volatile int32_t enc_total[4] = { 0, 0, 0, 0 };

// Con CHANGE: en cada flanco de A, si A==B gira en un sentido, si no en el otro.
// La condición ((A==B) ^ INV) reproduce exactamente el signo que daba v0 con RISING.
template<uint8_t A_PIN, uint8_t B_PIN, uint8_t IDX, uint8_t INV>
void isr_encoder() {
#if ENCODER_X2
    bool adelante = (digitalRead(A_PIN) == digitalRead(B_PIN)) ^ INV;
#else
    bool adelante = digitalRead(B_PIN) ^ INV;
#endif
    if (adelante) enc_total[IDX]++;
    else enc_total[IDX]--;
}

#if ENCODER_X2
#define MODO_ISR_ENC CHANGE
#else
#define MODO_ISR_ENC RISING
#endif

void encoders_begin() {
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
