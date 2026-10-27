#pragma once
#include "config.h"

template<int b_pin, volatile int32_t& enc_count, int dir = 0>
void encoder_ISR() {
    if (digitalRead(b_pin) ^ dir) {
        enc_count++;
    } else {
        enc_count--;
    }
}

void encoder_begin() {
    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoder_ISR<M1_ENC_B, m1_counter, 1>, RISING);

    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), encoder_ISR<M2_ENC_B, m2_counter, 0>, RISING);

    pinMode(M3_ENC_A, INPUT_PULLUP);
    pinMode(M3_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), encoder_ISR<M3_ENC_B, m3_counter, 1>, RISING);

    pinMode(M4_ENC_A, INPUT_PULLUP);
    pinMode(M4_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M4_ENC_A), encoder_ISR<M4_ENC_B, m4_counter, 0>, RISING);
}