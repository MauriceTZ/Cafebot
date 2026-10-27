#pragma once
#include "config.h"

template<int b_pin, volatile int32_t& enc_count, volatile int32_t& total_count, int dir = 0>
void encoder_ISR() {
    int8_t delta = (digitalRead(b_pin) ^ dir) ? 1 : -1;
    enc_count += delta;
    total_count += delta;
}

void encoder_begin() {
    pinMode(M1_ENC_A, INPUT_PULLUP);
    pinMode(M1_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoder_ISR<M1_ENC_B, m1_counter, m1_total_ticks, 1>, RISING);

    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), encoder_ISR<M2_ENC_B, m2_counter, m2_total_ticks, 0>, RISING);

    pinMode(M3_ENC_A, INPUT_PULLUP);
    pinMode(M3_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), encoder_ISR<M3_ENC_B, m3_counter, m3_total_ticks, 1>, RISING);

    pinMode(M4_ENC_A, INPUT_PULLUP);
    pinMode(M4_ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(M4_ENC_A), encoder_ISR<M4_ENC_B, m4_counter, m4_total_ticks, 0>, RISING);
}