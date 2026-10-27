#pragma once
#include "config.h"

// ISR generica para cualquier encoder
template<int b_pin, volatile int32_t& enc_count>
void encoder_interrupt_ISR() {
    if (digitalRead(b_pin))
        enc_count++;
    else
        enc_count--;
    //   Serial.printf("b_pin: %d, enc_count: %d\n", b_pin, enc_count);
}

void encoder_begin() {
    pinMode(ENCODER0_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER0_B_PIN, INPUT_PULLUP);
    attachInterrupt(
      ENCODER0_A_PIN,
      encoder_interrupt_ISR<ENCODER0_B_PIN, encoder0_count>,
      RISING);

    pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
    attachInterrupt(
      ENCODER1_A_PIN,
      encoder_interrupt_ISR<ENCODER1_B_PIN, encoder1_count>,
      RISING);

    pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
    attachInterrupt(
      ENCODER2_A_PIN,
      encoder_interrupt_ISR<ENCODER2_B_PIN, encoder2_count>,
      RISING);

    pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
    attachInterrupt(
      ENCODER3_A_PIN,
      encoder_interrupt_ISR<ENCODER3_B_PIN, encoder3_count>,
      RISING);
}