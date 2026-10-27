#pragma once
#include "config.h"

// ISR de los encoders. La sección crítica evita que se pierda un pulso justo
// cuando el control lee y pone en cero el contador. (No se usa un template:
// con IRAM_ATTR, el enlazador de la ESP32 falla con "dangerous relocation".)
void IRAM_ATTR encoder_contar(int b_pin, volatile int32_t& enc_count) {
  portENTER_CRITICAL_ISR(&mux_encoders);
  if (digitalRead(b_pin))
    enc_count = enc_count + 1;
  else
    enc_count = enc_count - 1;
  portEXIT_CRITICAL_ISR(&mux_encoders);
}
void IRAM_ATTR encoder0_ISR() { encoder_contar(ENCODER0_B_PIN, encoder0_count); }
void IRAM_ATTR encoder1_ISR() { encoder_contar(ENCODER1_B_PIN, encoder1_count); }
void IRAM_ATTR encoder2_ISR() { encoder_contar(ENCODER2_B_PIN, encoder2_count); }
void IRAM_ATTR encoder3_ISR() { encoder_contar(ENCODER3_B_PIN, encoder3_count); }

// Lee el contador y lo deja en cero, de una sola vez.
int32_t encoder_leer_y_reiniciar(volatile int32_t& enc_count) {
  portENTER_CRITICAL(&mux_encoders);
  int32_t pulsos = enc_count;
  enc_count = 0;
  portEXIT_CRITICAL(&mux_encoders);
  return pulsos;
}

void encoder_begin() {
  pinMode(ENCODER0_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER0_B_PIN, INPUT_PULLUP);
  attachInterrupt(
    ENCODER0_A_PIN,
    encoder0_ISR,
    RISING);

  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  attachInterrupt(
    ENCODER1_A_PIN,
    encoder1_ISR,
    RISING);

  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  attachInterrupt(
    ENCODER2_A_PIN,
    encoder2_ISR,
    RISING);

  pinMode(ENCODER3_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER3_B_PIN, INPUT_PULLUP);
  attachInterrupt(
    ENCODER3_A_PIN,
    encoder3_ISR,
    RISING);
}
