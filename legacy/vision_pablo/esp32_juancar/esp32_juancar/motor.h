#pragma once
#include "config.h"


// Configura un motor y lo deja detenido. Primero se ponen los pines en LOW:
// los IBT_2 dejan sus entradas en ALTO si nadie las maneja, y eso puede hacer
// girar una rueda sola.
void motor_iniciar(int lpwm_pin, int rpwm_pin) {
  pinMode(lpwm_pin, OUTPUT);
  pinMode(rpwm_pin, OUTPUT);
  digitalWrite(lpwm_pin, LOW);
  digitalWrite(rpwm_pin, LOW);
  ledcAttach(lpwm_pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
  ledcAttach(rpwm_pin, PWM_FREQUENCY, PWM_BIT_RESOLUTION);
  ledcWrite(lpwm_pin, 0);
  ledcWrite(rpwm_pin, 0);
}

// Habilita o deshabilita los 4 puentes H a la vez (R_EN y L_EN juntos).
void motores_habilitar(bool habilitar) {
  digitalWrite(MOTORES_R_EN_PIN, habilitar ? HIGH : LOW);
  digitalWrite(MOTORES_L_EN_PIN, habilitar ? HIGH : LOW);
}

void motor_begin() {
  // Lo primero: puentes H deshabilitados.
  pinMode(MOTORES_R_EN_PIN, OUTPUT);
  pinMode(MOTORES_L_EN_PIN, OUTPUT);
  motores_habilitar(false);
  motor_iniciar(MOTOR0_LPWM_PIN, MOTOR0_RPWM_PIN);
  motor_iniciar(MOTOR1_LPWM_PIN, MOTOR1_RPWM_PIN);
  motor_iniciar(MOTOR2_LPWM_PIN, MOTOR2_RPWM_PIN);
  motor_iniciar(MOTOR3_LPWM_PIN, MOTOR3_RPWM_PIN);
}
