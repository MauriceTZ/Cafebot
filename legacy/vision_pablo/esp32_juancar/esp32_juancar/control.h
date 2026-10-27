#pragma once
#include "config.h"
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "pid.h"
#include "cinematica.h"
#include "encoder.h"
#include "motor.h"

/*
  Setear velocidad (cm/seg)
*/
void set_velocidad(BLA::Matrix<3, 1, double> v) {
  // Serial.printf("set_velocidad({%lf, %lf, %lf})\n", v(0), v(1), v(2));
  velocidad_carro_objetivo = v;
  BLA::Matrix<2, 1, double> velocidad_sin_rotar = { v(0), v(1) };
  double theta = -posicion_carro_actual(2);
  BLA::Matrix<2, 2, double> rot = { cos(theta), -sin(theta),
                                    sin(theta), cos(theta) };
  BLA::Matrix<2, 1, double> velocidad_rotada = rot * velocidad_sin_rotar;
  v(0) = velocidad_rotada(0);
  v(1) = velocidad_rotada(1);
  BLA::Matrix<4, 1, double> result = mat_cinematica_directa * v;
  // Serial.println(result);
  motor0_pid.set_punto_referencia(result(0) / TWO_PI);
  motor1_pid.set_punto_referencia(result(1) / TWO_PI);
  motor2_pid.set_punto_referencia(result(2) / TWO_PI);
  motor3_pid.set_punto_referencia(result(3) / TWO_PI);
}

// El PID se recibe por REFERENCIA (&): si se pasa por copia, su integral y su
// error anterior se pierden en cada ciclo.
double control_motor(volatile int32_t& encoder,
                     PID& motor_pid,
                     volatile int32_t& motor_pwm,
                     int motor_lpwm_pin,
                     int motor_rpwm_pin) {
  // Calcular revoluciones por segundo
  int32_t pulsos = encoder_leer_y_reiniciar(encoder);
  double revolucion_seg = (double)TIMER_TRIGGER_FREQUENCY * (double)pulsos / PULSOS_POR_REVOLUCION;
  // Corrección por control PID (en rev/s)
  double revolucion_seg_pid_salida = motor_pid.control(revolucion_seg, dt);
  // PWM = lo que necesita la velocidad pedida (prealimentación, con el valor
  // medido PWM_POR_REVOLUCION) + la corrección del PID. Ya NO se acumula con
  // "+=": el PID tiene su propia integral, y acumular encima integraba dos veces.
  double rev_seg_total = motor_pid.get_punto_referencia() + revolucion_seg_pid_salida;
  double pwm = fmap(rev_seg_total, 0, 1, 0, PWM_POR_REVOLUCION);
  double referencia = motor_pid.get_punto_referencia();
  if (referencia > 0.01) pwm += PWM_ARRANQUE;
  else if (referencia < -0.01) pwm -= PWM_ARRANQUE;
  motor_pwm = (int32_t)pwm;
  // El valor PWM no puede salir del rango
  motor_pwm = constrain(motor_pwm, -((1 << PWM_BIT_RESOLUTION) - 1), (1 << PWM_BIT_RESOLUTION) - 1);
  if (motor_pwm >= 0) {
    ledcWrite(motor_rpwm_pin, 0);
    ledcWrite(motor_lpwm_pin, motor_pwm);
  } else {
    ledcWrite(motor_lpwm_pin, 0);
    ledcWrite(motor_rpwm_pin, -motor_pwm);
  }
  return revolucion_seg;
}

// Se llama desde la tarea de control (timer.h) con mutex_pos_carro tomado.
void control_carro() {
  switch (modo_actual) {
    case QUIETO:
      {
        encoder_leer_y_reiniciar(encoder0_count);
        encoder_leer_y_reiniciar(encoder1_count);
        encoder_leer_y_reiniciar(encoder2_count);
        encoder_leer_y_reiniciar(encoder3_count);
        motor0_pid.reiniciar();
        motor1_pid.reiniciar();
        motor2_pid.reiniciar();
        motor3_pid.reiniciar();
        motor0_pid.set_punto_referencia(0);
        motor1_pid.set_punto_referencia(0);
        motor2_pid.set_punto_referencia(0);
        motor3_pid.set_punto_referencia(0);
        motor0_pwm = motor1_pwm = motor2_pwm = motor3_pwm = 0;
        motores_habilitar(false);  // puentes H en reposo
        posicion_carro_actual = { 0, 0, 0 };
        velocidad_carro_actual = { 0, 0, 0 };

        ledcWrite(MOTOR0_LPWM_PIN, 0);
        ledcWrite(MOTOR0_RPWM_PIN, 0);
        ledcWrite(MOTOR1_LPWM_PIN, 0);
        ledcWrite(MOTOR1_RPWM_PIN, 0);
        ledcWrite(MOTOR2_LPWM_PIN, 0);
        ledcWrite(MOTOR2_RPWM_PIN, 0);
        ledcWrite(MOTOR3_LPWM_PIN, 0);
        ledcWrite(MOTOR3_RPWM_PIN, 0);
        return;
      }
      break;
    case MANDO_MANUAL:
      {
        double velx = fmap(lx, -1, 1, -VELOCIDAD_NAVEGACION, VELOCIDAD_NAVEGACION);
        double vely = fmap(ly, -1, 1, -VELOCIDAD_NAVEGACION, VELOCIDAD_NAVEGACION);
        double velw = fmap(rx, -1, 1, -1, 1);
        set_velocidad({ vely, velx, velw });
      }
      break;
    case MANDO_SET_RUTA:
      {
        // El mutex ya lo tomó la tarea de control (un mutex de FreeRTOS no se
        // puede tomar dos veces seguidas: se quedaría esperando para siempre).
        if (posicion_carro_objetivo != NULL) {  // Navegación en progreso
          BLA::Matrix<3, 1, double> dir = posicion_carro_actual - (*posicion_carro_objetivo);
          double distance = sqrt(dir(0) * dir(0) + dir(1) * dir(1));
          double angle = dir(2);
          if (distance < 1.0 && abs(degrees(angle)) < 5.0) {  // Llegó al punto
            delete posicion_carro_objetivo;
            posicion_carro_objetivo = NULL;
            set_velocidad({ 0, 0, 0 });
          } else {
            if (distance > 2.0) {
              dir = VELOCIDAD_NAVEGACION * dir * (1.0 / distance);
            }
            dir(0) *= -1;
            dir(2) = constrain(-angle, -1, 1);
            set_velocidad(dir);
          }
        } else {  // Revisar si hay nuevo punto
          double nuevo_punto[3];
          if (xQueueReceive(cola_navegacion, nuevo_punto, 0)) {
            posicion_carro_objetivo = new BLA::Matrix<3, 1, double>({ nuevo_punto[0],
                                                                      nuevo_punto[1],
                                                                      nuevo_punto[2] });
          }
        }
      }
      break;
  }


  motores_habilitar(true);  // en los modos que mueven el carro

  // Serial.printf("encoder0: %d encoder1: %d encoder2: %d encoder3: %d\n", encoder0_count, encoder1_count, encoder2_count, encoder3_count);
  double motor0_revol = control_motor(encoder0_count, motor0_pid, motor0_pwm, MOTOR0_LPWM_PIN, MOTOR0_RPWM_PIN);
  double motor1_revol = control_motor(encoder1_count, motor1_pid, motor1_pwm, MOTOR1_LPWM_PIN, MOTOR1_RPWM_PIN);
  double motor2_revol = control_motor(encoder2_count, motor2_pid, motor2_pwm, MOTOR2_LPWM_PIN, MOTOR2_RPWM_PIN);
  double motor3_revol = control_motor(encoder3_count, motor3_pid, motor3_pwm, MOTOR3_LPWM_PIN, MOTOR3_RPWM_PIN);

  // Pasar de revoluciones por segundo a Radianes por segundo de cada rueda
  double motor0_rad = fmap(motor0_revol, 0, 1, 0, TWO_PI);
  double motor1_rad = fmap(motor1_revol, 0, 1, 0, TWO_PI);
  double motor2_rad = fmap(motor2_revol, 0, 1, 0, TWO_PI);
  double motor3_rad = fmap(motor3_revol, 0, 1, 0, TWO_PI);

  // Calcular velocidad actual del carro, teniendo en cuenta la rotación...
  BLA::Matrix<4, 1, double> w = { motor0_rad, motor1_rad, motor2_rad, motor3_rad };
  BLA::Matrix<3, 1, double> vel_result = mat_cinematica_inversa * w;
  BLA::Matrix<2, 1, double> vel_result_sin_rotar = { vel_result(0), vel_result(1) };
  double theta = posicion_carro_actual(2);
  BLA::Matrix<2, 2, double> rot = { cos(theta), -sin(theta),
                                    sin(theta), cos(theta) };
  BLA::Matrix<2, 1, double> vel_result_rotada = rot * vel_result_sin_rotar;
  velocidad_carro_actual(0) = vel_result_rotada(0);
  velocidad_carro_actual(1) = -vel_result_rotada(1);
  velocidad_carro_actual(2) = vel_result(2);

  // Actualizar posición actual
  posicion_carro_actual += velocidad_carro_actual * dt;
}

void control_begin() {
  mutex_pos_carro = xSemaphoreCreateMutex();
  cola_navegacion = xQueueCreate(LARGO_COLA_NAVEGACION, sizeof(double) * 3);
  
  // double cola_prueba[][3] = {
  //   { 10, 0, 0 },
  //   { 10, 10, 0 },
  //   { 10, 10, HALF_PI },
  //   { 0, 0, HALF_PI },
  //   { 0, 0, 0 },
  // };
  // xQueueSend(cola_navegacion, cola_prueba[0], 0);
  // xQueueSend(cola_navegacion, cola_prueba[1], 0);
  // xQueueSend(cola_navegacion, cola_prueba[2], 0);
  // xQueueSend(cola_navegacion, cola_prueba[3], 0);
  // xQueueSend(cola_navegacion, cola_prueba[4], 0);
}

void prueba_manual() {
  set_velocidad({ 0, 8, 0 });
  delay(2000);
  set_velocidad({ 0, 0, 0 });
  delay(1000);

  set_velocidad({ 8, 0, 0 });
  delay(2000);
  set_velocidad({ 0, 0, 0 });
  delay(1000);

  set_velocidad({ -8, -8, 0 });
  delay(2000);
  set_velocidad({ 0, 0, 0 });
  delay(1000);
}
