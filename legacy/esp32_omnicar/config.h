// Configuracion y variables de cada módulo o función
#pragma once
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "pid.h"


// Timer
hw_timer_t* timer_Cfg = NULL;
#define TIMER_TICK_FREQUENCY (1000000)
#define TIMER_TRIGGER_FREQUENCY (60)


// Mando
const char* mac_consola_ps4 = "94:54:c5:2f:0a:da";
volatile bool square = false, triangle = false;
// Rango [-1, 1]
volatile double lx = 0, ly = 0;
volatile double rx = 0, ry = 0;

volatile bool l1 = false, r1 = false;
// Rango [-1, 1]
volatile double l2 = 0, r2 = 0;

#define ZONA_MUERTA 0.1


// Encoders
#define PULSOS_POR_REVOLUCION 800
#define ENCODER0_A_PIN 26
#define ENCODER0_B_PIN 25
volatile int32_t encoder0_count = 0;

#define ENCODER1_A_PIN 22
#define ENCODER1_B_PIN 23
volatile int32_t encoder1_count = 0;

#define ENCODER2_A_PIN 4
#define ENCODER2_B_PIN 13
volatile int32_t encoder2_count = 0;

#define ENCODER3_A_PIN 5
#define ENCODER3_B_PIN 18
volatile int32_t encoder3_count = 0;

/*  ^      <-- parte delantera del omnicar
  1   0
  3   2
*/
// Motores
#define PWM_FREQUENCY 1000
#define PWM_BIT_RESOLUTION 14

#define MOTOR0_LPWM_PIN 33
#define MOTOR0_RPWM_PIN 32
volatile int32_t motor0_pwm = 0;

#define MOTOR1_LPWM_PIN 19
#define MOTOR1_RPWM_PIN 21
volatile int32_t motor1_pwm = 0;

#define MOTOR2_LPWM_PIN 27
#define MOTOR2_RPWM_PIN 14
volatile int32_t motor2_pwm = 0;

#define MOTOR3_LPWM_PIN 16
#define MOTOR3_RPWM_PIN 17
volatile int32_t motor3_pwm = 0;


// PID
PID motor0_pid{ 0.3, 0.06, 0.0 };
PID motor1_pid{ 0.3, 0.06, 0.0 };
PID motor2_pid{ 0.3, 0.06, 0.0 };
PID motor3_pid{ 0.3, 0.06, 0.0 };
volatile double dt = 1.0 / (double)TIMER_TRIGGER_FREQUENCY;
// Valor medido experimentalmente, puede que dependa de la fuente y demás...
#define PWM_POR_REVOLUCION ((double)((1 << PWM_BIT_RESOLUTION) - 1) * 0.285)


// Cinematica
#define RADIO_RUEDA 5
#define LARGO_ROBOT 15
#define ANCHO_ROBOT 10


// Control
// Vx, Vy, Wz
BLA::Matrix<3, 1, double> velocidad_carro_actual = { 0 };
BLA::Matrix<3, 1, double> velocidad_carro_objetivo = { 0 };
// Px, Py, Wz
// No implementado
BLA::Matrix<3, 1, double> posicion_carro_actual = { 0 };
BLA::Matrix<3, 1, double>* posicion_carro_objetivo = NULL;
SemaphoreHandle_t mutex_pos_carro = NULL;


// Mapea valores de un rango a otro rango
double fmap(double x, double in_min, double in_max, double out_min, double out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define LARGO_COLA_NAVEGACION 100
QueueHandle_t cola_navegacion = NULL;
#define VELOCIDAD_NAVEGACION 20.0

// MODOS DE OPERACIÓN
enum MODO {
    // Rojo
    QUIETO,
    // Verde
    MANDO_SET_RUTA,
    // Azul
    MANDO_MANUAL
};
MODO modo_actual = QUIETO;
