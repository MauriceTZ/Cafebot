// Configuracion y variables de cada módulo o función
#pragma once
#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
#include "pid.h"


// Control: veces por segundo que corre la tarea de control (ver timer.h).
// 50 Hz = cada 20 ms exactos (con 60 Hz el periodo no da un número entero de ms).
#define TIMER_TRIGGER_FREQUENCY (50)


// Órdenes de velocidad recibidas por serial desde la Pi (antes, del mando).
// Rango [-1, 1]
volatile double lx = 0, ly = 0;
volatile double rx = 0;


// Encoders
// Pulsos por vuelta de la RUEDA contando solo el flanco de subida del canal A.
// Verificar: girar una rueda una vuelta a mano y leer el contador.
#define PULSOS_POR_REVOLUCION 800
portMUX_TYPE mux_encoders = portMUX_INITIALIZER_UNLOCKED;
// Encoders 0 y 1: los cables quedaron cruzados en el recableado (prueba del
// 16 oct: el motor 1, delantera izquierda, cuenta en 5/18).
#define ENCODER0_A_PIN 23
#define ENCODER0_B_PIN 22
volatile int32_t encoder0_count = 0;

#define ENCODER1_A_PIN 5
#define ENCODER1_B_PIN 18
volatile int32_t encoder1_count = 0;

#define ENCODER2_A_PIN 13
#define ENCODER2_B_PIN 4
volatile int32_t encoder2_count = 0;

// Encoder 3 contaba negativo con la rueda hacia adelante -> A y B intercambiados.
#define ENCODER3_A_PIN 25
#define ENCODER3_B_PIN 26
volatile int32_t encoder3_count = 0;

/*  ^      <-- parte delantera del omnicar
  1   0
  3   2
*/
// Motores
#define PWM_FREQUENCY 1000
#define PWM_BIT_RESOLUTION 14

// Enable de los puentes H: los 4 R_EN juntos a MOTORES_R_EN_PIN y los 4 L_EN
// juntos a MOTORES_L_EN_PIN, cada pin con 10 kΩ a GND (mientras la ESP32
// arranca quedan en bajo). Se manejan siempre juntos: el motor solo gira con
// las dos mitades del puente habilitadas; el sentido lo deciden RPWM/LPWM.
// En modo QUIETO se deshabilitan (puentes H en reposo). El GPIO 2 es además el
// LED azul de la placa: se enciende con los motores habilitados.
#define MOTORES_R_EN_PIN 12
#define MOTORES_L_EN_PIN 2

#define MOTOR0_LPWM_PIN 16
#define MOTOR0_RPWM_PIN 17
volatile int32_t motor0_pwm = 0;

// Ruedas izquierdas (1 y 3): motores montados en espejo, con PWM positivo
// giraban hacia atrás (prueba 16 oct) -> se intercambian LPWM y RPWM.
#define MOTOR1_LPWM_PIN 21
#define MOTOR1_RPWM_PIN 19
volatile int32_t motor1_pwm = 0;

#define MOTOR2_LPWM_PIN 14
#define MOTOR2_RPWM_PIN 27
volatile int32_t motor2_pwm = 0;

#define MOTOR3_LPWM_PIN 33
#define MOTOR3_RPWM_PIN 32
volatile int32_t motor3_pwm = 0;


// PID
PID motor0_pid{ 0.2, 0.05, 0.0 };
PID motor1_pid{ 0.2, 0.05, 0.0 };
PID motor2_pid{ 0.2, 0.05, 0.0 };
PID motor3_pid{ 0.2, 0.05, 0.0 };
volatile double dt = 1.0 / (double)TIMER_TRIGGER_FREQUENCY;
// Valor medido experimentalmente, puede que dependa de la fuente y demás...
// Prueba 16 oct (7,5 V, ruedas en el aire): con 59 % de PWM la rueda dio ~667
// pulsos/s = 0,83 rev/s (con 800 pulsos/rev) -> ~0,7 del PWM máximo por rev/s.
// El 0,285 de antes era ~2,5 veces menor: con teleop las ruedas apenas se movían.
#define PWM_POR_REVOLUCION ((double)((1 << PWM_BIT_RESOLUTION) - 1) * 0.7)
// PWM mínimo para que el motor venza el roce y arranque (fracción del máximo).
// Se suma solo cuando se pide moverse. ESTIMADO: ajustar si da tirones.
#define PWM_ARRANQUE ((double)((1 << PWM_BIT_RESOLUTION) - 1) * 0.10)


// Cinemática — medidas de la base, en cm. ¡PROVISIONALES: MEDIR LA BASE REAL!
// Con valores malos, las distancias y los giros salen mal (no se nota en el aire).
//   RADIO_RUEDA:    radio de la rueda mecanum (diámetro / 2).
//   MITAD_LARGO:    mitad de la distancia entre el eje de las ruedas delanteras
//                   y el de las traseras (centro de rueda a centro de rueda).
//   MITAD_ANCHO:    mitad de la distancia entre el centro de la rueda izquierda
//                   y el de la derecha.
// (En el paper de cinematica.h se llaman lx y ly; antes estaban como
//  LARGO_ROBOT = 15 y ANCHO_ROBOT = 10, sin decir si eran medidas completas o mitades.)
#define RADIO_RUEDA 5.0
#define MITAD_LARGO 15.0
#define MITAD_ANCHO 10.0


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
#define VELOCIDAD_NAVEGACION 30.0

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
