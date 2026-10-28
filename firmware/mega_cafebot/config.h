#pragma once
// =====================================================================
//  Cafebot - firmware Arduino Mega 2560 + shield 4Motor&9Servo
//  config.h : pines, dimensiones y constantes (UN solo lugar para ajustar)
//
//  Las secciones 1 y 2 vienen tal cual de Mega_Cafebot v0 (pines y medidas).
//  Lo marcado con  [CONFIRMAR]  es un supuesto que hay que verificar en el robot.
// =====================================================================
#include <Arduino.h>

// ---------------------------------------------------------------------
// 1. PINOUT (placa Moebius 4Motor & 9Servo)  -- sin cambios respecto a v0
// ---------------------------------------------------------------------
// Ubicación inferida de la cinemática de giro de v0 (M1/M4 +ang, M2/M3 -ang):
//   M1 = delantera derecha   M2 = delantera izquierda
//   M3 = trasera derecha     M4 = trasera izquierda          [CONFIRMAR]
#define M1_ENC_A 18
#define M1_ENC_B 31
#define M1_PWM 12
#define M1_DIR1 34
#define M1_DIR2 35
#define M1_SERVO_PIN 6      // 27 oct, verificado con Y: el pin 6 mueve la delantera DERECHA
#define M1_SERVO_OFFSET 89

#define M2_ENC_A 19
#define M2_ENC_B 38
#define M2_PWM 8
#define M2_DIR1 37
#define M2_DIR2 36
#define M2_SERVO_PIN 10
#define M2_SERVO_OFFSET 87

#define M3_ENC_A 3
#define M3_ENC_B 49
#define M3_PWM 9
#define M3_DIR1 43
#define M3_DIR2 42
#define M3_SERVO_PIN 13     // y el pin 13 la trasera DERECHA (en v0 estaban cruzados)
#define M3_SERVO_OFFSET 87

#define M4_ENC_A 2
#define M4_ENC_B A1
#define M4_PWM 5
#define M4_DIR1 A4
#define M4_DIR2 A5
#define M4_SERVO_PIN 45
#define M4_SERVO_OFFSET 90

// Polaridad (de v0): M2 tiene el motor invertido; encoders M1 y M3 invertidos.
#define M1_INV_MOTOR false
#define M2_INV_MOTOR true
#define M3_INV_MOTOR false
#define M4_INV_MOTOR false
#define M1_INV_ENC 1
#define M2_INV_ENC 0
#define M3_INV_ENC 1
#define M4_INV_ENC 0

// ---------------------------------------------------------------------
// 1b. SENSORES INFRARROJOS DE BORDE (nuevo)                 [CONFIRMAR pines]
// ---------------------------------------------------------------------
// Delanteros: digitales con umbral por trimmer. Traseros: analógicos (umbral por software).
// No necesitan interrupciones: se leen en el loop a ~1 kHz (a 200 mm/s el robot
// avanza 0,2 mm entre lecturas; la línea mide 19 mm).
#define IR_DI_PIN 22   // delantero izquierdo  (digital)  [CONFIRMAR]
#define IR_DD_PIN 23   // delantero derecho    (digital)  [CONFIRMAR]
#define IR_TI_PIN A8   // trasero izquierdo    (analógico) [CONFIRMAR]
#define IR_TD_PIN A9   // trasero derecho      (analógico) [CONFIRMAR]
// Nivel que entrega el sensor digital al ver NEGRO (la mayoría de módulos TCRT5000
// dan HIGH sobre negro porque no hay reflexión).                 [CONFIRMAR]
#define IR_DIGITAL_NEGRO_ES_HIGH 1
#define IR_PERIODO_US 1000

// ---------------------------------------------------------------------
// 1c. SERVOS AUXILIARES (compuertas de los depósitos)        [CONFIRMAR pines]
// ---------------------------------------------------------------------
// Comando "X k ángulo". La librería Servo del Mega usa el Timer5: los pines 44/45/46
// pierden su PWM, pero sirven igual como salida de servo.
#define N_SERVOS_AUX 3
// 0 = compuerta maduros (roja), 1 = compuerta sobremaduros (azul), 2 = inclinación de la cámara
const uint8_t PIN_SERVO_AUX[N_SERVOS_AUX] = { 44, 46, 11 };
const uint8_t AUX_INICIAL_DEG[N_SERVOS_AUX] = { 90, 90, 90 };  // compuertas cerradas, cámara horizontal [AJUSTAR]

// ---------------------------------------------------------------------
// 2. PARÁMETROS CINEMÁTICOS  -- sin cambios respecto a v0
// ---------------------------------------------------------------------
#define TIMER_TRIGGER_FREQUENCY 60.0f
const float dt = 1.0f / TIMER_TRIGGER_FREQUENCY;

#define PULSOS_POR_REVOLUCION 800.0f   // contando SOLO flancos de subida de A (como v0)
#define CINEMATICA_W 217.0f              // Ancho entre ruedas (mm)
#define CINEMATICA_L 223.0f              // Largo entre ejes (mm)
// Diámetro EFECTIVO, calibrado en el piso (incluye la compresión del neumático y el error de pulsos/vuelta):
//   27 oct: A 1000 -> odometría 997 mm, huincha 955 mm => 53 * 955/997 = 50,8 mm.
//   28 oct, ruedas cambiadas: A 1000 (odometría ~1000 mm) -> traseras 1022 mm (izq) y 1050 mm (der),
//           promedio 1036 mm => 50,8 * 1,036 = 52,6 mm. La diferencia izq/der está por revisar (docs/hardware.md).
#define CINEMATICA_DIAMETRO_RUEDA 52.6f  // mm
const float CIRCUNFERENCIA_RUEDA = PI * CINEMATICA_DIAMETRO_RUEDA;
// Corrección POR RUEDA (multiplica el diámetro de arriba). Orden M1 del-der, M2 del-izq, M3 tras-der, M4 tras-izq.
// 28 oct: A 1000 recorrió 1022 mm atrás-izq y 1050 mm atrás-der (promedio 1036) y se desvió ~6 cm a la
// izquierda => der 1050/1036 = 1,0135, izq 1022/1036 = 0,9865. Las delanteras se asumen iguales a las de
// su lado [CONFIRMAR]. Se cambian EN VIVO con "EK i factor" (queda en EEPROM, que manda sobre estos valores).
const float ESCALA_RUEDA_DEF[4] = { 1.0135f, 0.9865f, 1.0135f, 0.9865f };
const float CINEMATICA_D_GIRO_Z = sqrt((CINEMATICA_W * CINEMATICA_W) + (CINEMATICA_L * CINEMATICA_L));
const float ANGULO_SERVOS_Z = atan2(CINEMATICA_L, CINEMATICA_W) * 180.0f / PI;

// Decodificación del encoder (se puede cambiar EN VIVO con "EM 1" / "EM 2" y queda en EEPROM):
//   x1 = solo flanco de subida de A (como v0, 800 cuentas/vuelta de rueda)
//   x2 = ambos flancos de A (1600 cuentas/vuelta). Solo es válido si A y B están
//        realmente desfasados 90° (cuadratura). Verificar con "ED" (docs/hardware.md).
#define ENCODER_X2 0   // x1 por defecto hasta verificar la cuadratura en el robot

// ---------------------------------------------------------------------
// 3. SERVOS DE DIRECCIÓN
// ---------------------------------------------------------------------
// v0 tardaba 1500 ms + 100 ms por cada cambio de postura. Ahora el tiempo es
// proporcional al ángulo a recorrer (un servo típico gira 60° en ~0,15 s).
#define SERVO_MS_POR_GRADO 8.0f        // 45° -> 360 ms           [AJUSTAR]
#define SERVO_MS_MINIMO 150
#define SERVO_ASENTAMIENTO_MS 80
// Modo "cangrejo": las 4 ruedas al mismo ángulo => traslación sin girar.
// Signo por servo para que +ángulo = rueda girada hacia la IZQUIERDA en las 4.
// PASO 1 - SIGNO_SERVO: dice hacia dónde gira CADA servo. Se ajusta con "L 45 200":
// vistas desde arriba (frente arriba), las 4 ruedas deben quedar PARALELAS con la punta
// delantera hacia la izquierda ( \ \ ). La que quede en la otra diagonal ( / ) -> cambiar su signo.
// (A ±90° no se distingue: una rueda a +90 y otra a -90 quedan en la misma línea.)
#define CANGREJO_MAX_DEG SERVO_ANG_CAL_DEG
const int8_t SIGNO_SERVO[4] = { +1, +1, +1, +1 };  // 27 oct: deducido de las pruebas W 90 / L 45 [CONFIRMAR con L 45 200]

// PASO 2 - ESQUINA de cada rueda, vista desde arriba con el robot mirando hacia adelante.
//   ESQUINA_X: +1 = delantera, -1 = trasera.   ESQUINA_Y: +1 = izquierda, -1 = derecha.
// De esto salen el rombo del giro en el sitio y qué ruedas son del lado derecho/izquierdo.
// Verificado 27 oct en el robot: M1 = del.der, M2 = del.izq, M3 = tras.der, M4 = tras.izq.
const int8_t ESQUINA_X[4] = { +1, +1, -1, -1 };
const int8_t ESQUINA_Y[4] = { -1, +1, -1, +1 };
// Si con W 90 las ruedas quedan apuntando al centro (forma de X) en vez de rombo, -1.
#define SIGNO_GIRO +1
// Calibración fina en MICROSEGUNDOS (writeMicroseconds), no en grados enteros:
//   - El HS-311 acepta ~575-2460 µs; la librería Servo por defecto solo usa 544-2400 µs
//     y Servo.write() trabaja en grados enteros (≈10 µs). Con offsets de 87° la rueda
//     NO podía llegar a -90°: write(-3) se recortaba a 0.
//   - Cada servo tiene 3 puntos medidos (-90°, 0°, +90°) e interpolación lineal por tramo,
//     así se corrige el desfase del estriado (spline) y la no-linealidad de cada servo.
// Valores por defecto = equivalentes a v0 (offset en grados). Calibrar con los comandos
// Y / YS / YG (docs/hardware.md) y quedan guardados en EEPROM.
#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500
#define SERVO_US_POR_GRADO 10.311f   // (2400-544)/180, el mapeo de Servo.write()
// Ángulo al que se calibran los puntos extremos de la tabla. Si los servos no llegan
// a 90° exactos, calibrar las 4 ruedas paralelas a un ángulo común MEDIDO (p. ej. 88)
// y poner aquí ese valor: la odometría usa el ángulo real.
#define SERVO_ANG_CAL_DEG 90.0f

// ---------------------------------------------------------------------
// 4. CONTROL DE VELOCIDAD POR RUEDA (PI + prealimentación)
// ---------------------------------------------------------------------
// pwm = KFF*v_ref + PWM_ARRANQUE*sign(v_ref) + KP*e + KI*integral(e)
// Unidades: v en mm/s, e en mm/s. Ajustables en vivo con el comando J.
#define PWM_MAX 255
#define VEL_KFF 0.44f        // PWM por mm/s (de v0: 255*0.285 PWM por rev/s)
#define VEL_KP 0.60f
#define VEL_KI 4.00f
#define PWM_ARRANQUE 20      // vence la fricción estática       [AJUSTAR]
#define ACEL_MAX_MM_S2 400.0f  // rampa de la consigna (evita patinar)
#define FILTRO_VEL_ALFA 0.5f // pasa-bajos de la velocidad medida (0..1)

// ---------------------------------------------------------------------
// 5. NAVEGACIÓN / PRIMITIVAS
// ---------------------------------------------------------------------
#define VELOCIDAD_NAVEGACION 150.0f  // mm/s por defecto
#define VELOCIDAD_MINIMA 40.0f       // mm/s cerca del objetivo
#define VELOCIDAD_ANGULAR_MAX 0.8f   // rad/s
#define VELOCIDAD_ANGULAR_MIN 0.20f  // rad/s
#define KP_DISTANCIA 1.5f            // (mm/s)/mm
#define KP_ANGULO 2.2f               // (rad/s)/rad
#define KP_RUMBO 2.0f                // corrección de rumbo al avanzar recto
#define TOLERANCIA_DISTANCIA_MM 2.0f
#define TOLERANCIA_ANGULO_RAD 0.007f  // ~0,4°
#define UMBRAL_DESVIO_RUMBO_RAD 0.18f  // si un waypoint se desvía >10°, re-apunta
#define WATCHDOG_VEL_MS 500          // V/K deben repetirse antes de esto
#define LARGO_COLA 24

// ---------------------------------------------------------------------
// 6. SERIAL
// ---------------------------------------------------------------------
#define SERIAL_BAUD 115200
#define LARGO_LINEA 72

inline float wrapAngle(float a) {
    while (a > PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}
