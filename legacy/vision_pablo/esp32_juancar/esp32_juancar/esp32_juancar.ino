// El control ya NO corre dentro de la interrupción de un timer, sino en una
// tarea de FreeRTOS (timer.h): ahí se pueden usar float/double, semáforos,
// colas, new/delete y ledcWrite sin problemas. Las únicas interrupciones son
// las de los encoders (encoder.h), que solo suman o restan un contador.

// Sin mando: las órdenes llegan por serial (115200) desde la Raspberry Pi,
// que decide con la cámara. Cada comando responde "DONE" o "ERR <motivo>".
//   v <adelante> <lado> <giro>   velocidad (modo manual), cada valor en [-1, 1]
//                      (1 = VELOCIDAD_NAVEGACION cm/s; los signos hay que
//                      verificarlos en la base: qué lado es "positivo")
//   p <x> <y> <th>     agregar un punto a la ruta (cm, cm, rad) y pasar a modo ruta
//   q                  quieto (frena y borra la posición)
//   r                  reiniciar la posición a (0, 0, 0) y vaciar la ruta
//   ?                  estado (posición y velocidad)
// Seguridad: en modo manual, si pasan TIEMPO_SIN_ORDENES_MS sin un comando
// "v", el carro frena solo (por ejemplo, si la Pi se cuelga o se desconecta).

#include "timer.h"
#include "encoder.h"
#include "motor.h"
#include "cinematica.h"
#include "control.h"

#define TIEMPO_SIN_ORDENES_MS 500

String linea;
unsigned long ultima_orden_ms = 0;

void imprimir_estado() {
  Serial.printf("vel: vx=%lf vy=%lf wz=%lf | pos: x=%lf y=%lf th=%lf\n",
                velocidad_carro_actual(0), velocidad_carro_actual(1), velocidad_carro_actual(2),
                posicion_carro_actual(0), posicion_carro_actual(1), posicion_carro_actual(2));
}

void procesar(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  double a, b, c;
  if (sscanf(cmd.c_str(), "v %lf %lf %lf", &a, &b, &c) == 3) {
    // Como en el mando: ly (palanca vertical) = avanzar, lx (horizontal) = de
    // lado, rx = girar. control.h usa set_velocidad({ vely, velx, velw }).
    ly = constrain(a, -1.0, 1.0);
    lx = constrain(b, -1.0, 1.0);
    rx = constrain(c, -1.0, 1.0);
    modo_actual = MANDO_MANUAL;
    ultima_orden_ms = millis();
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "p %lf %lf %lf", &a, &b, &c) == 3) {
    double punto[3] = { a, b, c };
    if (xQueueSend(cola_navegacion, punto, 0) != pdTRUE) {
      Serial.println("ERR ruta llena");
      return;
    }
    modo_actual = MANDO_SET_RUTA;
    Serial.println("DONE");
  } else if (cmd == "q") {
    lx = ly = rx = 0;
    modo_actual = QUIETO;
    Serial.println("DONE");
  } else if (cmd == "r") {
    // La tarea de control también usa la posición: tomar el mutex.
    xSemaphoreTake(mutex_pos_carro, portMAX_DELAY);
    double p[3];
    while (xQueueReceive(cola_navegacion, p, 0) == pdTRUE);
    posicion_carro_actual = { 0, 0, 0 };
    velocidad_carro_actual = { 0, 0, 0 };
    xSemaphoreGive(mutex_pos_carro);
    Serial.println("DONE");
  } else if (cmd == "?") {
    xSemaphoreTake(mutex_pos_carro, portMAX_DELAY);
    imprimir_estado();
    xSemaphoreGive(mutex_pos_carro);
    Serial.println("DONE");
  } else {
    Serial.print("ERR comando no reconocido: ");
    Serial.println(cmd);
  }
}

void setup() {
  Serial.begin(115200);
  motor_begin();    // primero: dejar los 4 motores detenidos
  encoder_begin();
  control_begin();  // crea el mutex y la cola que usa la tarea de control
  timer_begin();    // al final: arranca la tarea de control
  Serial.println("Listo. Comandos: v, p, q, r, ?");

  // prueba_manual();
}

void loop() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') {
      procesar(linea);
      linea = "";
    } else if (ch != '\r') {
      linea += ch;
    }
  }

  // Seguridad: sin órdenes recientes en modo manual, frenar.
  if (modo_actual == MANDO_MANUAL && millis() - ultima_orden_ms > TIEMPO_SIN_ORDENES_MS) {
    lx = ly = rx = 0;
    modo_actual = QUIETO;
    Serial.println("aviso: sin ordenes, frenado");
  }
  delay(5);
}
