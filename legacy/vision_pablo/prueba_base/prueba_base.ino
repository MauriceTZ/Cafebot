// Prueba de la base mecanum: 4 motores con drivers IBT_2 (BTS7960) y encoders.
// Se controla por serial a 115200 (Monitor Serie con "Nueva línea", o desde
// la Pi con mover.py). Cada comando se detiene solo (máximo 5 s).
//
//   m <motor 0-3> <vel> <ms>   un motor (0=DI 1=DD 2=TI 3=TD)
//   f <vel> <ms>               adelante (las 4 ruedas)
//   l <vel> <ms>               lateral izquierda (DI y TD atrás, DD y TI adelante)
//   g <vel> <ms>               girar antihorario (izquierdas atrás, derechas adelante)
//   v <vx> <vy> <w> <ms>       combinación libre (cinemática mecanum)
//   e                          mostrar y reiniciar los contadores de los encoders
//   s                          parar todo
//   ?                          ayuda
// vel: -255..255 (negativo = al revés). Respuestas: "DONE" al terminar,
// "ERR <motivo>" si el comando es inválido o fue interrumpido. Al terminar cada
// movimiento se informa cuántos pulsos contó cada encoder (positivo = adelante
// si "invertido" está bien ajustado).
//
// Cableado (el mismo del programa de Juancar, esp32_juancar/config.h):
// cada IBT_2 con RPWM y LPWM a los pines de abajo; los 4 R_EN al GPIO 12 y los
// 4 L_EN al GPIO 2, cada uno con 10 kΩ a GND; VCC lógico a 3,3 V; GND común
// con la ESP32 (obligatorio). Los BTS7960 necesitan
// al menos ~5,5 V en B+/B− para funcionar.

// ---------------------------------------------------------------- configuración
struct Motor {
  const char* nombre;
  int pinL;        // LPWM (con velocidad positiva se activa este lado)
  int pinR;        // RPWM
  int encA;        // canal A del encoder (interrupción)
  int encB;        // canal B del encoder (sentido)
  bool invertido;  // true si con velocidad positiva la rueda gira hacia atrás
  bool encInvertido;  // true si el encoder cuenta al revés que la rueda
};

// En el código de Juancar: 0=delantera derecha, 1=delantera izquierda,
// 2=trasera derecha, 3=trasera izquierda. Aquí se ordenan como DI, DD, TI, TD.
Motor motores[4] = {
  {"DI", 19, 21, 5, 18, true, false},    // Juancar motor 1 (encoder cruzado con DD: verificado 16 oct)
  {"DD", 16, 17, 23, 22, false, false},  // Juancar motor 0
  {"TI", 32, 33, 26, 25, true, true},    // Juancar motor 3
  {"TD", 14, 27, 13, 4, false, false},   // Juancar motor 2
};

// Enable de los puentes H: los 4 R_EN juntos a PIN_R_EN y los 4 L_EN juntos a
// PIN_L_EN, cada pin con una resistencia de 10 kΩ a GND (mientras la ESP32
// arranca quedan en bajo: motores deshabilitados). Se manejan SIEMPRE juntos:
// el motor solo gira con las dos mitades del puente habilitadas; el sentido lo
// deciden RPWM/LPWM. Se habilitan solo mientras hay movimiento. (El LED azul
// de la placa está en el GPIO 2: se enciende con los motores habilitados.)
const int PIN_R_EN = 12;
const int PIN_L_EN = 2;

void habilitarPuentes(bool si) {
  digitalWrite(PIN_R_EN, si ? HIGH : LOW);
  digitalWrite(PIN_L_EN, si ? HIGH : LOW);
}

const int VEL_MAX = 255;       // bajar (p. ej. 150) si los motores son muy bruscos
const unsigned long MS_MAX = 5000;
const int PWM_FREQ = 20000;    // 20 kHz: fuera del rango audible
const int PWM_BITS = 8;        // duty 0..255

// ---------------------------------------------------------------- estado
volatile int32_t cuentas[4] = {0, 0, 0, 0};
unsigned long finMovimiento = 0;
bool moviendo = false;
String linea;

// Una interrupción por encoder: en cada flanco de subida de A, B dice el sentido.
void IRAM_ATTR contarEncoder(int i) {
  if (digitalRead(motores[i].encB)) cuentas[i]++;
  else cuentas[i]--;
}
void IRAM_ATTR isrEncoder0() { contarEncoder(0); }
void IRAM_ATTR isrEncoder1() { contarEncoder(1); }
void IRAM_ATTR isrEncoder2() { contarEncoder(2); }
void IRAM_ATTR isrEncoder3() { contarEncoder(3); }

void escribirMotor(int i, int vel) {
  vel = constrain(vel, -VEL_MAX, VEL_MAX);
  if (motores[i].invertido) vel = -vel;
  if (vel >= 0) {
    ledcWrite(motores[i].pinR, 0);
    ledcWrite(motores[i].pinL, vel);
  } else {
    ledcWrite(motores[i].pinL, 0);
    ledcWrite(motores[i].pinR, -vel);
  }
}

void pararTodo() {
  for (int i = 0; i < 4; i++) escribirMotor(i, 0);
  habilitarPuentes(false);  // puentes H en reposo
  moviendo = false;
}

// Lee y pone en cero los contadores (sin perder pulsos a medio camino).
void leerCuentas(int32_t salida[4]) {
  noInterrupts();
  for (int i = 0; i < 4; i++) {
    salida[i] = cuentas[i];
    cuentas[i] = 0;
  }
  interrupts();
  for (int i = 0; i < 4; i++)
    if (motores[i].encInvertido) salida[i] = -salida[i];
}

void informarEncoders() {
  int32_t c[4];
  leerCuentas(c);
  Serial.printf("encoders DI=%ld DD=%ld TI=%ld TD=%ld\n",
                (long)c[0], (long)c[1], (long)c[2], (long)c[3]);
}

// Velocidades en orden DI, DD, TI, TD.
void aplicar(int v[4], unsigned long ms) {
  if (moviendo) Serial.println("ERR interrumpido");
  int32_t basura[4];
  leerCuentas(basura);  // empezar a contar desde cero
  for (int i = 0; i < 4; i++) escribirMotor(i, v[i]);
  habilitarPuentes(true);
  finMovimiento = millis() + min(ms, MS_MAX);
  moviendo = true;
  Serial.printf("moviendo DI=%d DD=%d TI=%d TD=%d por %lu ms\n",
                v[0], v[1], v[2], v[3], min(ms, MS_MAX));
}

// Cinemática mecanum (vx adelante, vy izquierda, w antihorario).
// Si alguna rueda supera VEL_MAX, se escalan todas para mantener la dirección.
void mecanum(float vx, float vy, float w, unsigned long ms) {
  float r[4] = {vx - vy - w, vx + vy + w, vx + vy - w, vx - vy + w};
  float mayor = 0;
  for (int i = 0; i < 4; i++) mayor = max(mayor, fabsf(r[i]));
  float escala = mayor > VEL_MAX ? VEL_MAX / mayor : 1.0f;
  int v[4];
  for (int i = 0; i < 4; i++) v[i] = (int)lroundf(r[i] * escala);
  aplicar(v, ms);
}

void ayuda() {
  Serial.println("Comandos (vel -255..255, ms hasta 5000):");
  Serial.println("  m <motor 0-3> <vel> <ms>   0=DI 1=DD 2=TI 3=TD");
  Serial.println("  f <vel> <ms>   adelante");
  Serial.println("  l <vel> <ms>   lateral izquierda");
  Serial.println("  g <vel> <ms>   girar antihorario");
  Serial.println("  v <vx> <vy> <w> <ms>");
  Serial.println("  e   encoders  s   parar     ?   ayuda");
  for (int i = 0; i < 4; i++)
    Serial.printf("  motor %d %s: LPWM=%d RPWM=%d enc A=%d B=%d invertido=%s\n", i,
                  motores[i].nombre, motores[i].pinL, motores[i].pinR,
                  motores[i].encA, motores[i].encB, motores[i].invertido ? "si" : "no");
}

void procesar(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  const char* c = cmd.c_str();
  int a, b, d, e;
  if (cmd == "s") {
    if (moviendo) Serial.println("ERR interrumpido");
    pararTodo();
    Serial.println("DONE");
  } else if (cmd == "?") {
    ayuda();
    Serial.println("DONE");
  } else if (cmd == "e") {
    informarEncoders();
    Serial.println("DONE");
  } else if (sscanf(c, "m %d %d %d", &a, &b, &d) == 3) {
    if (a < 0 || a > 3) { Serial.println("ERR motor debe ser 0-3"); return; }
    int v[4] = {0, 0, 0, 0};
    v[a] = b;
    aplicar(v, d);
  } else if (sscanf(c, "f %d %d", &a, &b) == 2) {
    mecanum(a, 0, 0, b);
  } else if (sscanf(c, "l %d %d", &a, &b) == 2) {
    mecanum(0, a, 0, b);
  } else if (sscanf(c, "g %d %d", &a, &b) == 2) {
    mecanum(0, 0, a, b);
  } else if (sscanf(c, "v %d %d %d %d", &a, &b, &d, &e) == 4) {
    mecanum(a, b, d, e);
  } else {
    Serial.print("ERR comando no reconocido: ");
    Serial.println(cmd);
  }
}

void setup() {
  // Lo primero: puentes H deshabilitados, y las salidas de los motores en 0
  // (los IBT_2 dejan sus entradas en ALTO si nadie las maneja).
  pinMode(PIN_R_EN, OUTPUT);
  pinMode(PIN_L_EN, OUTPUT);
  habilitarPuentes(false);
  for (int i = 0; i < 4; i++) {
    pinMode(motores[i].pinR, OUTPUT);
    pinMode(motores[i].pinL, OUTPUT);
    digitalWrite(motores[i].pinR, LOW);
    digitalWrite(motores[i].pinL, LOW);
    ledcAttach(motores[i].pinR, PWM_FREQ, PWM_BITS);
    ledcAttach(motores[i].pinL, PWM_FREQ, PWM_BITS);
  }
  pararTodo();

  // Encoders: solo entradas, nunca salidas.
  for (int i = 0; i < 4; i++) {
    pinMode(motores[i].encA, INPUT_PULLUP);
    pinMode(motores[i].encB, INPUT_PULLUP);
  }
  attachInterrupt(motores[0].encA, isrEncoder0, RISING);
  attachInterrupt(motores[1].encA, isrEncoder1, RISING);
  attachInterrupt(motores[2].encA, isrEncoder2, RISING);
  attachInterrupt(motores[3].encA, isrEncoder3, RISING);

  Serial.begin(115200);
  Serial.println("prueba_base lista. Escribe ? para ayuda.");
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
  if (moviendo && (long)(millis() - finMovimiento) >= 0) {
    pararTodo();
    delay(300);  // dejar que la rueda se detenga para contar todos los pulsos
    informarEncoders();
    Serial.println("DONE");
  }
}
