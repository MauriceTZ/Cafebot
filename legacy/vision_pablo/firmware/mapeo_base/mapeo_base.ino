// Mapea el cableado de la base cuando no se sabe qué GPIO va a qué pin de los
// puentes H (IBT_2) ni a qué encoder.
//
// Comandos (115200, respuesta DONE / ERR):
//   enc <ms>       cuenta cambios en todos los pines durante <ms> (máx 20000).
//                  Girar UNA rueda a mano: los 2 pines que cambian son su encoder.
//   excluir g g..  marca pines como encoder: nunca se manejan como salida.
//   motores <ms>   todos los pines no excluidos en HIGH (así cualquier enable
//                  queda activo) y se baja UNO a la vez durante <ms>. La rueda
//                  que gira en ese momento tiene ese pin como LPWM o RPWM.
//                  Si un pin es enable, en su turno no gira nada.
//                  Además cuenta qué pines excluidos (encoders) cambian.
//   todos <ms>     todos los pines no excluidos en HIGH durante <ms>: nada
//                  debería girar (las dos entradas en HIGH frenan el motor).
//   s              todo en LOW.
// Las salidas usan la fuerza mínima (~5 mA) por si algún pin es la salida de
// un encoder.

#include "driver/gpio.h"

const int CAND[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
const int N = sizeof(CAND) / sizeof(CAND[0]);
const int SOLO_ENTRADA[] = {34, 35, 36, 39};
const int NE = sizeof(SOLO_ENTRADA) / sizeof(SOLO_ENTRADA[0]);

bool excluido[N];
String linea;

void salida(int p, int nivel) {
  pinMode(p, OUTPUT);
  gpio_set_drive_capability((gpio_num_t)p, GPIO_DRIVE_CAP_0);
  digitalWrite(p, nivel);
}

void reposo() {
  for (int i = 0; i < N; i++) {
    if (excluido[i]) pinMode(CAND[i], INPUT);
    else salida(CAND[i], LOW);
  }
  for (int j = 0; j < NE; j++) pinMode(SOLO_ENTRADA[j], INPUT);
}

int indice(int gpio) {
  for (int i = 0; i < N; i++)
    if (CAND[i] == gpio) return i;
  return -1;
}

// Cuenta cambios durante ms. soloExcluidos: mira solo encoders + solo-entrada;
// si no, mira todos los pines como entrada.
void contarCambios(unsigned long ms, bool soloExcluidos) {
  int pins[N + NE], prev[N + NE];
  long cambios[N + NE];
  int k = 0;
  for (int i = 0; i < N; i++)
    if (!soloExcluidos || excluido[i]) pins[k++] = CAND[i];
  for (int j = 0; j < NE; j++) pins[k++] = SOLO_ENTRADA[j];
  for (int i = 0; i < k; i++) { prev[i] = digitalRead(pins[i]); cambios[i] = 0; }
  unsigned long fin = millis() + ms;
  while ((long)(fin - millis()) > 0) {
    for (int i = 0; i < k; i++) {
      int v = digitalRead(pins[i]);
      if (v != prev[i]) { cambios[i]++; prev[i] = v; }
    }
  }
  bool alguno = false;
  for (int i = 0; i < k; i++) {
    if (cambios[i] > 2) {
      Serial.printf("    GPIO %d cambió %ld veces\n", pins[i], cambios[i]);
      alguno = true;
    }
  }
  if (!alguno) Serial.println("    (ningún encoder cambió)");
}

void todosAlto() {
  for (int i = 0; i < N; i++)
    if (!excluido[i]) salida(CAND[i], HIGH);
}

void setup() {
  for (int i = 0; i < N; i++) excluido[i] = false;
  reposo();  // lo antes posible, para que nada gire solo
  Serial.begin(115200);
  Serial.println("mapeo_base listo. Comandos: enc <ms>, excluir g.., motores <ms>, todos <ms>, s");
}

void procesar(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  int ms;
  if (cmd == "s") {
    reposo();
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "enc %d", &ms) == 1) {
    for (int i = 0; i < N; i++) pinMode(CAND[i], INPUT);
    Serial.printf("Contando cambios por %d ms: gira UNA rueda a mano...\n", ms);
    contarCambios(min((unsigned long)ms, 20000UL), false);
    reposo();
    Serial.println("DONE");
  } else if (cmd.startsWith("excluir")) {
    for (int i = 0; i < N; i++) excluido[i] = false;
    String resto = cmd.substring(7);
    resto.trim();
    while (resto.length() > 0) {
      int esp = resto.indexOf(' ');
      String t = esp < 0 ? resto : resto.substring(0, esp);
      int i = indice(t.toInt());
      if (i >= 0) excluido[i] = true;
      resto = esp < 0 ? "" : resto.substring(esp + 1);
      resto.trim();
    }
    reposo();
    Serial.print("excluidos:");
    for (int i = 0; i < N; i++)
      if (excluido[i]) Serial.printf(" %d", CAND[i]);
    Serial.println();
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "todos %d", &ms) == 1) {
    Serial.println("todos en HIGH (no debería girar nada)");
    todosAlto();
    contarCambios(min((unsigned long)ms, 5000UL), true);
    reposo();
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "motores %d", &ms) == 1) {
    for (int i = 0; i < N; i++) {
      if (excluido[i]) continue;
      todosAlto();
      digitalWrite(CAND[i], LOW);
      Serial.printf("GPIO %d en LOW (resto HIGH)\n", CAND[i]);
      contarCambios(min((unsigned long)ms, 5000UL), true);
      reposo();
      delay(1500);  // que la rueda se detenga antes del siguiente
    }
    Serial.println("DONE");
  } else {
    Serial.print("ERR comando no reconocido: ");
    Serial.println(cmd);
  }
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
}
