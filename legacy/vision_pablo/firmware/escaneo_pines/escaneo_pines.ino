// Descubre el cableado de la base sin seguir los cables a mano:
// qué pines son encoders y qué pines mueven cada motor (vía los IBT_2).
//
// Clasificación inicial de cada pin candidato (con pull-up / pull-down internos):
//   - "suelto": nada le impone nivel. Se deja en LOW (salida).
//   - "externo": algo le impone un nivel (encoder, resistencia del IBT_2...).
//     Se deja como entrada.
// Para no dañar nada al forzar un pin contra otra salida, todas las salidas
// usan la fuerza MÍNIMA de la ESP32 (~5 mA, GPIO_DRIVE_CAP_0).
//
// Comandos (115200, respuesta DONE / ERR):
//   list           clasificación de los pines
//   enc <ms>       cuenta cambios en las entradas durante <ms> (girar ruedas a
//                  mano para encontrar los encoders)
//   auto           para cada pin: lo invierte 1,5 s (suelto -> HIGH,
//                  externo alto -> LOW) y cuenta qué entradas cambian
//   p <gpio> <ms>  invierte solo ese pin (máx 5000 ms)
//   s              todo a su estado de reposo

#include "driver/gpio.h"

// GPIO de salida posibles en una ESP32 DevKit (sin 1/3 = USB serial y 6-11 = flash).
const int CAND[] = {2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
const int N = sizeof(CAND) / sizeof(CAND[0]);
// Pines solo-entrada: típicos para encoders. Solo se leen.
const int SOLO_ENTRADA[] = {34, 35, 36, 39};
const int NE = sizeof(SOLO_ENTRADA) / sizeof(SOLO_ENTRADA[0]);

bool suelto[N];
int nivelExterno[N];
String linea;

void reposo(int i) {
  int p = CAND[i];
  if (suelto[i]) {
    pinMode(p, OUTPUT);
    gpio_set_drive_capability((gpio_num_t)p, GPIO_DRIVE_CAP_0);
    digitalWrite(p, LOW);
  } else {
    pinMode(p, INPUT);
  }
}

void clasificar() {
  for (int i = 0; i < N; i++) {
    int p = CAND[i];
    pinMode(p, INPUT_PULLUP);
    delay(3);
    int a = digitalRead(p);
    pinMode(p, INPUT_PULLDOWN);
    delay(3);
    int b = digitalRead(p);
    suelto[i] = (a == 1 && b == 0);
    pinMode(p, INPUT);
    delay(3);
    nivelExterno[i] = digitalRead(p);
    reposo(i);
  }
  for (int j = 0; j < NE; j++) pinMode(SOLO_ENTRADA[j], INPUT);
}

int indice(int gpio) {
  for (int i = 0; i < N; i++)
    if (CAND[i] == gpio) return i;
  return -1;
}

// Cuenta cambios en todas las entradas (externos + solo-entrada) durante ms,
// excepto el pin "excluir". Imprime las que cambiaron.
void contarCambios(unsigned long ms, int excluir) {
  int pins[N + NE], prev[N + NE];
  long cambios[N + NE];
  int k = 0;
  for (int i = 0; i < N; i++)
    if (!suelto[i] && CAND[i] != excluir) pins[k++] = CAND[i];
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
  if (!alguno) Serial.println("    (ninguna entrada cambió)");
}

// Invierte un pin respecto de su reposo y mide qué se mueve.
void probarPin(int i, unsigned long ms) {
  int p = CAND[i];
  if (suelto[i]) {
    Serial.printf("GPIO %d -> HIGH\n", p);
    digitalWrite(p, HIGH);
  } else {
    int destino = nivelExterno[i] ? LOW : HIGH;
    Serial.printf("GPIO %d (externo %d) -> %s\n", p, nivelExterno[i], destino ? "HIGH" : "LOW");
    pinMode(p, OUTPUT);
    gpio_set_drive_capability((gpio_num_t)p, GPIO_DRIVE_CAP_0);
    digitalWrite(p, destino);
  }
  contarCambios(min(ms, 5000UL), p);
  reposo(i);
}

void listar() {
  for (int i = 0; i < N; i++) {
    if (suelto[i]) Serial.printf("GPIO %2d: suelto (LOW)\n", CAND[i]);
    else Serial.printf("GPIO %2d: externo, nivel %d\n", CAND[i], nivelExterno[i]);
  }
  for (int j = 0; j < NE; j++)
    Serial.printf("GPIO %2d: solo entrada, nivel %d\n", SOLO_ENTRADA[j], digitalRead(SOLO_ENTRADA[j]));
}

void setup() {
  clasificar();  // lo antes posible, para que nada gire solo
  Serial.begin(115200);
  Serial.println("escaneo_pines listo. Comandos: list, enc <ms>, auto, p <gpio> <ms>, s");
}

void procesar(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  int g, ms;
  if (cmd == "list") {
    listar();
    Serial.println("DONE");
  } else if (cmd == "s") {
    for (int i = 0; i < N; i++) reposo(i);
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "enc %d", &ms) == 1) {
    Serial.printf("Contando cambios por %d ms: gira las ruedas a mano...\n", ms);
    contarCambios(min((unsigned long)ms, 20000UL), -1);
    Serial.println("DONE");
  } else if (cmd == "auto") {
    for (int i = 0; i < N; i++) {
      if (CAND[i] == 2) continue;  // LED de la placa
      probarPin(i, 1500);
      delay(1500);  // dejar que la rueda se detenga
    }
    Serial.println("DONE");
  } else if (sscanf(cmd.c_str(), "p %d %d", &g, &ms) == 2) {
    int i = indice(g);
    if (i < 0) { Serial.println("ERR pin no candidato"); return; }
    probarPin(i, ms);
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
