// Prueba del enlace Pi <-> ESP32 por USB. NO mueve motores.
// Protocolo igual al del robot: la Pi manda una línea, la ESP32 responde
// "DONE [datos]" o "ERR <motivo>".
//   PING        -> DONE PONG
//   LED ON|OFF  -> DONE (LED de la placa, si tiene, en el pin 2)
//   cualquier otra cosa -> ERR desconocido: <linea>
// Se puede probar también desde el Monitor Serie del Arduino IDE (115200,
// fin de línea "Nueva línea").

const int PIN_LED = 2;
String linea;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  Serial.println("ESP32 lista");  // mensaje informativo (la Pi lo ignora)
}

void procesar(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;
  if (cmd == "PING") {
    Serial.println("DONE PONG");
  } else if (cmd == "LED ON") {
    digitalWrite(PIN_LED, HIGH);
    Serial.println("DONE");
  } else if (cmd == "LED OFF") {
    digitalWrite(PIN_LED, LOW);
    Serial.println("DONE");
  } else {
    Serial.print("ERR desconocido: ");
    Serial.println(cmd);
  }
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      procesar(linea);
      linea = "";
    } else if (c != '\r') {
      linea += c;
    }
  }
}
