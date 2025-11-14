// ==== TFmini-S con ESP32 (UART2) ====

HardwareSerial tfLidar1(2);  // Lidar 1 (UART2) en pines 34, 35
HardwareSerial tfLidar2(1);  // Lidar 2 (UART1) en pines 16, 17

enum ModoCoche {
  MODO_FRENADA,     // (vale 0)
  MODO_CARRERA,     // (vale 1)
  MODO_SEGUIMIENTO  // (vale 2)
};

ModoCoche MODO_ACTUAL = MODO_SEGUIMIENTO;  

// Mejoras: constantes y helper para parser TFmini-S
const uint16_t STOP_THRESHOLD = 10;      // umbral para STOP

// Lee un paquete TFmini-S desde 'ser'. Devuelve true si se consiguió una distancia válida.
bool readTFmini(HardwareSerial &ser, uint16_t &dist, unsigned long timeoutMs = 25) {
  uint8_t buf[9];
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    if (!ser.available()) continue;
    int b = ser.read();
    if (b != 0x59) continue; // buscar primer 0x59

    // esperar y comprobar segundo 0x59
    unsigned long t2 = millis();
    while ((millis() - t2) < timeoutMs && !ser.available()) { }
    if (!ser.available()) continue;
    int second = ser.peek();
    if (second != 0x59) continue;

    ser.read(); // consumir segundo 0x59
    buf[0] = 0x59; buf[1] = 0x59;

    // leer los 7 bytes restantes con timeout
    int idx = 2;
    unsigned long t3 = millis();
    while (idx < 9 && (millis() - t3) < timeoutMs) {
      if (ser.available()) {
        buf[idx++] = ser.read();
      }
    }
    if (idx != 9) continue; // paquete incompleto

    // checksum
    uint8_t sum = 0;
    for (int i = 0; i < 8; i++) sum += buf[i];
    if (sum != buf[8]) continue; // paquete corrupto

    dist = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    return true;
  }
  return false;
}

void setup() {
  Serial.begin(115200);           // Monitor serie del PC
  
  // Dejamos tus pines, como dijiste
  tfLidar1.begin(115200, SERIAL_8N1, 34, 35); 
  tfLidar2.begin(115200, SERIAL_8N1, 16, 17);
  
  Serial.println("Iniciando TFmini-S...");
}

void loop() {

  // El buffer 'data' ya no es 'static' ni global
  // Lo crearemos dentro del 'case'
  
  switch (MODO_ACTUAL) {

    case MODO_SEGUIMIENTO: {
      // =======================================================
      // Variables que recuerdan estado
      // =======================================================
      static uint16_t dist_1 = 0;
      static uint16_t dist_2 = 0;
      static unsigned long lastPrint = 0;
      const unsigned long PRINT_INTERVAL = 200; // ms

      const char* lectura1 = "N/A";
      const char* lectura2 = "N/A";

      // Intentar leer LIDAR 1 y 2 (no bloqueante, con timeout corto)
      if (readTFmini(tfLidar1, dist_1)) {
        lectura1 = (dist_1 <= STOP_THRESHOLD) ? "STOP" : "AVANZA";
      }

      if (readTFmini(tfLidar2, dist_2)) {
        lectura2 = (dist_2 <= STOP_THRESHOLD) ? "STOP" : "AVANZA";
      }

      // Imprimir periódicamente para no saturar el monitor
      if (millis() - lastPrint >= PRINT_INTERVAL) {
        lastPrint = millis();
        Serial.printf("LIDAR 1: %s \tDist: %d \t| LIDAR 2: %s \tDist: %d\n",
                      lectura1, dist_1, lectura2, dist_2);
      }

      // Pequeño delay para liberar CPU (no crítico gracias a timeouts)
      delay(5);

      break; 
    } 

    case MODO_CARRERA:
      Serial.println("Modo CARRERA activado");
      break;

    case MODO_FRENADA:
      // (Aquí deberías aplicar los mismos arreglos: 2 buffers)
      Serial.println("Modo FRENADA activado");
      break;
  }
}