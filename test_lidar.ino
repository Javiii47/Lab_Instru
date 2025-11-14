// ==== TFmini-S con ESP32 (UART2) ====

HardwareSerial tfSerial(2);  // Lidar 1 (UART2) en pines 34, 35
HardwareSerial tfLidar2(1);  // Lidar 2 (UART1) en pines 16, 17

enum ModoCoche {
  MODO_FRENADA,     // (vale 0)
  MODO_CARRERA,     // (vale 1)
  MODO_SEGUIMIENTO  // (vale 2)
};

ModoCoche MODO_ACTUAL = MODO_SEGUIMIENTO;

void setup() {
  Serial.begin(115200);           // Monitor serie del PC
  
  // Dejamos tus pines, como dijiste
  tfSerial.begin(115200, SERIAL_8N1, 34, 35); 
  tfLidar2.begin(115200, SERIAL_8N1, 16, 17);
  
  Serial.println("Iniciando TFmini-S...");
}

void loop() {

  // El buffer 'data' ya no es 'static' ni global
  // Lo crearemos dentro del 'case'
  
  switch (MODO_ACTUAL) {

    case MODO_SEGUIMIENTO: {
      
      // =======================================================
      // ARREGLO 1: Variables 'static' PARA QUE RECUERDEN SU VALOR
      // =======================================================
      static String lecturaActual_1 = "---"; 
      static String lecturaActual_2 = "---";
      static uint16_t dist_1 = 0;
      static uint16_t dist_2 = 0;

      // =======================================================
      // ARREGLO 2: Dos buffers (cajones) SEPARADOS
      // =======================================================
      uint8_t data_Lidar1[9]; // Buffer para tfSerial
      uint8_t data_Lidar2[9]; // Buffer para tfLidar2


      // --- Lógica Lidar 1 (tfSerial) ---
      if (tfSerial.available()) {
        if (tfSerial.read() == 0x59) {
          if (tfSerial.available() && tfSerial.peek() == 0x59) {
            tfSerial.read(); 

            if (tfSerial.available() >= 7) {
              // Usamos el buffer 1
              for (int i = 2; i < 9; i++) {
                data_Lidar1[i] = tfSerial.read();
              }
              data_Lidar1[0] = 0x59; data_Lidar1[1] = 0x59;

              uint8_t sum_1 = 0;
              for (int i = 0; i < 8; i++) sum_1 += data_Lidar1[i];

              if (sum_1 == data_Lidar1[8]) {
                dist_1 = data_Lidar1[2] + (data_Lidar1[3] << 8); 
                if (dist_1 <= 10) {
                  lecturaActual_1 = "STOP";
                } else {
                  lecturaActual_1 = "AVANZA";
                }
              }
            }
          }
        }
      }

      // --- Lógica Lidar 2 (tfLidar2) ---
      if (tfLidar2.available()) {
        if (tfLidar2.read() == 0x59) {
          if (tfLidar2.available() && tfLidar2.peek() == 0x59) {
            tfLidar2.read();

            if (tfLidar2.available() >= 7) {
              // Usamos el buffer 2
              for (int i = 2; i < 9; i++) {
                data_Lidar2[i] = tfLidar2.read();
              }
              data_Lidar2[0] = 0x59; data_Lidar2[1] = 0x59;

              uint8_t sum_2 = 0;
              for (int i = 0; i < 8; i++) sum_2 += data_Lidar2[i];

              if (sum_2 == data_Lidar2[8]) {
                dist_2 = data_Lidar2[2] + (data_Lidar2[3] << 8); 
                if (dist_2 <= 10) {
                  lecturaActual_2 = "STOP";
                } else {
                  lecturaActual_2 = "AVANZA";
                }
              }
            }
          }
        }
      }

      // --- Impresión ---
      // Te he vuelto a poner el %s para que muestre "STOP" o "AVANZA"
      Serial.printf("LIDAR 1: %s \tDist: %d \t| LIDAR 2: %s \tDist: %d\n",
                      lecturaActual_1.c_str(), dist_1,
                      lecturaActual_2.c_str(), dist_2);
      
      // Pequeño delay para no saturar el monitor
  
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