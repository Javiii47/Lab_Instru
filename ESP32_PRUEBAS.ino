// ==========================================
// INTEGRACIÓN TFMINI-S + CONTROL DE MOTOR
// ==========================================

#include <Arduino.h>

// --- CONFIGURACIÓN LIDAR ---
HardwareSerial tfLidar1(2);  // Lidar 1 (UART2) en pines 34 (RX), 35 (TX)
// --- CONFIGURACIÓN MOTOR ---
// Ajusta estos pines según tu cableado real si el motor gira al revés
const int RPWM_PIN = 26; // Velocidad Adelante
const int EN_R_PIN = 27; // Habilitar Adelante
const int LPWM_PIN = 25; // Velocidad Atrás
const int EN_L_PIN = 23; // Habilitar Atrás

// --- CONSTANTES DE VELOCIDAD ---
const int VELOCIDAD_MAX = 200; // Velocidad normal de crucero
const int VELOCIDAD_MIN = 75;  // Velocidad mínima para que el motor no se cale antes de llegar (ajustar si hace ruido pero no mueve)

// --- MODOS DE FUNCIONAMIENTO ---
enum ModoCoche {
  MODO_FRENADA,     // (vale 0)
  MODO_CARRERA,     // (vale 1)
  MODO_SEGUIMIENTO  // (vale 2)
};

// Selecciona aquí el modo inicial
ModoCoche MODO_ACTUAL = MODO_FRENADA;  

// --- CONSTANTES DE DISTANCIA ---
const uint16_t DISTANCIA_PARED_LATERAL = 10;      
const uint16_t DISTANCIA_OBJETIVO = 10;       // Punto exacto donde detenerse (10 cm)
const uint16_t DISTANCIA_INICIO_FRENADA = 60; // A partir de 60 cm empieza a reducir velocidad

// ==========================================
// FUNCIONES AUXILIARES DE MOTOR
// ==========================================

// Función para mover el motor hacia adelante con velocidad variable
void avanzarMotor(int velocidad) {
  // Limitamos la velocidad para no exceder 255 ni bajar de 0
  velocidad = constrain(velocidad, 0, 255);

  // Aseguramos que la marcha atrás esté desactivada
  digitalWrite(EN_L_PIN, HIGH); 
  analogWrite(LPWM_PIN, 0);

  // Activamos marcha adelante
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad);
}

// Función para detener el motor completamente
void detenerMotor() {
  digitalWrite(EN_R_PIN, LOW);
  digitalWrite(EN_L_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

// ==========================================
// PARSER LIDAR (Tu código original)
// ==========================================
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

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);           
  
  // Iniciar Lidar
  tfLidar1.begin(115200, SERIAL_8N1, 34, 35); 
  tfLidar2.begin(115200, SERIAL_8N1, 16, 17);
  
  // Iniciar Pines de Motor
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(EN_R_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(EN_L_PIN, OUTPUT);

  // Estado inicial del motor: Parado
  detenerMotor();
  
  Serial.println("Sistema Iniciado. Lidar + Motor OK.");
  Serial.print("Modo Actual: ");
  Serial.println(MODO_ACTUAL);
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {

  switch (MODO_ACTUAL) {

    // -----------------------------------------------------
    // MODO SEGUIMIENTO
    // -----------------------------------------------------
    case MODO_SEGUIMIENTO: {
      static uint16_t dist_1 = 0;
      static unsigned long lastPrint = 0;
      const unsigned long PRINT_INTERVAL = 200;

      const char* lectura1 = "N/A";

      // Lógica Lidar 1 (Frontal)
      if (readTFmini(tfLidar1, dist_1)) {
        if (dist_1 <= DISTANCIA_OBJETIVO) {
            lectura1 = "STOP";
            // Aquí deberías llamar a detenerMotor() si quieres que pare en este modo también
        } else {
            lectura1 = "AVANZA";
        }
      }

      // Debug
      if (millis() - lastPrint >= PRINT_INTERVAL) {
        lastPrint = millis();
        Serial.printf("SEGUIMIENTO | L1: %s (%d cm) | L2: %s (%d cm)\n",
                      lectura1, dist_1, lectura2, dist_2);
      }
      delay(5);
      break; 
    } 

    // -----------------------------------------------------
    // MODO CARRERA
    // -----------------------------------------------------
    case MODO_CARRERA:
      avanzarMotor(VELOCIDAD_MAX); 
      Serial.println("Modo CARRERA - A toda velocidad");
      delay(500);
      break;

    // -----------------------------------------------------
    // MODO FRENADA PROGRESIVA
    // -----------------------------------------------------
    case MODO_FRENADA: {
       static uint16_t dist_1 = 0;
       static unsigned long lastPrint = 0;
       const unsigned long PRINT_INTERVAL = 100; // Más rápido para ver el cambio de velocidad
       
       static int velocidadActual = 0;

       // 1. Leemos el sensor frontal
       if (readTFmini(tfLidar1, dist_1)) {
         
         // Ignorar lecturas erróneas de 0 (el TFmini a veces devuelve 0 si falla la lectura o rango excesivo)
         if (dist_1 == 0) return; 

         // 2. Lógica de Control de Velocidad
         if (dist_1 <= DISTANCIA_OBJETIVO) {
            // --- CASO 1: LLEGAMOS AL OBJETIVO (<= 10cm) ---
            velocidadActual = 0;
            detenerMotor();
         } 
         else if (dist_1 > DISTANCIA_INICIO_FRENADA) {
            // --- CASO 2: LEJOS DEL OBJETIVO (> 60cm) ---
            velocidadActual = VELOCIDAD_MAX;
            avanzarMotor(velocidadActual);
         } 
         else {
            // --- CASO 3: ZONA DE FRENADO (entre 10cm y 60cm) ---
            // Mapeamos la distancia proporcionalmente a la velocidad.
            // A 10cm -> VELOCIDAD_MIN
            // A 60cm -> VELOCIDAD_MAX
            velocidadActual = map(dist_1, DISTANCIA_OBJETIVO, DISTANCIA_INICIO_FRENADA, VELOCIDAD_MIN, VELOCIDAD_MAX);
            avanzarMotor(velocidadActual);
         }
       }

       // 3. Imprimir estado
       if (millis() - lastPrint >= PRINT_INTERVAL) {
         lastPrint = millis();
         Serial.printf("FRENADA | Dist: %d cm | PWM Motor: %d\n", dist_1, velocidadActual);
       }
       
       delay(5);
       break;
    }
  }
}