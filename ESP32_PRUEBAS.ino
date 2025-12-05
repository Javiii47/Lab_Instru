// ==========================================
// INTEGRACIÓN FINAL: TFMINI-S + MOTOR + SERVOS
// (OPTIMIZADO + CAMBIO DE MODO POR SERIAL)
// ==========================================

#include <Arduino.h>
#include <ESP32Servo.h> 

// --- CONFIGURACIÓN LIDAR ---
HardwareSerial tfLidar1(2);  // Pines 34 (RX), 35 (TX)

// --- CONFIGURACIÓN MOTOR ---
const int RPWM_PIN = 26; 
const int EN_R_PIN = 27; 
const int LPWM_PIN = 25; 
const int EN_L_PIN = 23; 

// --- CONFIGURACIÓN SERVOS ---
Servo servoLidar;      
Servo servoDireccion;  

const int PIN_SERVO_LIDAR = 13;     
const int PIN_SERVO_DIRECCION = 12; 

// Ángulos Servo Lidar
const int ANGULO_LIDAR_FRENTE = 0;      
const int ANGULO_LIDAR_PARED = 45;      

// Ángulos Servo Dirección
const int DIR_CENTRO = 90;
const int DIR_IZQUIERDA = 60;  
const int DIR_DERECHA = 120;   

// --- CONSTANTES DE VELOCIDAD ---
const int VELOCIDAD_MAX = 200; 
const int VELOCIDAD_MIN = 75;  
const int VELOCIDAD_SEGUIMIENTO = 80; // Lento y preciso

// --- MODOS DE FUNCIONAMIENTO ---
enum ModoCoche {
  MODO_FRENADA,     // 0
  MODO_CARRERA,     // 1
  MODO_SEGUIMIENTO  // 2
};

// Variable de modo (se puede cambiar por Serial)
ModoCoche MODO_ACTUAL = MODO_SEGUIMIENTO;  

// --- CONSTANTES DE DISTANCIA ---
const uint16_t DISTANCIA_OBJETIVO = 10;       // Frenada
const uint16_t DISTANCIA_INICIO_FRENADA = 60; // Frenada
const uint16_t DISTANCIA_PARED_DESEADA = 30;  // Seguimiento
const uint16_t MARGEN_SEGUIMIENTO = 1;        // Tolerancia +/- 1cm

// Variable global de distancia
uint16_t dist_actual = 0; 

// ==========================================
// FUNCIONES AUXILIARES MOTOR/SERVO
// ==========================================

void avanzarMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  digitalWrite(EN_L_PIN, HIGH); 
  analogWrite(LPWM_PIN, 0);
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad);
}

void detenerMotor() {
  digitalWrite(EN_R_PIN, LOW);
  digitalWrite(EN_L_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void girarRuedas(int angulo) {
  servoDireccion.write(angulo);
}

// ==========================================
// PARSER LIDAR "ULTRA RÁPIDO"
// ==========================================
bool readOnePacket(HardwareSerial &ser, uint16_t &dist) {
  if (ser.available() < 9) return false; 
  if (ser.read() == 0x59) {
    if (ser.peek() == 0x59) {
      ser.read(); 
      uint8_t buf[7]; 
      int checksum = 0x59 + 0x59;
      for (int i = 0; i < 7; i++) {
        buf[i] = ser.read();
        if (i < 6) checksum += buf[i]; 
      }
      if ((uint8_t)(checksum & 0xFF) == buf[6]) {
        dist = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        return true;
      }
    }
  }
  return false;
}

void actualizarLidarRapido() {
  uint16_t tempDist = 0;
  while (tfLidar1.available() >= 9) {
    if(readOnePacket(tfLidar1, tempDist)){
       if(tempDist > 0 && tempDist < 1200) { 
          dist_actual = tempDist; 
       }
    }
  }
}

// ==========================================
// FUNCIÓN PARA CAMBIAR MODO DESDE SERIAL
// ==========================================
void verificarCambioModo() {
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();       // Quitar espacios y saltos de línea
    comando.toUpperCase(); // Convertir a mayúsculas

    bool cambioRealizado = false;

    if (comando == "FRENADA" || comando == "0") {
      MODO_ACTUAL = MODO_FRENADA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
      cambioRealizado = true;
      Serial.println(">> CAMBIO A MODO: FRENADA");
    } 
    else if (comando == "CARRERA" || comando == "1") {
      MODO_ACTUAL = MODO_CARRERA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
      cambioRealizado = true;
      Serial.println(">> CAMBIO A MODO: CARRERA");
    } 
    else if (comando == "SEGUIMIENTO" || comando == "2") {
      MODO_ACTUAL = MODO_SEGUIMIENTO;
      servoLidar.write(ANGULO_LIDAR_PARED);
      cambioRealizado = true;
      Serial.println(">> CAMBIO A MODO: SEGUIMIENTO");
    }

    if (cambioRealizado) {
      detenerMotor();           // Parar por seguridad al cambiar
      girarRuedas(DIR_CENTRO);  // Enderezar ruedas
      delay(500);               // Dar tiempo al servo Lidar a moverse
    }
  }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);           
  tfLidar1.begin(115200, SERIAL_8N1, 34, 35); 
  
  pinMode(RPWM_PIN, OUTPUT); pinMode(EN_R_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT); pinMode(EN_L_PIN, OUTPUT);

  servoLidar.setPeriodHertz(50);    
  servoLidar.attach(PIN_SERVO_LIDAR, 500, 2400); 
  servoDireccion.setPeriodHertz(50);
  servoDireccion.attach(PIN_SERVO_DIRECCION, 500, 2400);

  detenerMotor();
  girarRuedas(DIR_CENTRO);
  
  // Posición inicial
  if (MODO_ACTUAL == MODO_SEGUIMIENTO) {
    servoLidar.write(ANGULO_LIDAR_PARED);
  } else {
    servoLidar.write(ANGULO_LIDAR_FRENTE);
  }
  delay(500); 

  Serial.println("=== SISTEMA LISTO ===");
  Serial.println("Escribe 'FRENADA', 'CARRERA' o 'SEGUIMIENTO' para cambiar.");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {

  // 1. Verificar si hay órdenes del usuario por Serial
  verificarCambioModo();

  // 2. Limpiar buffer Lidar (Anti-Lag)
  actualizarLidarRapido();

  // 3. Máquina de estados
  switch (MODO_ACTUAL) {

    // --- MODO SEGUIMIENTO ---
    case MODO_SEGUIMIENTO: {
      if (dist_actual == 0) return; 

      if (dist_actual > (DISTANCIA_PARED_DESEADA + MARGEN_SEGUIMIENTO)) {
         girarRuedas(DIR_IZQUIERDA);
      }
      else if (dist_actual < (DISTANCIA_PARED_DESEADA - MARGEN_SEGUIMIENTO)) {
         girarRuedas(DIR_DERECHA);
      }
      else {
         girarRuedas(DIR_CENTRO);
      }
      avanzarMotor(VELOCIDAD_SEGUIMIENTO);
      break; 
    } 

    // --- MODO CARRERA ---
    case MODO_CARRERA:
      avanzarMotor(VELOCIDAD_MAX); 
      delay(50); 
      break;

    // --- MODO FRENADA ---
    case MODO_FRENADA: {
       static int velocidadActual = 0;
       if (dist_actual == 0) return; 

       if (dist_actual <= DISTANCIA_OBJETIVO) {
          velocidadActual = 0;
          detenerMotor();
       } 
       else if (dist_actual > DISTANCIA_INICIO_FRENADA) {
          velocidadActual = VELOCIDAD_MAX;
          avanzarMotor(velocidadActual);
       } 
       else {
          velocidadActual = map(dist_actual, DISTANCIA_OBJETIVO, DISTANCIA_INICIO_FRENADA, VELOCIDAD_MIN, VELOCIDAD_MAX);
          avanzarMotor(velocidadActual);
       }
       break;
    }
  }
}