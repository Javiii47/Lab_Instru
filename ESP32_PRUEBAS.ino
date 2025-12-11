#include <Arduino.h>
#include <ESP32Servo.h> 
#include <WiFi.h>

// ==========================================
// 1. CONFIGURACIÓN WIFI (SCADA)
// ==========================================
//const char* ssid = "iPhone de Lucia";      // PON AQUÍ TU RED
//const char* password = "hola1234";  

const char* ssid = "Qué mira bobo";      // PON AQUÍ TU RED
const char* password = "Peppa Pig";         // PON AQUÍ TU PASS
WiFiServer tcpServer(23);                  // Puerto 23
WiFiClient client;                         // Cliente global

// Variables para simular batería
float sim_vbat = 12.6;
float sim_ibat = 2.0;

// ==========================================
// 2. CONFIGURACIÓN HARDWARE COCHE
// ==========================================
// --- LIDAR ---
HardwareSerial tfLidar1(2);  // Pines 16 (RX), 17 (TX)w

// --- MOTOR ---
const int RPWM_PIN = 15; 
const int EN_R_PIN = 26; 
const int LPWM_PIN = 27; 
const int EN_L_PIN = 33; 

// --- SERVOS ---
Servo servoLidar;      
Servo servoDireccion;  
const int PIN_SERVO_LIDAR = 13;     
const int PIN_SERVO_DIRECCION = 18; 

// --- ÁNGULOS ---
const int ANGULO_LIDAR_FRENTE = 0;      
const int ANGULO_LIDAR_PARED = 45;      
const int DIR_CENTRO = 90;
const int DIR_IZQUIERDA = 60;  
const int DIR_DERECHA = 120;   

// --- CONSTANTES DE CONTROL ---
int VELOCIDAD_MAX = 200; 
int VELOCIDAD_MIN = 0;
int VELOCIDAD_SEGUIMIENTO = 80; 
uint16_t DISTANCIA_INICIO_FRENADA = 60; 

// Distancia objetivo para frenar (recibida del SCADA)
uint16_t DISTANCIA_OBJETIVO = 10; 

const uint16_t DISTANCIA_PARED_DESEADA = 30;  
const uint16_t MARGEN_SEGUIMIENTO = 1; 

// --- MODOS DE FUNCIONAMIENTO ---
enum ModoCoche {
  MODO_FRENADA,     // 0
  MODO_CARRERA,     // 1
  MODO_SEGUIMIENTO, // 2
  MODO_MANUAL       // 3 
};

ModoCoche MODO_ACTUAL = MODO_SEGUIMIENTO;  

// Variables Globales de Estado
uint16_t dist_actual = 0;   // Lectura Lidar
int pwm_actual = 0;         // PWM actual del motor
int sentido_actual = 1;     // 1 = Adelante, 0 = Atrás
int angulo_servo_actual = 90; 

// Variables recibidas del SCADA (Buffers)
int rx_angulo = 90;
int rx_duty_fwd = 0;
int rx_duty_bwd = 0;
int rx_dist_frenada = 10;
int rx_btn_carrera = 0;
int rx_btn_frenada = 0;
int rx_btn_seguimiento = 0;


// ==========================================
// 3. FUNCIONES PARSER (RECIBIR DATOS SCADA)
// ==========================================
// Espera: Ángulo;DutyFwd;DutyBwd;DistFrenada;Modo C;Modo F;Modo S*
void procesarTramaSCADA(String trama) {
  
  // 1. Ángulo
  int idx1 = trama.indexOf(';');
  if(idx1 == -1) return; 
  rx_angulo = trama.substring(0, idx1).toInt();
  
  // 2. Duty Fwd
  int idx2 = trama.indexOf(';', idx1 + 1);
  if(idx2 == -1) return;
  rx_duty_fwd = trama.substring(idx1 + 1, idx2).toInt();

  // 3. Duty Bwd
  int idx3 = trama.indexOf(';', idx2 + 1);
  if(idx3 == -1) return;
  rx_duty_bwd = trama.substring(idx2 + 1, idx3).toInt();

  // 4. Distancia Frenada Objetivo
  int idx4 = trama.indexOf(';', idx3 + 1);
  if(idx4 == -1) return;
  rx_dist_frenada = trama.substring(idx3 + 1, idx4).toInt();
  if(rx_dist_frenada > 0) DISTANCIA_OBJETIVO = rx_dist_frenada;

  // 5. Modo Carrera (0 o 1)
  int idx5 = trama.indexOf(';', idx4 + 1);
  if(idx5 == -1) return;
  rx_btn_carrera = trama.substring(idx4 + 1, idx5).toInt();

  // 6. Modo Frenada (0 o 1)
  int idx6 = trama.indexOf(';', idx5 + 1);
  if(idx6 == -1) return;
  rx_btn_frenada = trama.substring(idx5 + 1, idx6).toInt();

  // 7. Modo Seguimiento (0 o 1)
  rx_btn_seguimiento = trama.substring(idx6 + 1).toInt();

  // --- LÓGICA DE CAMBIO DE MODO ---
  if (rx_btn_carrera == 1) {
    if (MODO_ACTUAL != MODO_CARRERA) {
        MODO_ACTUAL = MODO_CARRERA;
        servoLidar.write(ANGULO_LIDAR_FRENTE);
        Serial.println("SCADA -> MODO CARRERA");
    }
  }
  else if (rx_btn_frenada == 1) {
    if (MODO_ACTUAL != MODO_FRENADA) {
        MODO_ACTUAL = MODO_FRENADA;
        servoLidar.write(ANGULO_LIDAR_FRENTE);
        Serial.println("SCADA -> MODO FRENADA");
    }
  }
  else if (rx_btn_seguimiento == 1) {
    if (MODO_ACTUAL != MODO_SEGUIMIENTO) {
        MODO_ACTUAL = MODO_SEGUIMIENTO;
        servoLidar.write(ANGULO_LIDAR_PARED);
        Serial.println("SCADA -> MODO SEGUIMIENTO");
    }
  }
}

// ==========================================
// 4. FUNCIONES HARDWARE (MOTOR / LIDAR)
// ==========================================

void avanzarMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad; 
  sentido_actual = 1;
  digitalWrite(EN_L_PIN, HIGH); 
  analogWrite(LPWM_PIN, 0);
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad);
}

// --- NUEVA FUNCIÓN AÑADIDA PARA MODO TELEDIRIGIDO ---
void retrocederMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad; 
  sentido_actual = 0; // Indicamos que va hacia atrás
  digitalWrite(EN_L_PIN, HIGH); 
  analogWrite(LPWM_PIN, velocidad); // Activar PWM izquierdo para retroceso
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);         // Apagar PWM derecho
}
// ----------------------------------------------------

void detenerMotor() {
  pwm_actual = 0;
  digitalWrite(EN_R_PIN, LOW);
  digitalWrite(EN_L_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void girarRuedas(int angulo) {
  servoDireccion.write(angulo);
  angulo_servo_actual = angulo;
}

// Lidar Non-Blocking
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
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);           

  // --- 1. INICIAR HARDWARE ---
  tfLidar1.begin(115200, SERIAL_8N1, 16, 17); 
  
  pinMode(RPWM_PIN, OUTPUT); pinMode(EN_R_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT); pinMode(EN_L_PIN, OUTPUT);

  servoLidar.setPeriodHertz(50);    
  servoLidar.attach(PIN_SERVO_LIDAR, 500, 2400); 
  servoDireccion.setPeriodHertz(50);
  servoDireccion.attach(PIN_SERVO_DIRECCION, 500, 2400);

  // Estado Inicial
  detenerMotor();
  girarRuedas(DIR_CENTRO);
  
  if (MODO_ACTUAL == MODO_SEGUIMIENTO) servoLidar.write(ANGULO_LIDAR_PARED);
  else servoLidar.write(ANGULO_LIDAR_FRENTE);

  // --- 2. CONEXIÓN WIFI ---
  Serial.println("\nConectando a WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Esperamos conexión
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("\nWiFi Conectado!");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP()); 

  tcpServer.begin();
  Serial.println("Servidor TCP puerto 23 iniciado");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop() {
  
  // 1. GESTIÓN WIFI
  if (tcpServer.hasClient()) {
    if (client.connected()) client.stop();
    client = tcpServer.available();
    Serial.println("-> SCADA (LabVIEW) Conectado");
  }

  if (client && client.connected()) {
    if (client.available()) {
      String tramaRecibida = client.readStringUntil('*');
      tramaRecibida.trim();
      procesarTramaSCADA(tramaRecibida);
    }
  }

  // 2. LEER SENSORES
  actualizarLidarRapido();

  // 3. LÓGICA DE CONTROL
  switch (MODO_ACTUAL) {

    // --- SEGUIMIENTO ---
    case MODO_SEGUIMIENTO: {
      if (dist_actual > 0) {
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
      }
      break; 
    } 

    // --- CARRERA (MODIFICADO: AHORA ES TELEDIRIGIDO) ---
    case MODO_CARRERA:
      // Control DIRECCIÓN: Usamos el ángulo que llega del SCADA
      girarRuedas(rx_angulo);

      // Control MOTOR: Usamos los duties que llegan del SCADA
      // Ponemos un pequeño umbral (5) para que si el slider está a 0 no vibre
      if (rx_duty_fwd > 5) {
          // Si hay señal de avance
          avanzarMotor(rx_duty_fwd);
      } 
      else if (rx_duty_bwd > 5) {
          // Si hay señal de retroceso (y no de avance)
          retrocederMotor(rx_duty_bwd);
      } 
      else {
          // Si ambos están a 0
          detenerMotor();
      }
      break;

    // --- FRENADA ---
    case MODO_FRENADA: {
       if (dist_actual > 0) {
         if (dist_actual <= DISTANCIA_OBJETIVO) {
            detenerMotor();
         } 
         else if (dist_actual > DISTANCIA_INICIO_FRENADA) {
            avanzarMotor(VELOCIDAD_MAX);
         } 
         else {
            int vel = map(dist_actual, DISTANCIA_OBJETIVO, DISTANCIA_INICIO_FRENADA, VELOCIDAD_MIN, VELOCIDAD_MAX);
            avanzarMotor(vel);
         }
       }
       girarRuedas(DIR_CENTRO);
       break;
    }
  }

  // 4. ENVIAR DATOS A SCADA
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 100) {
    lastSendTime = millis();
    
    if (client && client.connected()) {
      int d_frontal = (MODO_ACTUAL == MODO_FRENADA || MODO_ACTUAL == MODO_CARRERA) ? dist_actual : 0;
      int d_lateral = (MODO_ACTUAL == MODO_SEGUIMIENTO) ? dist_actual : 0;
      
      sim_vbat = 12.0 + (random(-5, 5) / 10.0);
      sim_ibat = 1.5 + (pwm_actual / 255.0) + (random(-1, 1) / 10.0);

      String tramaTx = String(angulo_servo_actual) + ";" + 
                       String(d_frontal) + ";" + 
                       String(d_lateral) + ";" + 
                       String(pwm_actual) + ";" + 
                       String(sentido_actual) + ";" + 
                       String(sim_vbat, 1) + ";" +    
                       String(sim_ibat, 1) + "*";    

      client.print(tramaTx);
    }
  }
}