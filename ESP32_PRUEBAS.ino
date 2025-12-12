#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <math.h>

// ==========================================
// 1. CONFIGURACIÓN WIFI (SCADA)
// ==========================================
// Usamos la configuración del primer código (Lab3)
const char *ssid = "coche_lab3";     // TU RED
const char *password = "abcd1234";   // TU PASS
WiFiServer tcpServer(23);            // Puerto 23
WiFiClient client;                   // Cliente global

// Variables para simular batería
float sim_vbat = 6;
float sim_ibat = 5;

// ==========================================
// 2. CONFIGURACIÓN HARDWARE
// ==========================================
// --- LIDAR ---
HardwareSerial tfLidar1(2); // Pines 16 (RX), 17 (TX)

// --- MOTOR ---
const int RPWM_PIN = 25;
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

// --- CONFIGURACIÓN FÍSICA ---
const double ANGLO_SENSOR = 45.0;
const double RAD_CONVERSION = 3.14159 / 180.0;

// Distancias
uint16_t DISTANCIA_OBJETIVO = 20; // Objetivo general
const uint16_t MARGEN_SEGUIMIENTO = 1;

// ==========================================
// 3. VARIABLES DE CONTROL (PID MÚLTIPLE)
// ==========================================

// --- A. PID SEGUIMIENTO DE PARED (WALL FOLLOW) ---
// (Valores del Código 1)
double Kp_Wall = 2.0;
double Ki_Wall = 0.05;
double Kd_Wall = 4.0;
double setPoint_Wall = 30.0; // Distancia deseada a la pared

// Variables internas Wall Follow
double error_Wall, lastError_Wall = 0, integral_Wall = 0;
double pidOutput_Wall = 0; 
unsigned long lastTimePID_Wall = 0;
const int sampleTime_Wall = 50;

// Filtro Lidar (Solo para Wall Follow)
const int NUM_LECTURAS = 3;
double lecturas[NUM_LECTURAS];
int indiceLectura = 0;
double totalFiltro = 0;
const double MAX_DIST_LOGICA = 80.0;

// Velocidades Wall Follow
int velocidadBase = 100;  
int velocidadMinima = 50; 

// --- B. PD FRENADA (BRAKING) ---
// (Valores del Código 2)
float Kp_Brake = 1.2;  // Proporcional Freno
float Kd_Brake = 0.5;  // Derivativo Freno

// Variables internas Frenada
unsigned long last_time_pd_brake = 0;
int last_dist_pd_brake = 0;
bool frenada_completada = false;

// ==========================================
// 4. VARIABLES GLOBALES DE ESTADO
// ==========================================
enum ModoCoche {
  MODO_FRENADA,     // 0
  MODO_CARRERA,     // 1
  MODO_SEGUIMIENTO, // 2
  MODO_ESPERA       // 3
};

ModoCoche MODO_ACTUAL = MODO_ESPERA;

uint16_t dist_actual = 0; // Lectura Lidar Global
int pwm_actual = 0;       
int sentido_actual = 1;   // 1 = Adelante, 0 = Atrás
int angulo_servo_actual = 90;

// Buffers SCADA
int rx_angulo = 90;
int rx_duty_fwd = 0;
int rx_duty_bwd = 0;
int rx_dist_frenada = 10;
int rx_btn_carrera = 0;
int rx_btn_frenada = 0;
int rx_btn_seguimiento = 0;

// ==========================================
// 5. FUNCIONES HARDWARE BÁSICAS
// ==========================================

void detenerMotor() {
  pwm_actual = 0;
  digitalWrite(EN_R_PIN, LOW);
  digitalWrite(EN_L_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

// Restauramos avanzarMotor (Necesario para el PD de frenada)
void avanzarMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad;
  sentido_actual = 1;

  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, velocidad); // Velocidad por LPWM
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, 0);         // 0 por RPWM
}

// Restauramos retrocederMotor (Necesario para el PD de frenada)
void retrocederMotor(int velocidad) {
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad;
  sentido_actual = 0;

  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, 0);         // 0 por LPWM
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad); // Velocidad por RPWM
}

// Función giroMotor (Para modo carrera o giros diferenciales si se usara)
void giroMotor(int velocidad_f, int velocidad_b) {
  velocidad_f = map(velocidad_f, 0, 100, 0, 255);
  velocidad_b = map(velocidad_b, 0, 100, 0, 255);

  int aux_pwm = velocidad_f - velocidad_b;
  if (aux_pwm < 0) sentido_actual = 0;
  else sentido_actual = 1;
  
  pwm_actual = abs(aux_pwm);
  if (pwm_actual <= 5) {
    detenerMotor();
    return;
  }
  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, velocidad_f);
  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad_b);
}

void girarRuedas(int angulo) {
  servoDireccion.write(angulo);
  angulo_servo_actual = angulo;
}

// --- LIDAR ---
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
    if (readOnePacket(tfLidar1, tempDist)) {
      if (tempDist < 1200) { // Filtro picos grandes
        dist_actual = tempDist;
      }
    }
  }
}

// Filtro Promedio (Solo usado en Wall Follow para suavizar)
double filtrarLidar(double nuevaLectura) {
  totalFiltro = totalFiltro - lecturas[indiceLectura];
  if (nuevaLectura > MAX_DIST_LOGICA || nuevaLectura == 0) {
    nuevaLectura = MAX_DIST_LOGICA;
  }
  lecturas[indiceLectura] = nuevaLectura;
  totalFiltro = totalFiltro + lecturas[indiceLectura];
  indiceLectura++;
  if (indiceLectura >= NUM_LECTURAS) indiceLectura = 0;
  return totalFiltro / NUM_LECTURAS;
}

// Función dinámica del Código 1 (Usa avanzarMotor restaurado)
void gestionarVelocidadDinamica(double pidOut) {
  double giroAbs = abs(pidOut); 
  int velObjetivo = map(giroAbs, 0, 45, velocidadBase, velocidadMinima);
  avanzarMotor(velObjetivo);
}

// ==========================================
// 6. PARSER SCADA
// ==========================================
void procesarTramaSCADA(String trama) {
  int idx1 = trama.indexOf(';');
  if (idx1 == -1) return;
  rx_angulo = trama.substring(0, idx1).toInt();

  int idx2 = trama.indexOf(';', idx1 + 1);
  if (idx2 == -1) return;
  rx_duty_fwd = trama.substring(idx1 + 1, idx2).toInt();

  int idx3 = trama.indexOf(';', idx2 + 1);
  if (idx3 == -1) return;
  rx_duty_bwd = trama.substring(idx2 + 1, idx3).toInt();

  int idx4 = trama.indexOf(';', idx3 + 1);
  if (idx4 == -1) return;
  rx_dist_frenada = trama.substring(idx3 + 1, idx4).toInt();
  if (rx_dist_frenada > 0) DISTANCIA_OBJETIVO = rx_dist_frenada;

  int idx5 = trama.indexOf(';', idx4 + 1);
  if (idx5 == -1) return;
  rx_btn_carrera = trama.substring(idx4 + 1, idx5).toInt();

  int idx6 = trama.indexOf(';', idx5 + 1);
  if (idx6 == -1) return;
  rx_btn_frenada = trama.substring(idx5 + 1, idx6).toInt();

  rx_btn_seguimiento = trama.substring(idx6 + 1).toInt();

  // --- LÓGICA DE CAMBIO DE MODO ---
  if (rx_btn_carrera == 1) {
    if (MODO_ACTUAL != MODO_CARRERA) {
      MODO_ACTUAL = MODO_CARRERA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
    }
  }
  else if (rx_btn_frenada == 1) {
    if (MODO_ACTUAL != MODO_FRENADA) {
      MODO_ACTUAL = MODO_FRENADA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
      // RESETEO DEL PD DE FRENADA AL ENTRAR
      last_dist_pd_brake = 0;
      last_time_pd_brake = millis();
      frenada_completada = false; 
    }
  }
  else if (rx_btn_seguimiento == 1) {
    if (MODO_ACTUAL != MODO_SEGUIMIENTO) {
      MODO_ACTUAL = MODO_SEGUIMIENTO;
      servoLidar.write(ANGULO_LIDAR_PARED);
    }
  }

  else {MODO_ACTUAL = MODO_ESPERA;}
}

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // 1. HARDWARE
  tfLidar1.begin(115200, SERIAL_8N1, 16, 17);
  pinMode(RPWM_PIN, OUTPUT); pinMode(EN_R_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT); pinMode(EN_L_PIN, OUTPUT);

  servoLidar.setPeriodHertz(50);
  servoLidar.attach(PIN_SERVO_LIDAR, 500, 2400);
  servoDireccion.setPeriodHertz(50);
  servoDireccion.attach(PIN_SERVO_DIRECCION, 1160, 1900);

  detenerMotor();
  girarRuedas(DIR_CENTRO);

  // Inicializar filtro Wall Follow
  for (int i = 0; i < NUM_LECTURAS; i++) lecturas[i] = 0;

  // 2. WIFI (AP MODO)
  Serial.println("\nAbriendo Zona Wifi...");
  WiFi.softAP(ssid, password);
  Serial.println(WiFi.softAPIP());

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
    Serial.println("-> SCADA Conectado");
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

  // ---------------------------------------------------------
  // MODO SEGUIMIENTO (Código 1 - PID Geométrico)
  // ---------------------------------------------------------
  case MODO_SEGUIMIENTO: {
    unsigned long now = millis();
    actualizarLidarRapido(); // Relectura extra

    if (now - lastTimePID_Wall >= sampleTime_Wall) {
      double dt = (double)(now - lastTimePID_Wall) / 1000.0;

      // A. Filtros
      double inputRaw = (double)dist_actual;
      double inputFiltrado = filtrarLidar(inputRaw);
      double distPerpendicular = inputFiltrado * sin(ANGLO_SENSOR * RAD_CONVERSION);

      // B. Error
      error_Wall = distPerpendicular - setPoint_Wall;

      // C. PID
      integral_Wall += error_Wall * dt;
      double pTerm = Kp_Wall * error_Wall;
      double dTerm = Kd_Wall * (error_Wall - lastError_Wall) / dt;
      double preOutput = pTerm + (Ki_Wall * integral_Wall) + dTerm;

      // D. Salida y Servo
      double limiteLogico = 45.0;
      if (preOutput > limiteLogico) {
        integral_Wall -= error_Wall * dt;
        preOutput = limiteLogico;
      } else if (preOutput < -limiteLogico) {
        integral_Wall -= error_Wall * dt;
        preOutput = -limiteLogico;
      }
      pidOutput_Wall = preOutput;

      int anguloFinal = DIR_CENTRO + (int)pidOutput_Wall;
      girarRuedas(anguloFinal);

      // E. Velocidad Dinámica
      gestionarVelocidadDinamica(pidOutput_Wall);

      lastError_Wall = error_Wall;
      lastTimePID_Wall = now;
    }
    break;
  }

  // ---------------------------------------------------------
  // MODO CARRERA (Teledirigido)
  // ---------------------------------------------------------
  case MODO_CARRERA: {
    // Usamos giroMotor del código 1 para control manual preciso
    giroMotor(rx_duty_fwd, rx_duty_bwd);
    girarRuedas(rx_angulo);
    break;
  }

  // ---------------------------------------------------------
  // MODO FRENADA (Código 2 - PD de Precisión)
  // ---------------------------------------------------------
  case MODO_FRENADA: {
    servoLidar.write(ANGULO_LIDAR_FRENTE);

    // 1. Rearme (Si nos alejamos 50cm del objetivo reactivamos)
    if (dist_actual > (DISTANCIA_OBJETIVO + 50)) {
      frenada_completada = false;
    }

    if (frenada_completada) {
      detenerMotor();
    } 
    else {
      // --- CÁLCULO PD ---
      unsigned long now = millis();
      double dt = (now - last_time_pd_brake) / 1000.0;
      if (dt <= 0) dt = 0.001;

      // Error: Positivo = Lejos, Negativo = Nos pasamos
      int error = dist_actual - DISTANCIA_OBJETIVO;

      // Velocidad: (diferencia distancia) / tiempo
      double velocidad = (dist_actual - last_dist_pd_brake) / dt;

      // Filtro de ruido derivativo (si da saltos absurdos de >3m/s)
      if (abs(velocidad) > 300) velocidad = 0;

      // ECUACIÓN PD: Salida = Kp*Error + Kd*Velocidad
      int salida_control = (Kp_Brake * error) + (Kd_Brake * velocidad);

      // Actualizar memoria
      last_time_pd_brake = now;
      last_dist_pd_brake = dist_actual;

      // --- ZONA MUERTA (STOP) ---
      // Aumentamos tolerancia a 5cm para motores DC simples
      if (abs(error) <= 5) {
        detenerMotor();
        frenada_completada = true; // Bloqueo
      } 
      else {
        // --- ACTUADORES ---
        if (salida_control > 0) {
          // AVANZAR
          int pwm_base = 40; 
          // Si estamos muy cerca (<15cm error), reducimos la fuerza base
          if (error < 15) pwm_base = 30;

          int pwm_final = salida_control + pwm_base;
          avanzarMotor(pwm_final);
        } 
        else {
          // RETROCEDER (Frenado activo por Kd)
          int pwm_freno = abs(salida_control);
          // Limitamos freno marcha atrás para evitar rebote brusco
          pwm_freno = constrain(pwm_freno, 0, 90);
          retrocederMotor(pwm_freno);
        }
      }
    }
    girarRuedas(DIR_CENTRO);
    break;
  }

  case MODO_ESPERA: 
    detenerMotor();
    break;
  }

  // 4. ENVIAR DATOS A SCADA
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 100) {
    lastSendTime = millis();
    if (client && client.connected()) {
      int d_frontal = (MODO_ACTUAL == MODO_FRENADA || MODO_ACTUAL == MODO_CARRERA) ? dist_actual : 0;
      int d_lateral = (MODO_ACTUAL == MODO_SEGUIMIENTO) ? dist_actual : 0;
      
      sim_vbat = 20.0; // Valor simulado fijo como en codigo 1
      sim_ibat = 3;

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