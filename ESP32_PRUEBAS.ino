#include <Arduino.h>
#include <ESP32Servo.h>
#include <WiFi.h>
#include <math.h>

// ==========================================
// 1. CONFIGURACIÓN WIFI (SCADA)
// ==========================================
const char *ssid = "coche_lab3";   // PON AQUÍ TU RED
const char *password = "abcd1234"; // PON AQUÍ TU PASS
WiFiServer tcpServer(23);          // Puerto 23
WiFiClient client;                 // Cliente global

// Variables para simular batería
float sim_vbat = 6;
float sim_ibat = 5;

// ==========================================
// 2. CONFIGURACIÓN HARDWARE COCHE
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

// --- CONFIGURACIÓN FÍSICA Y SERVO ---
const double ANGLO_SENSOR = 45.0;
const double RAD_CONVERSION = 3.14159 / 180.0;

// Cambiar a true si las ruedas giran hacia la pared en vez de alejarse
const bool INVERTIR_SERVO = false;

// Distancias de Seguridad
uint16_t DISTANCIA_OBJETIVO = 20; // Paramos a 20cm para asegurar
const uint16_t MARGEN_SEGUIMIENTO = 1;

// --- MODOS DE FUNCIONAMIENTO ---
enum ModoCoche
{
  MODO_FRENADA,     // 0
  MODO_CARRERA,     // 1
  MODO_SEGUIMIENTO, // 2
  MODO_ESPERA       // 3
};

// !!!!!!! SELECCIONA AQUÍ EL MODO !!!!!!!
ModoCoche MODO_ACTUAL = MODO_ESPERA;

// Variables Globales de Estado
uint16_t dist_actual = 0; // Lectura Lidar
int pwm_actual = 0;       // PWM actual del motor
int sentido_actual = 1;   // 1 = Adelante, 0 = Atrás
int angulo_servo_actual = 90;

// Variables recibidas del SCADA (Buffers)
int rx_angulo = 90;
int rx_duty_fwd = 0;
int rx_duty_bwd = 0;
int rx_dist_frenada = 10;
int rx_btn_carrera = 0;
int rx_btn_frenada = 0;
int rx_btn_seguimiento = 0;

// Variables PID
double Kp = 2.0;
double Ki = 0.05;
double Kd = 4.0;
double setPoint = 30.0; // Distancia deseada (cm)

// Variables PID internas
double error, lastError = 0, integral = 0;
double pidOutput = 0; // Resultado del PID (-45 a 45 grados lógicos)

// Filtro Lidar
const int NUM_LECTURAS = 3;
double lecturas[NUM_LECTURAS];
int indiceLectura = 0;
double totalFiltro = 0;
const double MAX_DIST_LOGICA = 80.0;

// Velocidades (Escala 0-255 para coincidir con tu función avanzarMotor)
int velocidadBase = 160;  // Velocidad en recta
int velocidadMinima = 90; // Velocidad en curva cerrada

// Timing
unsigned long lastTimePID = 0;
const int sampleTime = 50;

// ==========================================
// 3. FUNCIONES PARSER (RECIBIR DATOS SCADA)
// ==========================================
void procesarTramaSCADA(String trama)
{
  int idx1 = trama.indexOf(';');
  if (idx1 == -1)
    return;
  rx_angulo = trama.substring(0, idx1).toInt();

  int idx2 = trama.indexOf(';', idx1 + 1);
  if (idx2 == -1)
    return;
  rx_duty_fwd = trama.substring(idx1 + 1, idx2).toInt();

  int idx3 = trama.indexOf(';', idx2 + 1);
  if (idx3 == -1)
    return;
  rx_duty_bwd = trama.substring(idx2 + 1, idx3).toInt();

  int idx4 = trama.indexOf(';', idx3 + 1);
  if (idx4 == -1)
    return;
  rx_dist_frenada = trama.substring(idx3 + 1, idx4).toInt();
  if (rx_dist_frenada > 0)
    DISTANCIA_OBJETIVO = rx_dist_frenada;

  int idx5 = trama.indexOf(';', idx4 + 1);
  if (idx5 == -1)
    return;
  rx_btn_carrera = trama.substring(idx4 + 1, idx5).toInt();

  int idx6 = trama.indexOf(';', idx5 + 1);
  if (idx6 == -1)
    return;
  rx_btn_frenada = trama.substring(idx5 + 1, idx6).toInt();

  rx_btn_seguimiento = trama.substring(idx6 + 1).toInt();

  // Cambio de modo desde SCADA
  if (rx_btn_carrera == 1)
  {
    if (MODO_ACTUAL != MODO_CARRERA)
    {
      MODO_ACTUAL = MODO_CARRERA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
    }
  }
  else if (rx_btn_frenada == 1)
  {
    if (MODO_ACTUAL != MODO_FRENADA)
    {
      MODO_ACTUAL = MODO_FRENADA;
      servoLidar.write(ANGULO_LIDAR_FRENTE);
    }
  }
  else if (rx_btn_seguimiento == 1)
  {
    if (MODO_ACTUAL != MODO_SEGUIMIENTO)
    {
      MODO_ACTUAL = MODO_SEGUIMIENTO;
      servoLidar.write(ANGULO_LIDAR_PARED);
    }
  }
}

// ==========================================
// 4. FUNCIONES HARDWARE (CORREGIDAS)
// ==========================================

// --- LÓGICA INVERTIDA PARA CORREGIR EL SENTIDO ---
void avanzarMotor(int velocidad)
{
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad;
  sentido_actual = 1;

  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, velocidad); // Velocidad por LPWM

  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, 0); // 0 por RPWM
}

void retrocederMotor(int velocidad)
{
  velocidad = constrain(velocidad, 0, 255);
  pwm_actual = velocidad;
  sentido_actual = 0;

  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, 0); // 0 por LPWM

  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, velocidad); // Velocidad por RPWM
}

void detenerMotor()
{
  pwm_actual = 0;
  digitalWrite(EN_R_PIN, LOW);
  digitalWrite(EN_L_PIN, LOW);
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void girarRuedas(int angulo)
{
  servoDireccion.write(angulo);
  angulo_servo_actual = angulo;
}

// Lidar Non-Blocking
bool readOnePacket(HardwareSerial &ser, uint16_t &dist)
{
  if (ser.available() < 9)
    return false;
  if (ser.read() == 0x59)
  {
    if (ser.peek() == 0x59)
    {
      ser.read();
      uint8_t buf[7];
      int checksum = 0x59 + 0x59;
      for (int i = 0; i < 7; i++)
      {
        buf[i] = ser.read();
        if (i < 6)
          checksum += buf[i];
      }
      if ((uint8_t)(checksum & 0xFF) == buf[6])
      {
        dist = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
        return true;
      }
    }
  }
  return false;
}

void actualizarLidarRapido()
{
  uint16_t tempDist = 0;
  while (tfLidar1.available() >= 9)
  {
    if (readOnePacket(tfLidar1, tempDist))
    {
      if (tempDist < 1200)
      { // Filtramos errores grandes
        dist_actual = tempDist;
      }
    }
  }
}
double filtrarLidar(double nuevaLectura)
{
  totalFiltro = totalFiltro - lecturas[indiceLectura];
  // Si la lectura es 0 o gigante (hueco), limitamos para que el PID reaccione rápido pero suave
  if (nuevaLectura > MAX_DIST_LOGICA || nuevaLectura == 0)
  {
    nuevaLectura = MAX_DIST_LOGICA;
  }
  lecturas[indiceLectura] = nuevaLectura;
  totalFiltro = totalFiltro + lecturas[indiceLectura];

  indiceLectura++;
  if (indiceLectura >= NUM_LECTURAS)
    indiceLectura = 0;

  return totalFiltro / NUM_LECTURAS;
}

void gestionarVelocidadDinamica(double pidOut)
{
  // Cuanto más cerrada sea la curva (mayor pidOut), más lento vamos
  double giroAbs = abs(pidOut); // Valor entre 0 y 45 aprox

  // Mapeamos: 0 grados -> Vel Base, 45 grados -> Vel Minima
  int velObjetivo = map(giroAbs, 0, 45, velocidadBase, velocidadMinima);

  // Llamamos a TU función de motor
  avanzarMotor(velObjetivo);
}
// ==========================================
// SETUP
// ==========================================
void setup()
{
  Serial.begin(115200);

  // --- 1. INICIAR HARDWARE ---
  tfLidar1.begin(115200, SERIAL_8N1, 16, 17);

  pinMode(RPWM_PIN, OUTPUT);
  pinMode(EN_R_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);
  pinMode(EN_L_PIN, OUTPUT);

  servoLidar.setPeriodHertz(50);
  servoLidar.attach(PIN_SERVO_LIDAR, 500, 2400);
  servoDireccion.setPeriodHertz(50);
  servoDireccion.attach(PIN_SERVO_DIRECCION, 1160, 1900);

  // Estado Inicial
  detenerMotor();
  girarRuedas(DIR_CENTRO);

  // Inicializar array filtro
  for (int i = 0; i < NUM_LECTURAS; i++)
    lecturas[i] = 0;

  // --- 2. CONEXIÓN WIFI (COMENTADO PARA PRUEBAS OFFLINE) ---

  /*    //Modo STA
  Serial.println("\nConectando a WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  */
  // Modo AP
  Serial.println("\nAbriendo Zona Wifi...");
  WiFi.softAP(ssid, password);

  /*
  // Esperamos conexión
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
*/
  /*    //Modo STA
  Serial.println("\nWiFi Conectado!");
  Serial.print("IP ESP32: ");
  Serial.println(WiFi.localIP());
  */

  // Alternativa: Modo AP
  Serial.println(WiFi.softAPIP());

  tcpServer.begin();
  Serial.println("Servidor TCP puerto 23 iniciado");
}

// ==========================================
// LOOP PRINCIPAL
// ==========================================
void loop()
{

  // 1. GESTIÓN WIFI (Solo funcionará si descomentas el setup)
  if (tcpServer.hasClient())
  {
    if (client.connected())
      client.stop();
    client = tcpServer.available();
    Serial.println("-> SCADA (LabVIEW) Conectado");
  }

  if (client && client.connected())
  {
    if (client.available())
    {
      String tramaRecibida = client.readStringUntil('*');
      tramaRecibida.trim();
      procesarTramaSCADA(tramaRecibida);
    }
  }

  // 2. LEER SENSORES
  actualizarLidarRapido();

  // 3. LÓGICA DE CONTROL
  switch (MODO_ACTUAL)
  {

  // --- SEGUIMIENTO ---
  case MODO_SEGUIMIENTO:
  {

    unsigned long now = millis();

    // 1. LEER SENSOR SIEMPRE (Para no perder paquetes serial)
    actualizarLidarRapido();

    // 2. CICLO PID (Cada 50ms)
    if (now - lastTimePID >= sampleTime)
    {
      double dt = (double)(now - lastTimePID) / 1000.0;

      // --- A. FILTRADO Y GEOMETRÍA ---
      double inputRaw = (double)dist_actual;
      double inputFiltrado = filtrarLidar(inputRaw);

      // Distancia perpendicular (Cateto opuesto)
      double distPerpendicular = inputFiltrado * sin(ANGLO_SENSOR * RAD_CONVERSION);

      // --- B. CÁLCULO DEL ERROR ---
      // PARED A LA IZQUIERDA:
      // Si dist > 30 (lejos) -> Error POSITIVO -> Girar a la Izquierda para acercarse
      // Si dist < 30 (cerca) -> Error NEGATIVO -> Girar a la Derecha para alejarse
      // NOTA: Ajusta el signo (+/-) según hacia dónde gire tu servo con angulos > 90
      error = distPerpendicular - setPoint;

      // --- C. ALGORITMO PID ---
      integral += error * dt;
      double pTerm = Kp * error;
      double dTerm = Kd * (error - lastError) / dt;
      double preOutput = pTerm + (Ki * integral) + dTerm;

      // Anti-Windup (Limitamos la salida lógica a +/- 45 grados de corrección)
      double limiteLogico = 45.0;
      if (preOutput > limiteLogico)
      {
        integral -= error * dt;
        preOutput = limiteLogico;
      }
      else if (preOutput < -limiteLogico)
      {
        integral -= error * dt;
        preOutput = -limiteLogico;
      }
      pidOutput = preOutput;

      // --- D. APLICAR A TUS FUNCIONES ---

      // 1. Calcular ángulo final (90 +/- corrección)
      // Asumimos: < 90 es Izquierda, > 90 es Derecha (o viceversa según tu servo)
      // Si tu coche gira al revés, cambia el '+' por un '-'
      int anguloFinal = DIR_CENTRO + (int)pidOutput;

      // Llamamos a TU función de servo
      girarRuedas(anguloFinal);

      // 2. Controlar velocidad (Frenar en curvas)
      // Llamamos a TU función de motor a través de la lógica dinámica
      gestionarVelocidadDinamica(pidOutput);

      // Actualizar variables
      lastError = error;
      lastTimePID = now;
    }
    break;
  }
  // --- CARRERA (TELEDIRIGIDO) ---
  case MODO_CARRERA:
  {
    girarRuedas(rx_angulo);
    if (rx_duty_fwd > 5)
      avanzarMotor(rx_duty_fwd);
    else if (rx_duty_bwd > 5)
      retrocederMotor(rx_duty_bwd);
    else
      detenerMotor();
    break;
  }

    // --- FRENADA ANTICIPADA CON BLOQUEO
  case MODO_FRENADA:
  {

    servoLidar.write(ANGULO_LIDAR_FRENTE);
    Serial.println(DISTANCIA_OBJETIVO);
    static bool frenada_completada = false;

    // ============================================================
    // AJUSTES DE CALIBRACIÓN FINA
    // ============================================================
    float target_usuario = DISTANCIA_OBJETIVO * 1.1132 + 0.2856;

    // 1. CORRECCIÓN DEL OFFSET (EL CAMBIO IMPORTANTE)
    // Disparo del freno
    int anticipacion = 20;

    // 2. RAMPA
    int dist_inicio_rampa = 180;

    // 3. VELOCIDADES
    // Bajamos un pelín la mínima para que llegue muriendo y el frenazo sea seco.
    // Si el coche se cala con este valor (al bajar la bateria o algo) subir a 55 o 60
    int vel_minima_llegada = 50;
    int vel_crucero = 140;

    // Distancia real restante hasta el punto de "GOLPE DE FRENO"
    float distancia_al_trigger = dist_actual - (target_usuario + anticipacion);

    // Rearme: Margen de seguridad para volver a activar
    if (dist_actual > (target_usuario + 60))
      frenada_completada = false;

    if (frenada_completada)
    {
      detenerMotor();
    }
    else
    {
      // --- FASE 1: GOLPE DE FRENO (DISPARO) ---
      if (distancia_al_trigger <= 0)
      {

        // TIEMPO DE FRENADO ADAPTATIVO
        // Si venía rápido, frenamos más tiempo.
        int tiempo_freno = 300;
        if (pwm_actual > 100)
          tiempo_freno = 500;

        retrocederMotor(255); // Freno a fondo
        delay(tiempo_freno);

        detenerMotor();
        frenada_completada = true;
      }

      // --- FASE 2: RAMPA DE APROXIMACIÓN ---
      else if (distancia_al_trigger <= dist_inicio_rampa)
      {
        // Mapeamos para que baje suavemente hasta vel_minima
        int pwm_calculado = map(distancia_al_trigger, 0, dist_inicio_rampa, vel_minima_llegada, vel_crucero);

        // Constrain es vital para no enviar valores negativos o absurdos
        pwm_calculado = constrain(pwm_calculado, vel_minima_llegada, vel_crucero);

        avanzarMotor(pwm_calculado);
      }

      // --- FASE 3: VELOCIDAD CRUCERO ---
      else
      {
        avanzarMotor(vel_crucero);
      }
    }
    girarRuedas(DIR_CENTRO);
    break;
  }
  }

  // 4. ENVIAR DATOS A SCADA (Si hay conexión)
  static unsigned long lastSendTime = 0;
  if (millis() - lastSendTime > 100)
  {
    lastSendTime = millis();
    if (client && client.connected())
    {
      int d_frontal = (MODO_ACTUAL == MODO_FRENADA || MODO_ACTUAL == MODO_CARRERA) ? dist_actual : 0;
      int d_lateral = (MODO_ACTUAL == MODO_SEGUIMIENTO) ? dist_actual : 0;
      sim_vbat = 20.0;
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