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

// --- VARIABLES DEL CONTROLADOR PD  MODO FRENADA ---
// AJUSTA ESTOS DOS VALORES PARA CALIBRAR
float Kp_Frenada = 1.2;   // PROPORCIONAL: "Fuerza del muelle". Si es bajo, no llega. Si es alto, oscila.
float Kd_Frenada = 0.5;  // DERIVATIVO: "Freno de aire". Evita que te estrelles si vienes rápido.

// Variables de memoria para el cálculo matemático
unsigned long last_time_pd = 0;
int last_dist_pd = 0;


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
int velocidadBase = 100;  // Velocidad en recta
int velocidadMinima = 50; // Velocidad en curva cerrada

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
  else
  {
    MODO_ACTUAL = MODO_ESPERA;
  }
}

// ==========================================
// 4. FUNCIONES HARDWARE (CORREGIDAS)
// ==========================================

// --- LÓGICA INVERTIDA PARA CORREGIR EL SENTIDO ---
void giroMotor(int velocidad_f, int velocidad_b)
{

  velocidad_f = map(velocidad_f, 0, 100, 0, 255);
  velocidad_b = map(velocidad_b, 0, 100, 0, 255);
  
  int aux_pwm = velocidad_f - velocidad_b;
  if(aux_pwm < 0) {
    sentido_actual = 0;
  }
  else {
    sentido_actual = 1;
  }
  pwm_actual = abs(aux_pwm);
  if (pwm_actual <= 5) {
    detenerMotor();
  }
  else {
    digitalWrite(EN_L_PIN, HIGH);
    analogWrite(LPWM_PIN, velocidad_f); // Velocidad por LPWM

    digitalWrite(EN_R_PIN, HIGH);
    analogWrite(RPWM_PIN, velocidad_b); // 0 por RPWM
  }
  
}
   
void avanzarMotor(int velocidad)
{
  velocidad = map(velocidad, 0, 100, 0, 255);
  pwm_actual = velocidad;
  sentido_actual = 1;

  digitalWrite(EN_L_PIN, HIGH);
  analogWrite(LPWM_PIN, velocidad); // Velocidad por LPWM

  digitalWrite(EN_R_PIN, HIGH);
  analogWrite(RPWM_PIN, 0); // 0 por RPWM
}

void retrocederMotor(int velocidad)
{
  velocidad = map(velocidad, 0, 100, 0, 255);
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
    /*
    girarRuedas(rx_angulo);
    if (rx_duty_fwd > 5)
      avanzarMotor(rx_duty_fwd);
    else if (rx_duty_bwd > 5)
      retrocederMotor(rx_duty_bwd);
    else
      detenerMotor();
    */
    giroMotor(rx_duty_fwd, rx_duty_bwd);
    girarRuedas(rx_angulo);
    break;
  }

    // --- FRENADA ANTICIPADA CON BLOQUEO
      case MODO_FRENADA: {
        servoLidar.write(ANGULO_LIDAR_FRENTE);
        
        static bool frenada_completada = false;

        // 1. Rearme (Si nos alejamos 50cm del objetivo reactivamos)
        if (dist_actual > (DISTANCIA_OBJETIVO + 50)) {
             frenada_completada = false;
        }

        if (frenada_completada) {
            detenerMotor();
        } 
        else {
            // --- MATEMÁTICAS PD ---
            unsigned long now = millis();
            double dt_frenada = (now - last_time_pd) / 1000.0; 
            if (dt_frenada <= 0) dt_frenada = 0.001; 

            // Error: Positivo si estamos LEJOS, Negativo si nos hemos pasado
            int error_frenada = dist_actual - DISTANCIA_OBJETIVO;

            // Velocidad: Negativa si nos acercamos
            double velocidad = (dist_actual - last_dist_pd) / dt_frenada;

            // Lógica de seguridad para derivadas locas (ruido del sensor)
            // Si la velocidad calculada es absurda (>200cm/s), la ignoramos
            if (abs(velocidad) > 300) velocidad = 0;

            // FÓRMULA PD
            int salida_control = (Kp_Frenada * error_frenada) + (Kd_Frenada * velocidad);

            // Actualizamos memoria
            last_time_pd = now;
            last_dist_pd = dist_actual;

            // --- ZONA MUERTA (LA CLAVE PARA QUE PARE) ---
            // Si el error es menor de 4cm, consideramos que hemos llegado.
            // Esto evita que el coche intente corregir milímetros y se vuelva loco.
           
// --- ZONA MUERTA AUMENTADA ---
            // Aumentamos a 5cm. Si está entre 15 y 25 (siendo 20 el objetivo), SE ACABÓ.
            // Los motores DC no tienen precisión para menos.
            if (abs(error_frenada) <= 5) {
                detenerMotor();
                frenada_completada = true; // Candado inmediato
            }
            else {
                // GESTIÓN DEL MOTOR
                if (salida_control > 0) {
                    // AVANZAR
                    int pwm_base = 40; // Bajamos la base (antes 45 o 60)
                    
                    // TRUCO NUEVO: Si estamos muy cerca (menos de 15cm de error),
                    // quitamos la fuerza base para que no se pase de largo.
                    if (error_frenada < 15) pwm_base = 30; 

                    int pwm_final = salida_control + pwm_base; 
                    avanzarMotor(pwm_final);
                }
                else {
                   // RETROCEDER
                   // Aquí la Kd actúa frenando.
                   int pwm_freno = abs(salida_control);
                   
                   // Limitamos mucho el freno marcha atrás para evitar el rebote
                   pwm_freno = constrain(pwm_freno, 0, 90); 
                   
                   retrocederMotor(pwm_freno);
                }
            }
        }
        girarRuedas(DIR_CENTRO);
        break;
    }

    case MODO_ESPERA: { break; }
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