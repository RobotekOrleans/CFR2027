#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

//==============Configuration Wi-Fi & OTA================
const char *WIFI_SSID = "Robotek 2.4"; //Modifier le nom du wifi
const char *WIFI_PASS = "RobotekIsTheBest"; //Modifier le mdp du wifi
const char *OTA_HOSTNAME = "robot-esp32";

// Mettre à 1 dès que les 4 capteurs ultrasons HC-SR04 sont branchés
#define ULTRASONS_ACTIFS 0

//==============Branchements (23 broches)================
// Liaison UART2 avec Raspberry Pi 4
#define RXD2 16
#define TXD2 17

// Moteurs (Ponts en H L298N) & Encodeurs
// Index : [0]=GAv, [1]=DAv, [2]=GAr, [3]=DAr
const int PIN_PWM[4] = {13, 12, 5, 2};
const int PIN_IN1[4] = {26, 33, 19, 22};
const int PIN_IN2[4] = {27, 25, 18, 21};
const int PIN_ENC[4] = {4, 14, 32, 15};

// Ultrasons (1 TRIG commun sur GPIO 23 + 4 ECHO sur broches input-only)
#define NB_CAPTEURS 4
#define TRIG_PIN_COMMUN 23
const uint8_t ECHO_PINS[NB_CAPTEURS] = {34, 35, 36, 39};
const float DISTANCE_URGENCE_CM = 20.0f;

//=============Communication avec Raspberry Pi===============
const byte b_numChars = 48;
char c_receivedChars[b_numChars];
boolean bol_newData = false;

int n_motorSpeeds[4] = {0, 0, 0, 0};                    // [0]=GAv, [1]=DAv, [2]=GAr, [3]=DAr
bool bol_ignoreSensor[4] = {false, false, false, false}; // [0]=Avant, [1]=Droit, [2]=Gauche, [3]=Arrière
QueueHandle_t motorQueue;
TaskHandle_t h_ThreadDeplacement = NULL; // Handle pour suspendre le Cœur 1 pendant un flash OTA

// Watchdog de sécurité communication RPi
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long ul_lastComTime = 0;
bool bol_watchdogTriggered = false;

//=============Structures PID===============
struct MotorPID {
  float Kp = 2.0f;
  float Ki = 0.4f;
  float Kd = 0.05f;

  int targetSpeedTicks = 0; // Consigne en ticks par période de 20 ms
  int prevDirection = 0;
  long lastTicks = 0;

  float errorSum = 0.0f;
  float lastError = 0.0f;
  int pwmOutput = 0;
};
MotorPID pid[4];

//=============Commandes Matérielles & Sécurité===============
void commanderMoteur(int index, int pwmSigne)
{
  if (pwmSigne > 0)
  {
    digitalWrite(PIN_IN1[index], HIGH);
    digitalWrite(PIN_IN2[index], LOW);
    analogWrite(PIN_PWM[index], constrain(pwmSigne, 0, 255));
  }
  else if (pwmSigne < 0)
  {
    digitalWrite(PIN_IN1[index], LOW);
    digitalWrite(PIN_IN2[index], HIGH);
    analogWrite(PIN_PWM[index], constrain(-pwmSigne, 0, 255));
  }
  else
  {
    digitalWrite(PIN_IN1[index], LOW);
    digitalWrite(PIN_IN2[index], LOW);
    analogWrite(PIN_PWM[index], 0);
  }
}

void arretUrgenceMoteurs()
{
  for (int i = 0; i < 4; i++)
  {
    n_motorSpeeds[i] = 0;
  }
  if (motorQueue != NULL)
  {
    xQueueOverwrite(motorQueue, &n_motorSpeeds);
  }
}

//=============Initialisation Wi-Fi & ArduinoOTA===============
void configurerWiFiEtOTA()
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.print("Connexion Wi-Fi");
  unsigned long debutTentative = millis();
  // Attente max de 5 secondes pour ne pas bloquer le robot si le Wi-Fi est coupé
  while (WiFi.status() != WL_CONNECTED && (millis() - debutTentative < 5000))
  {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.printf("\nWi-Fi connecte ! IP : %s | Hostname : %s.local\n",
                  WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
  }
  else
  {
    Serial.println("\nWi-Fi non disponible au boot (reconnexion auto en arriere-plan).");
  }

  ArduinoOTA.setHostname(OTA_HOSTNAME);
  // Optionnel : décommenter pour protéger le flash par mot de passe
  // ArduinoOTA.setPassword("robot123");

  ArduinoOTA.onStart([]() {
    // Sécurité critique : arrêt immédiat des 4 roues et gel de la tâche PID avant écriture Flash
    arretUrgenceMoteurs();
    if (h_ThreadDeplacement != NULL)
    {
      vTaskSuspend(h_ThreadDeplacement);
    }
    for (int i = 0; i < 4; i++)
    {
      commanderMoteur(i, 0);
    }
    Serial.println("\n[OTA] Debut de la mise a jour Wi-Fi -> Moteurs coupes.");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Mise a jour terminee ! Redemarrage...");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progression : %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("\n[OTA] Erreur [%u] -> Redemarrage de securite.\n", error);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

//=============Parsing Communication Raspberry Pi===============
void readSerialData()
{
  static boolean bol_ReceiveInProgress = false;
  static byte b_index = 0;
  const char c_StartMarker = '<';
  const char c_EndMarker = '>';
  char c_Data;

  while (Serial2.available() > 0 && bol_newData == false)
  {
    c_Data = Serial2.read();
    if (bol_ReceiveInProgress == true)
    {
      if (c_Data == c_StartMarker)
      {
        b_index = 0;
      }
      else if (c_Data != c_EndMarker)
      {
        c_receivedChars[b_index] = c_Data;
        b_index++;
        if (b_index >= b_numChars - 1)
        {
          b_index = b_numChars - 1;
        }
      }
      else
      {
        c_receivedChars[b_index] = '\0';
        bol_ReceiveInProgress = false;
        b_index = 0;
        bol_newData = true;
      }
    }
    else if (c_Data == c_StartMarker)
    {
      bol_ReceiveInProgress = true;
    }
  }
}

void parseData()
{
  if (bol_newData == true)
  {
    char c_typeCmd = c_receivedChars[0];

    // 1. Consignes de vitesse : <V,GAv,DAv,GAr,DAr> (en ticks / 20 ms)
    if (c_typeCmd == 'V')
    {
      int v0, v1, v2, v3;
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &v0, &v1, &v2, &v3) == 5)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;

        n_motorSpeeds[0] = v0;
        n_motorSpeeds[1] = v1;
        n_motorSpeeds[2] = v2;
        n_motorSpeeds[3] = v3;

        xQueueOverwrite(motorQueue, &n_motorSpeeds);
      }
    }
    // 2. Masquage des capteurs ultrasons : <I,i0,i1,i2,i3>
    else if (c_typeCmd == 'I')
    {
      int n_i0, n_i1, n_i2, n_i3;
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &n_i0, &n_i1, &n_i2, &n_i3) == 5)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;

        bol_ignoreSensor[0] = (n_i0 == 1);
        bol_ignoreSensor[1] = (n_i1 == 1);
        bol_ignoreSensor[2] = (n_i2 == 1);
        bol_ignoreSensor[3] = (n_i3 == 1);
      }
    }
    // 3. Réglage des gains PID depuis la RPi : <P,Kp,Ki,Kd>
    else if (c_typeCmd == 'P')
    {
      float newKp, newKi, newKd;
      if (sscanf(c_receivedChars, "%c,%f,%f,%f", &c_typeCmd, &newKp, &newKi, &newKd) == 4)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;

        for (int i = 0; i < 4; i++)
        {
          pid[i].Kp = newKp;
          pid[i].Ki = newKi;
          pid[i].Kd = newKd;
          pid[i].errorSum = 0.0f;
          pid[i].lastError = 0.0f;
        }
      }
    }

    bol_newData = false;
  }
}

void verifierWatchdog()
{
  if ((millis() - ul_lastComTime > WATCHDOG_TIMEOUT_MS) && !bol_watchdogTriggered)
  {
    arretUrgenceMoteurs();
    Serial2.println("<WATCHDOG,STOP>");
    bol_watchdogTriggered = true;
  }
}

//=============Gestion 4 ultrasons===============
volatile unsigned long echoStart[NB_CAPTEURS] = {0};
volatile unsigned long echoEnd[NB_CAPTEURS] = {0};
volatile bool b_echoReceived[NB_CAPTEURS] = {false, false, false, false};
float f_distancesUS[NB_CAPTEURS] = {999.0f, 999.0f, 999.0f, 999.0f};

void IRAM_ATTR echoInterrupt()
{
  unsigned long now = micros();
  for (int i = 0; i < NB_CAPTEURS; i++)
  {
    if (digitalRead(ECHO_PINS[i]) == HIGH)
    {
      echoStart[i] = now;
    }
    else if (echoStart[i] != 0)
    {
      echoEnd[i] = now;
      b_echoReceived[i] = true;
      echoStart[i] = 0;
    }
  }
}

void DeclancherPing()
{
  digitalWrite(TRIG_PIN_COMMUN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_COMMUN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_COMMUN, LOW);
}

void traiterUltrason()
{
  for (int i = 0; i < NB_CAPTEURS; i++)
  {
    if (b_echoReceived[i] == true)
    {
      b_echoReceived[i] = false;
      unsigned long duration = echoEnd[i] - echoStart[i];

      if (duration > 0 && duration < 25000)
      {
        float distance = duration * 0.034f / 2.0f;
        f_distancesUS[i] = distance;

        if (distance < DISTANCE_URGENCE_CM && !bol_ignoreSensor[i])
        {
          bool moteursEnMouvement = (n_motorSpeeds[0] != 0 || n_motorSpeeds[1] != 0 ||
                                     n_motorSpeeds[2] != 0 || n_motorSpeeds[3] != 0);
          if (moteursEnMouvement)
          {
            arretUrgenceMoteurs();
            Serial2.printf("<URGENCE,%d,%.1f>\n", i, distance);
          }
        }
      }
    }
  }
}

void envoyerTelemetrieUS()
{
  Serial2.printf("<US,%.1f,%.1f,%.1f,%.1f>\n",
                 f_distancesUS[0], f_distancesUS[1], f_distancesUS[2], f_distancesUS[3]);
}

//=============Encodeurs===============
volatile long tickCounters[4] = {0, 0, 0, 0};
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR Encoder0() { tickCounters[0]++; }
void IRAM_ATTR Encoder1() { tickCounters[1]++; }
void IRAM_ATTR Encoder2() { tickCounters[2]++; }
void IRAM_ATTR Encoder3() { tickCounters[3]++; }

//=============Threads FreeRTOS===============
void ThreadComRasbe(void *parameter) // Cœur 0 : Wi-Fi OTA, UART2 RPi & Ultrasons
{
  unsigned long dernierPing = 0;
  unsigned long derniereTelemetrie = 0;
  ul_lastComTime = millis();

  while (1)
  {
    ArduinoOTA.handle(); // Écoute les demandes de flash Wi-Fi sans bloquer

    readSerialData();
    parseData();
    verifierWatchdog();

#if ULTRASONS_ACTIFS
    if (millis() - dernierPing >= 50)
    {
      DeclancherPing();
      dernierPing = millis();
    }
    traiterUltrason();

    if (millis() - derniereTelemetrie >= 100)
    {
      envoyerTelemetrieUS();
      derniereTelemetrie = millis();
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ThreadDeplacement(void *parameter) // Cœur 1 : Boucle PID stricte à 50 Hz (20 ms)
{
  int n_consignesMoteurs[4] = {0, 0, 0, 0};
  TickType_t LastWakeTime = xTaskGetTickCount();
  const TickType_t Frequency = pdMS_TO_TICKS(20);

  while (1)
  {
    if (xQueueReceive(motorQueue, &n_consignesMoteurs, 0) == pdTRUE)
    {
      for (int i = 0; i < 4; i++)
      {
        int newDir = (n_consignesMoteurs[i] > 0) ? 1 : ((n_consignesMoteurs[i] < 0) ? -1 : 0);
        if (newDir != pid[i].prevDirection)
        {
          pid[i].errorSum = 0.0f;
          pid[i].lastError = 0.0f;
          pid[i].prevDirection = newDir;
        }
        pid[i].targetSpeedTicks = n_consignesMoteurs[i];
      }
    }

    for (int i = 0; i < 4; i++)
    {
      portENTER_CRITICAL(&encMux);
      long currentTicks = tickCounters[i];
      portEXIT_CRITICAL(&encMux);

      long measuredSpeed = currentTicks - pid[i].lastTicks;
      pid[i].lastTicks = currentTicks;

      if (pid[i].targetSpeedTicks == 0)
      {
        pid[i].errorSum = 0.0f;
        pid[i].lastError = 0.0f;
        pid[i].pwmOutput = 0;
        commanderMoteur(i, 0);
        continue;
      }

      float targetAbs = (float)abs(pid[i].targetSpeedTicks);
      float error = targetAbs - (float)measuredSpeed;

      pid[i].errorSum = constrain(pid[i].errorSum + error, -800.0f, 800.0f);
      float derivative = error - pid[i].lastError;
      pid[i].lastError = error;

      float output = (pid[i].Kp * error) + (pid[i].Ki * pid[i].errorSum) + (pid[i].Kd * derivative);
      pid[i].pwmOutput = constrain((int)output, 0, 255);

      int finalPWM = (pid[i].targetSpeedTicks > 0) ? pid[i].pwmOutput : -pid[i].pwmOutput;
      commanderMoteur(i, finalPWM);
    }

    vTaskDelayUntil(&LastWakeTime, Frequency);
  }
}

//=============Setup===============
void setup()
{
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

  motorQueue = xQueueCreate(1, sizeof(n_motorSpeeds));

  // Initialisation Moteurs & Encodeurs (en premier pour sécuriser les sorties à LOW)
  for (int i = 0; i < 4; i++)
  {
    pinMode(PIN_PWM[i], OUTPUT);
    pinMode(PIN_IN1[i], OUTPUT);
    pinMode(PIN_IN2[i], OUTPUT);
    commanderMoteur(i, 0);

    pinMode(PIN_ENC[i], INPUT_PULLUP);
  }
  attachInterrupt(digitalPinToInterrupt(PIN_ENC[0]), Encoder0, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC[1]), Encoder1, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC[2]), Encoder2, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC[3]), Encoder3, RISING);

#if ULTRASONS_ACTIFS
  pinMode(TRIG_PIN_COMMUN, OUTPUT);
  digitalWrite(TRIG_PIN_COMMUN, LOW);
  for (int i = 0; i < NB_CAPTEURS; i++)
  {
    pinMode(ECHO_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[i]), echoInterrupt, CHANGE);
  }
#endif

  // Démarrage Wi-Fi et serveur OTA
  configurerWiFiEtOTA();

  // Création des Threads sur les deux cœurs
  xTaskCreatePinnedToCore(ThreadComRasbe, "CommunicationRasbery", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ThreadDeplacement, "Deplacement", 10000, NULL, 1, &h_ThreadDeplacement, 1);

  Serial.println("ESP32 pret (UART2 + OTA Wi-Fi actifs).");
}

//=============Loop===============
void loop()
{
  vTaskDelete(NULL);
}