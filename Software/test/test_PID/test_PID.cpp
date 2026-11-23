#include <Arduino.h>

//==============Configuration Mode Test================
#define TEST_SANS_RPI 1       // 1 = Saisie manuelle sur PC (USB) | 0 = Liaison UART2 avec RPi 4
#define ULTRASONS_BRANCHES 0  // 0 = Ultrasons débranchés | 1 = Activer les 4 capteurs HC-SR04

//==============Branchements================
// Liaison UART2 avec Raspberry Pi
#define RXD2 16
#define TXD2 17

// Moteurs (Ponts en H L298N) & Encodeurs
// Index : [0]=GAv, [1]=DAv, [2]=GAr, [3]=DAr
const int PIN_PWM[4] = {13, 12, 5, 2};
const int PIN_IN1[4] = {26, 33, 19, 22};
const int PIN_IN2[4] = {27, 25, 18, 21};
const int PIN_ENC[4] = {4, 14, 32, 15};

// Ultrasons (1 TRIG commun sur GPIO 23 + 4 ECHO sur broches GPI)
#define NB_CAPTEURS 4
#define TRIG_PIN_COMMUN 23
const uint8_t ECHO_PINS[NB_CAPTEURS] = {34, 35, 36, 39};

//=============Structures & Variables Globales===============
struct MotorCommand {
  bool directPWM; // true = PWM brut (-255..255) via <M>, false = consigne PID (ticks/20ms) via <V>
  int values[4];
};

struct MotorPID {
  float Kp = 4.0f;
  float Ki = 0.8f;
  float Kd = 0.05f;

  int targetSpeedTicks = 0; // Consigne en ticks par période de 20 ms
  int prevDirection = 0;
  long lastTicks = 0;

  float errorSum = 0.0f;
  float lastError = 0.0f;
  int pwmOutput = 0;
};

MotorPID pid[4];
QueueHandle_t motorQueue;

const byte b_numChars = 48;
char c_receivedChars[b_numChars];
boolean bol_newData = false;

bool bol_ignoreSensor[4] = {false, false, false, false};
bool bol_moteursActifs = false;

// Watchdog de sécurité communication RPi
const unsigned long WATCHDOG_TIMEOUT_MS = 500;
unsigned long ul_lastComTime = 0;
bool bol_watchdogTriggered = false;

#if TEST_SANS_RPI
bool bol_watchdogActif = false; // Désactivé en saisie manuelle PC
#else
bool bol_watchdogActif = true;
#endif

int n_moteurAffichage = -1; // -1 = pas d'affichage continu, 0..3 = affiche ticks et PWM du moteur choisi

//=============Fonctions de Commande & Communication===============
void envoyerConsignesQueue(bool directPWM, int v0, int v1, int v2, int v3)
{
  MotorCommand cmd;
  cmd.directPWM = directPWM;
  cmd.values[0] = v0;
  cmd.values[1] = v1;
  cmd.values[2] = v2;
  cmd.values[3] = v3;
  bol_moteursActifs = (v0 != 0 || v1 != 0 || v2 != 0 || v3 != 0);
  xQueueOverwrite(motorQueue, &cmd);
}

void arretUrgenceMoteurs()
{
  envoyerConsignesQueue(false, 0, 0, 0, 0);
}

void readSerialData(Stream &port)
{
  static boolean bol_ReceiveInProgress = false;
  static byte b_index = 0;
  const char c_StartMarker = '<';
  const char c_EndMarker = '>';
  char c_Data;

  while (port.available() > 0 && bol_newData == false)
  {
    c_Data = port.read();
    if (bol_ReceiveInProgress == true)
    {
      if (c_Data == c_StartMarker)
      {
        b_index = 0; // Réinitialise si une nouvelle trame commence
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
    int v0, v1, v2, v3;

    // 1. Consigne de vitesse avec PID : <V,GAv,DAv,GAr,DAr> (en ticks/20ms)
    if (c_typeCmd == 'V')
    {
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &v0, &v1, &v2, &v3) == 5)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;
        envoyerConsignesQueue(false, v0, v1, v2, v3);

        Serial.printf("OK [PID] -> GAv:%d | DAv:%d | GAr:%d | DAr:%d ticks/20ms\n", v0, v1, v2, v3);
      }
      else
      {
        Serial.println("Erreur format V -> Attendu : <V,v0,v1,v2,v3>");
      }
    }
    // 2. Consigne PWM directe sans PID : <M,pwm0,pwm1,pwm2,pwm3> (entre -255 et 255)
    else if (c_typeCmd == 'M')
    {
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &v0, &v1, &v2, &v3) == 5)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;
        envoyerConsignesQueue(true, v0, v1, v2, v3);

        Serial.printf("OK [PWM Direct] -> GAv:%d | DAv:%d | GAr:%d | DAr:%d\n", v0, v1, v2, v3);
      }
      else
      {
        Serial.println("Erreur format M -> Attendu : <M,p0,p1,p2,p3>");
      }
    }
    // 3. Modification des gains PID à la volée : <P,Kp,Ki,Kd>
    else if (c_typeCmd == 'P')
    {
      float newKp, newKi, newKd;
      if (sscanf(c_receivedChars, "%c,%f,%f,%f", &c_typeCmd, &newKp, &newKi, &newKd) == 4)
      {
        for (int i = 0; i < 4; i++)
        {
          pid[i].Kp = newKp;
          pid[i].Ki = newKi;
          pid[i].Kd = newKd;
          pid[i].errorSum = 0.0f;
          pid[i].lastError = 0.0f;
        }
        Serial.printf("OK [Gains PID] -> Kp:%.3f | Ki:%.3f | Kd:%.3f\n", newKp, newKi, newKd);
      }
      else
      {
        Serial.println("Erreur format P -> Attendu : <P,Kp,Ki,Kd>");
      }
    }
    // 4. Afficher les mesures en continu d'un moteur : <G,0> à <G,3> (ou <G,-1> pour stopper)
    else if (c_typeCmd == 'G')
    {
      int m;
      if (sscanf(c_receivedChars, "%c,%d", &c_typeCmd, &m) == 2 && m >= -1 && m <= 3)
      {
        n_moteurAffichage = m;
        if (m == -1) Serial.println("Affichage continu coupe.");
      }
      else
      {
        Serial.println("Erreur format G -> Attendu : <G,0> (0 a 3, ou -1 pour couper)");
      }
    }
    // 5. Masques capteurs ultrasons : <I,i0,i1,i2,i3>
    else if (c_typeCmd == 'I')
    {
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &v0, &v1, &v2, &v3) == 5)
      {
        ul_lastComTime = millis();
        bol_watchdogTriggered = false;
        bol_ignoreSensor[0] = (v0 == 1);
        bol_ignoreSensor[1] = (v1 == 1);
        bol_ignoreSensor[2] = (v2 == 1);
        bol_ignoreSensor[3] = (v3 == 1);
        Serial.printf("OK [Masques US] -> %d,%d,%d,%d\n", v0, v1, v2, v3);
      }
    }
    else
    {
      Serial.println("Commande inconnue. Utilise <M,...>, <V,...>, <P,Kp,Ki,Kd> ou <G,0..3>");
    }

    bol_newData = false;
  }
}

void verifierWatchdog()
{
  if (!bol_watchdogActif) return;

  if ((millis() - ul_lastComTime > WATCHDOG_TIMEOUT_MS) && !bol_watchdogTriggered)
  {
    arretUrgenceMoteurs();
    Serial.println("WATCHDOG : Perte de communication ! Arret des moteurs.");
    bol_watchdogTriggered = true;
  }
}

//=============Gestion 4 ultrasons===============
volatile unsigned long echoStart[NB_CAPTEURS] = {0};
volatile unsigned long echoEnd[NB_CAPTEURS] = {0};
volatile bool b_echoReceived[NB_CAPTEURS] = {false, false, false, false};
float f_distancesUS[NB_CAPTEURS] = {999.0, 999.0, 999.0, 999.0};

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

        if (distance < 20.0f && !bol_ignoreSensor[i] && bol_moteursActifs)
        {
          arretUrgenceMoteurs();
          Serial.printf("<URGENCE,Capteur_%d,%.1fcm>\n", i, distance);
#if !TEST_SANS_RPI
          Serial2.printf("<URGENCE,%d>\n", i);
#endif
        }
      }
    }
  }
}

void envoyerTelemetrieUS()
{
#if !TEST_SANS_RPI
  Serial2.printf("<US,%.1f,%.1f,%.1f,%.1f>\n",
                 f_distancesUS[0], f_distancesUS[1], f_distancesUS[2], f_distancesUS[3]);
#endif
}

//=============Déplacement & Encodeurs===============
volatile long tickCounters[4] = {0, 0, 0, 0};
portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR Encoder0() { tickCounters[0]++; }
void IRAM_ATTR Encoder1() { tickCounters[1]++; }
void IRAM_ATTR Encoder2() { tickCounters[2]++; }
void IRAM_ATTR Encoder3() { tickCounters[3]++; }

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

//=============Threads FreeRTOS===============
void ThreadComRasbe(void *parameter)
{
  unsigned long dernierPing = 0;
  unsigned long derniereTelemetrie = 0;
  ul_lastComTime = millis();

  while (1)
  {
#if TEST_SANS_RPI
    readSerialData(Serial);  // Saisie manuelle depuis le PC
#else
    readSerialData(Serial2); // Réception depuis la Raspberry Pi 4
#endif
    parseData();
    verifierWatchdog();

#if ULTRASONS_BRANCHES
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

void ThreadDeplacement(void *parameter)
{
  MotorCommand cmdActuelle = {false, {0, 0, 0, 0}};
  TickType_t LastWakeTime = xTaskGetTickCount();
  const TickType_t Frequency = pdMS_TO_TICKS(20); // Période fixe de 20 ms (50 Hz)

  while (1)
  {
    if (xQueueReceive(motorQueue, &cmdActuelle, 0) == pdTRUE)
    {
      for (int i = 0; i < 4; i++)
      {
        int newDir = (cmdActuelle.values[i] > 0) ? 1 : ((cmdActuelle.values[i] < 0) ? -1 : 0);
        if (newDir != pid[i].prevDirection || cmdActuelle.directPWM)
        {
          pid[i].errorSum = 0.0f;
          pid[i].lastError = 0.0f;
          pid[i].prevDirection = newDir;
        }
        pid[i].targetSpeedTicks = cmdActuelle.values[i];
      }
    }

    long measuredSpeeds[4] = {0, 0, 0, 0};

    for (int i = 0; i < 4; i++)
    {
      portENTER_CRITICAL(&encMux);
      long currentTicks = tickCounters[i];
      portEXIT_CRITICAL(&encMux);

      long measuredSpeed = currentTicks - pid[i].lastTicks;
      pid[i].lastTicks = currentTicks;
      measuredSpeeds[i] = measuredSpeed;

      // Mode PWM direct (<M,...>) : applique le PWM sans correction PID
      if (cmdActuelle.directPWM)
      {
        int pwmBrut = constrain(cmdActuelle.values[i], -255, 255);
        pid[i].pwmOutput = abs(pwmBrut);
        commanderMoteur(i, pwmBrut);
        continue;
      }

      // Mode PID (<V,...>) : arrêt strict si consigne == 0
      if (pid[i].targetSpeedTicks == 0)
      {
        pid[i].errorSum = 0.0f;
        pid[i].lastError = 0.0f;
        pid[i].pwmOutput = 0;
        commanderMoteur(i, 0);
        continue;
      }

      // Calcul PID en valeur absolue
      float targetAbs = (float)abs(pid[i].targetSpeedTicks);
      float error = targetAbs - (float)measuredSpeed;

      pid[i].errorSum = constrain(pid[i].errorSum + error, -300.0f, 300.0f);
      float derivative = error - pid[i].lastError;
      pid[i].lastError = error;

      float output = (pid[i].Kp * error) + (pid[i].Ki * pid[i].errorSum) + (pid[i].Kd * derivative);
      pid[i].pwmOutput = constrain((int)output, 0, 255);

      int finalPWM = (pid[i].targetSpeedTicks > 0) ? pid[i].pwmOutput : -pid[i].pwmOutput;
      commanderMoteur(i, finalPWM);
    }

    // Affichage en temps réel si activé avec <G,0..3>
    if (n_moteurAffichage >= 0 && n_moteurAffichage <= 3)
    {
      int m = n_moteurAffichage;
      Serial.printf("Consigne:%d,Mesure_M%d:%ld,PWM_M%d:%d\n",
                    abs(pid[m].targetSpeedTicks), m, measuredSpeeds[m], m, pid[m].pwmOutput);
    }

    vTaskDelayUntil(&LastWakeTime, Frequency);
  }
}

//=============Setup===============
void setup()
{
  Serial.begin(115200);
#if !TEST_SANS_RPI
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif

  motorQueue = xQueueCreate(1, sizeof(MotorCommand));

  // Initialisation Moteurs & Encodeurs
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

#if ULTRASONS_BRANCHES
  pinMode(TRIG_PIN_COMMUN, OUTPUT);
  digitalWrite(TRIG_PIN_COMMUN, LOW);
  for (int i = 0; i < NB_CAPTEURS; i++)
  {
    pinMode(ECHO_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[i]), echoInterrupt, CHANGE);
  }
#endif

  // Création des Threads
  xTaskCreatePinnedToCore(ThreadComRasbe, "CommunicationRasbery", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ThreadDeplacement, "Deplacement", 10000, NULL, 1, NULL, 1);

  delay(500);
  Serial.println("\n=== ESP32 PRET (SAISIE MANUELLE USB) ===");
  Serial.println("Commandes disponibles :");
  Serial.println("  <M,p0,p1,p2,p3>  -> PWM direct (-255 a 255) sans PID");
  Serial.println("  <V,v0,v1,v2,v3>  -> Vitesse cible avec PID (ticks / 20 ms)");
  Serial.println("  <P,Kp,Ki,Kd>     -> Modifier les gains PID en direct");
  Serial.println("  <G,0> (0..3/-1)  -> Afficher Consigne / Mesure / PWM d'un moteur");
}

//=============Loop===============
void loop()
{
  vTaskDelete(NULL);
}