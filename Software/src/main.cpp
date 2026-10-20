#include <Arduino.h>

//=============Communication avec Rasbery===============
const byte b_numChars = 32;
char c_receivedChars[b_numChars];
boolean bol_newData = false;
void readSerialData() //Lecture de la data envoyé par la rasbery sur la liaison série sous la forme <V,MoteurAvantGauche, MoteurAvantDroit, MoteurArriereGauche, MoteurArriereGauche>
{
  static boolean bol_ReceiveInProgress = false; //false = pas de données transmises, true = données entrain d'être transmises
  static byte b_index = 0;
  char c_StartMarker = '<'; //Marqueur de début de la transmission
  char c_EndMarker = '>';   //Marqueur de fin de la transmission
  char c_Data; //Data transmise

  while(Serial.available() > 0 && bol_newData == false)
  {
    c_Data = Serial.read();
    if(bol_ReceiveInProgress == true)
    {
      if(c_Data != c_EndMarker)
      {
        c_receivedChars[b_index] = c_Data;
        b_index++;
      }
      else
      {
        c_receivedChars[b_index] = '\0';
        bol_ReceiveInProgress = false;
        b_index = 0;
        bol_newData = true;
      }
    }
    else if(c_Data == c_StartMarker)
    {
      bol_ReceiveInProgress = true;
    }
  }
}

int n_motorSpeeds[4] = {0, 0, 0, 0}; // Tableau des vitesses : [0] = GAv, [1] = DAv, [2] = GAr, [3] = DAr
bool bol_ignoreSensor[4] = {false, false, false, false}; // Tableau des capteurs : [0] = Avant, [1] = Droit, [2] = Gauche, [3] = Arrière
QueueHandle_t motorQueue; // Queue pour transmettre les 4 consignes de vitesse du Cœur 0 au Cœur 1
// Watchdog de sécurité communication RPi
const unsigned long WATCHDOG_TIMEOUT_MS = 500; // Timeout de 500 ms
unsigned long ul_lastComTime = 0; // Time du dernier message reçu
bool bol_watchdogTriggered = false; // Flag pour éviter de spammer le Serial
void parseData() //Répartition des valeurs reçu dans les variables de chaque moteur et capteurs
{
  if (bol_newData == true) //Si newData == true on a reçu des nouvelles données à trier
  {
    char c_typeCmd = c_receivedChars[0];
    if (c_typeCmd == 'V') //Si le premier caractère est V c'est des données moteurs
    {
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &n_motorSpeeds[0], &n_motorSpeeds[1],&n_motorSpeeds[2],&n_motorSpeeds[3]) == 5) 
      {
        ul_lastComTime = millis();     // Reset Watchdog
        bol_watchdogTriggered = false;

        // On envoie le tableau de vitesses dans la Queue vers le ThreadDeplacement
        xQueueSend(motorQueue, &n_motorSpeeds, 0);

        //Retour des données dans le monitor
        Serial.print("✅ OK -> Commande: "); Serial.print(c_typeCmd);
        Serial.print(" | Moteur GAv: "); Serial.print(n_motorSpeeds[0]);
        Serial.print(" | Moteur DAv: "); Serial.println(n_motorSpeeds[1]);
        Serial.print(" | Moteur GAr: "); Serial.print(n_motorSpeeds[2]);
        Serial.print(" | Moteur DAr: "); Serial.println(n_motorSpeeds[3]);
        Serial.println("🚗 Vitesses mises à jour");
      }
    }
    else if(c_typeCmd == 'I') //Si le premier caractère est I c'est des données capteurs
    {
      int n_i0, n_i1, n_i2, n_i3;
      if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &n_i0, &n_i1, &n_i2, &n_i3) == 5) 
      {
        ul_lastComTime = millis();     // Reset Watchdog 
        bol_watchdogTriggered = false;

        bol_ignoreSensor[0] = (n_i0 == 1); //True si i1 == 1, si true = ignorer les données du capteur.
        bol_ignoreSensor[1] = (n_i1 == 1);
        bol_ignoreSensor[2] = (n_i2 == 1);
        bol_ignoreSensor[3] = (n_i3 == 1);
        //Retour des données dans le monitor
        Serial.print("✅ OK -> Commande: "); Serial.print(c_typeCmd);
        Serial.print(" | Capteur 1 : "); Serial.print(n_i0);
        Serial.print(" | Capteur 2 : "); Serial.println(n_i1);
        Serial.print(" | Capteur 3 : "); Serial.print(n_i2);
        Serial.print(" | Capteur 4 : "); Serial.println(n_i3);
        Serial.println("🛡️ Masques US mis à jour par la RPi !");
      }
    }
     else // Si il y a une erreur sur le format d'envois des données : 
     {
      Serial.println("❌ Erreur de syntaxe (Format attendu : <V,X0,X1,X2,X3>) ou <I,i1,i2,i3,i4>");
    }   
    bol_newData = false; //La transimission est terminé on remet le newData à false
  }
}
void verifierWatchdog() //Watchdog pour arrêter le mouvement si la communication avec la RPI est coupé
{
  // Si le temps écoulé dépasse le timeout et que la sécurité n'a pas encore déclenché
  if ((millis() - ul_lastComTime > WATCHDOG_TIMEOUT_MS) && !bol_watchdogTriggered)
  {
    for (int i = 0; i < 4; i++)
    {
      n_motorSpeeds[i] = 0;
    }

    // Écrasement prioritaire dans la Queue vers Cœur 1
    xQueueOverwrite(motorQueue, &n_motorSpeeds);

    Serial.println("🚨 WATCHDOG : Perte de communication RPi ! Arrêt des moteurs.");
    bol_watchdogTriggered = true; // Sécurité activée
  }
}

//=============Gestion 4 ultrasons===============
#define NB_CAPTEURS 4
const uint8_t TRIG_PINS[NB_CAPTEURS] = {27, 26, 25, 4};
const uint8_t ECHO_PINS[NB_CAPTEURS] = {32, 33, 34, 35};
volatile unsigned long echoStart[NB_CAPTEURS] = {0};
volatile unsigned long echoEnd[NB_CAPTEURS] = {0};
volatile bool b_echoReceived[NB_CAPTEURS] = {false, false, false, false};
float f_distancesUS[NB_CAPTEURS] = {0.0, 0.0, 0.0, 0.0};// Tableau global pour stocker les dernières distances (en cm) pour la télémétrie
uint8_t currentSensor = 0; // Index du capteur en cours

void IRAM_ATTR echoInterrupt() //Interuption lors que le signal est de retour
{
  // On regarde quelle broche vient de changer d'état pour être sûr de ne pas se tromper
  if(digitalRead(ECHO_PINS[0]) == HIGH) echoStart[0] = micros();
  else if(digitalRead(ECHO_PINS[0]) == LOW && echoStart[0] != 0) { echoEnd[0] = micros(); b_echoReceived[0] = true; }

  if(digitalRead(ECHO_PINS[1]) == HIGH) echoStart[1] = micros();
  else if(digitalRead(ECHO_PINS[1]) == LOW && echoStart[1] != 0) { echoEnd[1] = micros(); b_echoReceived[1] = true; }

  if(digitalRead(ECHO_PINS[2]) == HIGH) echoStart[2] = micros();
  else if(digitalRead(ECHO_PINS[2]) == LOW && echoStart[2] != 0) { echoEnd[2] = micros(); b_echoReceived[2] = true; }

  if(digitalRead(ECHO_PINS[3]) == HIGH) echoStart[3] = micros();
  else if(digitalRead(ECHO_PINS[3]) == LOW && echoStart[3] != 0) { echoEnd[3] = micros(); b_echoReceived[3] = true; }
}
void DeclancherPing()//Envois du signal
{
  digitalWrite(TRIG_PINS[currentSensor], LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PINS[currentSensor], HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PINS[currentSensor], LOW);
}
void traiterUltrason()//Traitement du signal pour obtenir la distance
{
    if(b_echoReceived[currentSensor] == true)
    {
      b_echoReceived[currentSensor] = false;
      unsigned long duration = echoEnd[currentSensor] - echoStart[currentSensor]; //Calcul du temps mis par l'onde pour faire l'aller retour
      if(duration > 0 && duration < 25000) //Sécurité
      {
        float distance = duration * 0.034/2; //Calcul de la distance parcouru par l'onde à l'allé
        f_distancesUS[currentSensor] = distance; // Stockage pour la télémétrie
        Serial.print("Capteur ");
        Serial.print(currentSensor);
        Serial.print(" -> Distance = ");
        Serial.print(distance);
        Serial.println(" cm");

        if (distance < 20.0 && !bol_ignoreSensor[currentSensor]) //Envois un message d'arrêt aux moteurs si distance < 20 et si la rasberry n'a pas demandé la desactivation du capteur
        {
          //Remise à zéro immédiate du tableau de vitesses
          for (int i = 0; i < 4; i++)
          {
            n_motorSpeeds[i] = 0;
          }

          //Écrasement prioritaire dans la Queue FreeRTOS pour le Cœur 1
          xQueueOverwrite(motorQueue, &n_motorSpeeds);

          //A envoyer à la rasberry
          Serial.print("<⚠️,URGENCE, Capteur ");
          Serial.print(currentSensor);
          Serial.println(">");
        }
      }
    }
    currentSensor = (currentSensor + 1) % NB_CAPTEURS; // Utilisation du modulo pour boucler de 0 à 3
}
void envoyerTelemetrieUS()//Fonction pour envoyer régulièrement la télémétrie à la RPi
{
  Serial.print("<US,");
  Serial.print(f_distancesUS[0], 1); Serial.print(",");
  Serial.print(f_distancesUS[1], 1); Serial.print(",");
  Serial.print(f_distancesUS[2], 1); Serial.print(",");
  Serial.print(f_distancesUS[3], 1);
  Serial.println(">");
}

//=============Threads===============
void ThreadComRasbe(void * parameter) //Thread 1 : Communication RPI et gestion ultrasons
{
  unsigned long dernierPing = 0;
  unsigned long derniereTelemetrie = 0;
  ul_lastComTime = millis(); // Initialisation au démarrage
  while(1)
  {
    readSerialData(); //Lecture de la liaison série avec la RPI
    parseData(); //Traitement des données reçu par la liaison série avec la RPI
    verifierWatchdog(); // Contrôle permanent du timeout
    
    if (millis() - dernierPing > 50) //On envoit une onde sur les ultrasons toutes les 50ms sans bloquer le programme
    {
      DeclancherPing(); 
      dernierPing = millis();
    }
    traiterUltrason(); // On traite le retour du signal s'il est prêt

    if (millis() - derniereTelemetrie > 100) // Envoi de la télémétrie à la RPi toutes les 100ms pour ne pas bloquer la liaison série
    {
      envoyerTelemetrieUS();
      derniereTelemetrie = millis();
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}
void ThreadDeplacement(void * parameter) //Thread 2 : Gestion des moteurs
{
  int n_consignesMoteurs[4] = {0, 0, 0, 0};
  while(1)
  {
    if (xQueueReceive(motorQueue, &n_consignesMoteurs, 0) == pdTRUE) // On regarde si de nouvelles consignes sont arrivées dans la Queue
    {
      // Réception des nouvelles consignes sur le Cœur 1 !
      Serial.print("⚙️ Cœur 1 a reçu les nouvelles consignes : ");
      Serial.print(n_consignesMoteurs[0]); Serial.print(", ");
      Serial.print(n_consignesMoteurs[1]); Serial.print(", ");
      Serial.print(n_consignesMoteurs[2]); Serial.print(", ");
      Serial.println(n_consignesMoteurs[3]);
    }

    // Implémenter le PID et la gestion des PWM
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

//=============Setup===============
void setup() {
  Serial.begin(115200);

  // Création d'une Queue pouvant contenir 1 élément (le tableau de 4 int)
  motorQueue = xQueueCreate(1, sizeof(n_motorSpeeds));

  // Configuration des broches pour les 4 capteurs & Attachement des interruptions sur les 4 pins ECHO
  for (int i = 0; i < NB_CAPTEURS; i++) {
    pinMode(TRIG_PINS[i], OUTPUT);
    pinMode(ECHO_PINS[i], INPUT);
    attachInterrupt(digitalPinToInterrupt(ECHO_PINS[i]), echoInterrupt, CHANGE); //Interuption
  }

  //Création des Threads
  xTaskCreatePinnedToCore(ThreadComRasbe, "CommunicationRasbery", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ThreadDeplacement, "Deplacement", 10000, NULL, 1, NULL, 1);
}

//=============Loop===============
void loop() {
  vTaskDelete(NULL);
}
