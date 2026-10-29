#include <Arduino.h>

//Configuration des broches
#define PIN_STBY 13

// Ordre des moteurs : [0] = Gauche Avant, [1] = Droit Avant, [2] = Gauche Arrière, [3] = Droit Arrière
const int PIN_PWM[4] = {27, 25, 18, 21};
const int PIN_DIR[4] = {26, 33, 19, 22};

const char* NOM_MOTEUR[4] = {
    "Gauche Avant (GAv)",
    "Droit Avant (DAv)",
    "Gauche Arrière (GAr)",
    "Droit Arrière (DAr)"
};

//Fonction générique pour piloter un moteur
void piloterMoteur(int id, int pwm, bool sensAvant) 
{
    digitalWrite(PIN_DIR[id], sensAvant ? HIGH : LOW);
    analogWrite(PIN_PWM[id], constrain(pwm, 0, 255));
}

// Fonction pour stopper tous les moteurs
void stopperTousMoteurs() 
{
    for (int i = 0; i < 4; i++)
    {
        analogWrite(PIN_PWM[i], 0);
        digitalWrite(PIN_DIR[i], LOW);
    }
}

void setup() 
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   TEST MATÉRIEL DES MOTEURS (ESP32)");
    Serial.println("=============================================");

    // Configuration des broches en sortie
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH); // Déblocage des ponts en H

    for (int i = 0; i < 4; i++) {
        pinMode(PIN_PWM[i], OUTPUT);
        pinMode(PIN_DIR[i], OUTPUT);
    }

    stopperTousMoteurs();
    Serial.println("--> Broches initialisées & Standby désactivé.");
}

void loop() {
    // TEST 1 : Test individuel de chaque moteur (Avant puis Arrière)
    Serial.println("\n[TEST 1] Vérification individuelle de chaque roue");
    for (int i = 0; i < 4; i++) {
        Serial.printf("--> Test : %s | Marche AVANT (PWM 120)", NOM_MOTEUR[i]);
        piloterMoteur(i, 120, true);
        delay(1500);

        Serial.println("STOP");
        stopperTousMoteurs();
        delay(500);

        Serial.printf("--> Test : %s | Marche ARRIÈRE (PWM 120)", NOM_MOTEUR[i]);
        piloterMoteur(i, 120, false);
        delay(1500);

        Serial.println("STOP");
        stopperTousMoteurs();
        delay(1000);
    }

    // TEST 2 : Avance globale (4 roues ensemble)
    Serial.println("\n[TEST 2] Avance globale (4 moteurs en AVANT à 50% PWM)");
    for (int i = 0; i < 4; i++) {
        piloterMoteur(i, 128, true);
    }
    delay(2000);

    stopperTousMoteurs();
    Serial.println("STOP");
    delay(1000);


    // TEST 3 : Recul global (4 roues ensemble)
    Serial.println("\n[TEST 3] Recul global (4 moteurs en ARRIÈRE à 50% PWM)");
    for (int i = 0; i < 4; i++) 
    {
        piloterMoteur(i, 128, false);
    }
    delay(2000);

    stopperTousMoteurs();
    Serial.println("STOP");

    Serial.println("\n=== Fin de la séquence de test. Redémarrage dans 5 secondes.");
    delay(5000);
}