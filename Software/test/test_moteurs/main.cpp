#include <Arduino.h>

// Broches des 4 moteurs : [0]=GAv, [1]=DAv, [2]=GAr, [3]=DAr
const int PIN_PWM[4] = {13, 12, 5, 2};
const int PIN_IN1[4] = {26, 33, 19, 22};
const int PIN_IN2[4] = {27, 25, 18, 21};

// Puissance PWM du test (entre 0 et 255)
const int VITESSE_TEST = 200;

// Fonction d'arrêt complet des 4 moteurs
void stopperTousLesMoteurs() {
    for (int i = 0; i < 4; i++) {
        digitalWrite(PIN_IN1[i], LOW);
        digitalWrite(PIN_IN2[i], LOW);
        analogWrite(PIN_PWM[i], 0);
    }
}

// Fonction de pilotage simultané des 4 moteurs (IN1, IN2, PWM)
void piloterTousLesMoteurs(int vitesse, bool sensAvant) {
    int pwmVal = constrain(vitesse, 0, 255);

    if (pwmVal == 0) {
        stopperTousLesMoteurs();
        return;
    }

    for (int i = 0; i < 4; i++) {
        if (sensAvant) {
            // Marche Avant : IN1 = HIGH, IN2 = LOW
            digitalWrite(PIN_IN1[i], HIGH);
            digitalWrite(PIN_IN2[i], LOW);
        } else {
            // Marche Arrière : IN1 = LOW, IN2 = HIGH
            digitalWrite(PIN_IN1[i], LOW);
            digitalWrite(PIN_IN2[i], HIGH);
        }
        analogWrite(PIN_PWM[i], pwmVal);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   TEST SIMULTANE : 4 MOTEURS (GAv, DAv, GAr, DAr)");
    Serial.println("=============================================");

    // Configuration des 12 broches en sortie
    for (int i = 0; i < 4; i++) {
        pinMode(PIN_IN1[i], OUTPUT);
        pinMode(PIN_IN2[i], OUTPUT);
        pinMode(PIN_PWM[i], OUTPUT);
    }

    // Sécurité au démarrage
    stopperTousLesMoteurs();

    Serial.println("--> Broches des 4 ponts en H initialisees.");
    Serial.println("--> Pause de 3 secondes avant le lancement...");
    delay(3000);
}

void loop() {
    Serial.println("\n--- Debut du cycle 4 Moteurs ---");

    // 1. MARCHE AVANT
    Serial.printf("--> MARCHE AVANT (PWM %d - 2 secondes)\n", VITESSE_TEST);
    piloterTousLesMoteurs(VITESSE_TEST, true);
    delay(2000);

    // 2. STOP
    Serial.println("--> STOP (1 seconde)");
    stopperTousLesMoteurs();
    delay(1000);

    // 3. MARCHE ARRIERE
    Serial.printf("--> MARCHE ARRIERE (PWM %d - 2 secondes)\n", VITESSE_TEST);
    piloterTousLesMoteurs(VITESSE_TEST, false);
    delay(2000);

    // 4. STOP
    Serial.println("--> STOP (Pause 3 secondes)");
    stopperTousLesMoteurs();
    delay(3000);
}