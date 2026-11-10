#include <Arduino.h>

// Broches du Moteur 1 (Gauche Avant - GAv)
#define PIN_IN1 26
#define PIN_IN2 27
#define PIN_PWM 13

// Fonction d'arrêt complet du moteur 1
void stopperMoteur1() {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
    analogWrite(PIN_PWM, 0);
}

// Fonction de pilotage à 3 fils (IN1, IN2, PWM)
void piloterMoteur1(int vitesse, bool sensAvant) {
    int pwmVal = constrain(vitesse, 0, 255);

    if (pwmVal == 0) {
        stopperMoteur1();
        return;
    }

    if (sensAvant) {
        // Marche Avant : IN1 = HIGH, IN2 = LOW
        digitalWrite(PIN_IN1, HIGH);
        digitalWrite(PIN_IN2, LOW);
    } else {
        // Marche Arrière : IN1 = LOW, IN2 = HIGH
        digitalWrite(PIN_IN1, LOW);
        digitalWrite(PIN_IN2, HIGH);
    }

    analogWrite(PIN_PWM, pwmVal);
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=============================================");
    Serial.println("   TEST UNIQUE : MOTEUR 1 (GAUCHE AVANT)");
    Serial.println("=============================================");

    // Configuration des 3 broches en sortie
    pinMode(PIN_IN1, OUTPUT);
    pinMode(PIN_IN2, OUTPUT);
    pinMode(PIN_PWM, OUTPUT);

    // Sécurité au démarrage
    stopperMoteur1();

    Serial.println("--> Broches 26 (IN1), 27 (IN2) et 13 (PWM) initialisees.");
    Serial.println("--> Pause de 3 secondes avant le lancement...");
    delay(3000);
}

void loop() {
    Serial.println("\n--- Debut du cycle Moteur 1 ---");

    // 1. MARCHE AVANT
    Serial.println("--> MARCHE AVANT (PWM 150 - 2 secondes)");
    piloterMoteur1(150, true);
    delay(2000);

    // 2. STOP
    Serial.println("--> STOP (1 seconde)");
    stopperMoteur1();
    delay(1000);

    // 3. MARCHE ARRIERE
    Serial.println("--> MARCHE ARRIERE (PWM 150 - 2 secondes)");
    piloterMoteur1(150, false);
    delay(2000);

    // 4. STOP
    Serial.println("--> STOP (Pause 3 secondes)");
    stopperMoteur1();
    delay(3000);
}