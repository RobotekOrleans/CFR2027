#include <Arduino.h>

#define NB_CAPTEURS 4
#define TRIG_PIN_COMMUN 23
const uint8_t ECHO_PINS[NB_CAPTEURS] = {34, 35, 36, 39};

const char* NOM_ULTRASONS[4] = {
    "Gauche Avant (GAv)",
    "Droit Avant (DAv)",
    "Gauche Arrière (GAr)",
    "Droit Arrière (DAr)"
};


float mesurerDistance(int index) {
    digitalWrite(TRIG_PIN_COMMUN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN_COMMUN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN_COMMUN, LOW);
    unsigned long duration = pulseIn(ECHO_PINS[index], HIGH, 25000);
    return (duration > 0) ? (duration * 0.0343) / 2.0 : -1.0;
}

void setup() 
{ 
    Serial.begin(115200);
    Serial.println("\n=============================================");
    Serial.println("   TEST MATÉRIEL DES Ultrasons (ESP32)");
    Serial.println("=============================================");

    for (int i = 0; i < 4; i++) {
        pinMode(TRIG_PIN_COMMUN, OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
    }
}

void loop() 
{
    // TEST 1 : Test individuel de chaque ultrasons
    Serial.println("\n[TEST 1] Vérification individuelle de chaque ultrasons");
    for (int i = 0; i < 4; i++)
    {
        float distance = mesurerDistance(i);

        if (distance > 0) 
        {
            Serial.printf("Capteur %d [%s] -> Distance = %.1f cm\n", i, NOM_ULTRASONS[i], distance);
        } 
        else 
        {
            Serial.printf("Capteur %d [%s] -> Pas d'écho\n", i, NOM_ULTRASONS[i]);
        }
        delay(50);
    }
    delay(1000);
}