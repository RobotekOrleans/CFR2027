#include <Arduino.h>
#include <unity.h>

const uint8_t TRIG_PINS[4] = {27, 26, 25, 4};
const uint8_t ECHO_PINS[4] = {32, 33, 34, 35};

float mesurerDistance(int index) {
    digitalWrite(TRIG_PINS[index], LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PINS[index], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PINS[index], LOW);
    unsigned long duration = pulseIn(ECHO_PINS[index], HIGH, 25000);
    return (duration > 0) ? (duration * 0.0343) / 2.0 : -1.0;
}

void test_capteur_0(void) {
    float dist = mesurerDistance(0);
    TEST_ASSERT_GREATER_THAN(2.0, dist); // Valide si la distance est > 2 cm
}

void setup() {
    delay(2000); // Temps d'ouverture du port série
    UNITY_BEGIN();
    for (int i = 0; i < 4; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
    }
    RUN_TEST(test_capteur_0);
    UNITY_END();
}

void loop() {}