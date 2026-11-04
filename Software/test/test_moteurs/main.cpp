#include <Arduino.h>

//Configuration des broches UART2 pour la Raspberry Pi 4
#define RXD2 16
#define TXD2 17

int n_motorSpeeds[4] = {0, 0, 0, 0};      // [0]=GAv, [1]=DAv, [2]=GAr, [3]=DAr
bool bol_ignoreSensor[4] = {false, false, false, false}; // [0]=Av, [1]=Dr, [2]=Ga, [3]=Ar

const byte b_numChars = 32;
char c_receivedChars[b_numChars];
boolean bol_newData = false;

//Envoi des données vers la Raspberry Pi
void envoyerDonnees(byte val1, byte val2, byte val3, byte val4) {
    // Utilisation de Serial2.print pour envoyer du texte ASCII (<US,50,40,50,60>)
    Serial2.print("<US,");
    Serial2.print(val1);
    Serial2.print(",");
    Serial2.print(val2);
    Serial2.print(",");
    Serial2.print(val3);
    Serial2.print(",");
    Serial2.print(val4);
    Serial2.println(">");

    // Copie sur le moniteur série USB du PC pour le débogage
    Serial.printf("[TX -> Pi] <US,%d,%d,%d,%d>\n", val1, val2, val3, val4);
}

//Lecture de l'UART2 (Raspberry Pi)
void readSerialData() {
    static boolean bol_ReceiveInProgress = false;
    static byte b_index = 0;
    char c_StartMarker = '<';
    char c_EndMarker = '>';
    char c_Data;

    while (Serial2.available() > 0 && !bol_newData) {
        c_Data = Serial2.read();

        if (bol_ReceiveInProgress) {
            if (c_Data != c_EndMarker) {
                // Sécurité contre le débordement de mémoire (Buffer Overflow)
                if (b_index < b_numChars - 1) {
                    c_receivedChars[b_index] = c_Data;
                    b_index++;
                }
            } else {
                c_receivedChars[b_index] = '\0'; // Fin de chaîne de caractères
                bol_ReceiveInProgress = false;
                b_index = 0;
                bol_newData = true;
            }
        } else if (c_Data == c_StartMarker) {
            bol_ReceiveInProgress = true;
        }
    }
}

//Découpage et traitement des commandes reçues
void receptionDonnees() {
    if (!bol_newData) return;

    char c_typeCmd = c_receivedChars[0];

    if (c_typeCmd == 'V') { // Commande Vitesse Moteurs : <V,v0,v1,v2,v3>
        int v0, v1, v2, v3;
        if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &v0, &v1, &v2, &v3) == 5) {
            n_motorSpeeds[0] = v0;
            n_motorSpeeds[1] = v1;
            n_motorSpeeds[2] = v2;
            n_motorSpeeds[3] = v3;

            Serial.printf("✅ OK [Moteurs] -> GAv:%d | DAv:%d | GAr:%d | DAr:%d\n", v0, v1, v2, v3);
        } else {
            Serial.println("❌ Erreur de format commande V (Attendu : <V,v0,v1,v2,v3>)");
        }
    } 
    else if (c_typeCmd == 'I') { // Commande Masquage Capteurs : <I,i0,i1,i2,i3>
        int i0, i1, i2, i3;
        if (sscanf(c_receivedChars, "%c,%d,%d,%d,%d", &c_typeCmd, &i0, &i1, &i2, &i3) == 5) {
            bol_ignoreSensor[0] = (i0 == 1);
            bol_ignoreSensor[1] = (i1 == 1);
            bol_ignoreSensor[2] = (i2 == 1);
            bol_ignoreSensor[3] = (i3 == 1);

            Serial.printf("🛡️ OK [Masques US] -> Cap1:%d | Cap2:%d | Cap3:%d | Cap4:%d\n", i0, i1, i2, i3);
        } else {
            Serial.println("❌ Erreur de format commande I (Attendu : <I,i0,i1,i2,i3>)");
        }
    } 
    else {
        Serial.printf("❌ Erreur : Type de commande inconnu ('%c')\n", c_typeCmd);
    }

    bol_newData = false;
}

void setup() {
    Serial.begin(115200);

    // Port Série UART2 (GPIO 16=RX2, GPIO 17=TX2 pour la Raspberry Pi)
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    Serial.println("=== ESP32 Démarré — Module de communication UART prêt ===");
}

void loop() {
    static unsigned long lastSendTime = 0;

    //Écouter et traiter les données provenant de la Raspberry Pi
    readSerialData();
    receptionDonnees();

    //Envoi périodique des mesures toutes les 500ms
    if (millis() - lastSendTime >= 500) {
        envoyerDonnees(50, 40, 50, 60);
        lastSendTime = millis();
    }
}