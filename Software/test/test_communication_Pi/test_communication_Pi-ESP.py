import serial
import time

# --- Configuration de la liaison UART0 de la RPi4 (/dev/serial0 sur Pins 8 & 10) ---
try:
    ser = serial.Serial(
        port='/dev/serial0',  # Modifier en '/dev/ttyAMA0' si /dev/serial0 ne répond pas
        baudrate=115200,
        timeout=1
    )
    print(" Port série /dev/serial0 ouvert avec succès à 115200 bauds.")
except Exception as e:
    print(f"❌ Erreur lors de l'ouverture du port série : {e}")
    exit(1)


def envoyer_commande_vitesse(v0, v1, v2, v3):
    """Envoie une consigne de vitesse en ticks/20ms : <V,v0,v1,v2,v3>"""
    trame = f"<V,{v0},{v1},{v2},{v3}>\n"
    ser.write(trame.encode('utf-8'))
    ser.flush()  # Force l'envoi physique immédiat sur la broche TX
    print(f"[TX -> ESP32] Vitesse (ticks/20ms) : {trame.strip()}")


def envoyer_masque_capteurs(i0, i1, i2, i3):
    """Envoie le masque des capteurs US à ignorer (1 = ignorer, 0 = actif) : <I,i0,i1,i2,i3>"""
    trame = f"<I,{i0},{i1},{i2},{i3}>\n"
    ser.write(trame.encode('utf-8'))
    ser.flush()  # Force l'envoi physique immédiat sur la broche TX
    print(f"[TX -> ESP32] Masque US : {trame.strip()}")


def lire_reponse_esp32():
    """Lit et décode la télémétrie ou les alertes transmises par l'ESP32"""
    if ser.in_waiting > 0:
        ligne = ser.readline().decode('utf-8', errors='ignore').strip()

        # Format d'encadrement <...>
        if ligne.startswith('<') and ligne.endswith('>'):
            contenu = ligne[1:-1]
            elements = contenu.split(',')
            header = elements[0]

            # 1. Télémétrie Ultrasons <US,d1,d2,d3,d4>
            if header == "US" and len(elements) == 5:
                try:
                    us1, us2, us3, us4 = [float(x) for x in elements[1:]]
                    print(
                        f"[RX <- ESP32]  US -> "
                        f"GAv:{us1:.1f}cm | DAv:{us2:.1f}cm | GAr:{us3:.1f}cm | DAr:{us4:.1f}cm"
                    )
                except ValueError:
                    print(f" Erreur de parsing US : {ligne}")

            # 2. Alerte Arrêt d'Urgence <,URGENCE, Capteur X>
            elif "URGENCE" in header or "URGENCE" in line_content(elements):
                print(f"[RX <- ESP32]  ALERTE OBSTACLE PROCHE : {ligne}")

            else:
                print(f"[RX <- ESP32] Trame inconnue : {ligne}")
        elif ligne:
            # Logs de débogage texte renvoyés par l'ESP32
            print(f"[RX <- ESP32 Debug] {ligne}")


def line_content(elements):
    return ",".join(elements)


print("\n=== Test de Communication UART Raspberry Pi 4 <-> ESP32 ===")
print("Appuyez sur Ctrl+C pour interrompre le test.\n")

time.sleep(1)
compteur = 0

try:
    while True:
        # 1. Écouter la télémesure en continu
        lire_reponse_esp32()

        # 2. Changer de consigne toutes les 3 secondes (30 * 100ms)
        if compteur % 30 == 0:
            cycle = (compteur // 30) % 4

            if cycle == 0:
                print("\n--- Phase 1 : Marche avant lente ---")
                envoyer_commande_vitesse(15, 15, 15, 15)

            elif cycle == 1:
                print("\n--- Phase 2 : Rotation sur place ---")
                envoyer_commande_vitesse(-15, 15, -15, 15)

            elif cycle == 2:
                print("\n--- Phase 3 : Masquage Capteur DAv ---")
                envoyer_masque_capteurs(0, 1, 0, 0)

            elif cycle == 3:
                print("\n--- Phase 4 : Consigne d'arrêt ---")
                envoyer_commande_vitesse(0, 0, 0, 0)

        compteur += 1
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[FIN] Arrêt manuel. Envoi de la consigne d'arrêt de sécurité...")
    envoyer_commande_vitesse(0, 0, 0, 0)
finally:
    ser.close()