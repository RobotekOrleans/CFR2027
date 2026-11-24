import serial
import time

# --- Configuration de la liaison UART0 de la RPi4 (/dev/serial0 sur Pins 8 & 10) ---
try:
    ser = serial.Serial(
        port='/dev/serial0',  # Modifier en '/dev/ttyAMA0' si /dev/serial0 ne répond pas
        baudrate=115200,
        timeout=0.05
    )
    print("✅ Port série /dev/serial0 ouvert avec succès à 115200 bauds.")
except Exception as e:
    print(f"❌ Erreur lors de l'ouverture du port série : {e}")
    exit(1)


def envoyer_commande_vitesse(v0, v1, v2, v3, afficher=False):
    """Envoie une consigne de vitesse en ticks/20ms : <V,GAv,DAv,GAr,DAr>"""
    trame = f"<V,{v0},{v1},{v2},{v3}>\n"
    ser.write(trame.encode('utf-8'))
    ser.flush()
    if afficher:
        print(f"[TX -> ESP32] Vitesse (ticks/20ms) : {trame.strip()}")


def envoyer_masque_capteurs(i0, i1, i2, i3):
    """Envoie le masque des capteurs US (1 = ignorer, 0 = actif) : <I,Avant,Droit,Gauche,Arriere>"""
    trame = f"<I,{i0},{i1},{i2},{i3}>\n"
    ser.write(trame.encode('utf-8'))
    ser.flush()
    print(f"[TX -> ESP32] Masque US : {trame.strip()}")


def lire_reponse_esp32():
    """Lit et décode toute la télémétrie ou les alertes en attente depuis l'ESP32"""
    while ser.in_waiting > 0:
        ligne = ser.readline().decode('utf-8', errors='ignore').strip()

        if ligne.startswith('<') and ligne.endswith('>'):
            contenu = ligne[1:-1]
            elements = contenu.split(',')
            header = elements[0]

            # 1. Télémétrie Ultrasons <US,Avant,Droit,Gauche,Arriere>
            if header == "US" and len(elements) == 5:
                try:
                    us_av, us_dr, us_ga, us_ar = [float(x) for x in elements[1:]]
                    print(
                        f"[RX <- ESP32] US -> "
                        f"Avant:{us_av:.1f}cm | Droit:{us_dr:.1f}cm | "
                        f"Gauche:{us_ga:.1f}cm | Arrière:{us_ar:.1f}cm"
                    )
                except ValueError:
                    print(f"⚠️ Erreur de parsing US : {ligne}")

            # 2. Alerte Arrêt d'Urgence <URGENCE,X>
            elif "URGENCE" in contenu:
                print(f"[RX <- ESP32] 🚨 ALERTE OBSTACLE PROCHE : {ligne}")

            else:
                print(f"[RX <- ESP32] Trame inconnue : {ligne}")
        elif ligne:
            print(f"[RX <- ESP32 Debug] {ligne}")


print("\n=== Test de Communication UART & Échelons PID (RPi 4 <-> ESP32) ===")
print("Appuyez sur Ctrl+C pour interrompre le test.\n")

time.sleep(1)
compteur = 0
consigne_actuelle = [0, 0, 0, 0]

try:
    while True:
        # 1. Lire toutes les réponses en attente de l'ESP32
        lire_reponse_esp32()

        # 2. Changer de phase toutes les 3 secondes (30 * 100ms)
        if compteur % 30 == 0:
            cycle = (compteur // 30) % 4

            if cycle == 0:
                print("\n--- Phase 1 : Marche avant lente (15 ticks/20ms) ---")
                consigne_actuelle = [15, 15, 15, 15]
                envoyer_commande_vitesse(*consigne_actuelle, afficher=True)

            elif cycle == 1:
                print("\n--- Phase 2 : Rotation sur place (-15 / +15) ---")
                consigne_actuelle = [-15, 15, -15, 15]
                envoyer_commande_vitesse(*consigne_actuelle, afficher=True)

            elif cycle == 2:
                print("\n--- Phase 3 : Masquage Capteur Droit (index 1) ---")
                envoyer_masque_capteurs(0, 1, 0, 0)

            elif cycle == 3:
                print("\n--- Phase 4 : Consigne d'arrêt ---")
                consigne_actuelle = [0, 0, 0, 0]
                envoyer_commande_vitesse(*consigne_actuelle, afficher=True)
        else:
            # Maintien du Watchdog (< 500 ms) : on réactualise la consigne toutes les 100 ms
            envoyer_commande_vitesse(*consigne_actuelle, afficher=False)

        compteur += 1
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[FIN] Arrêt manuel. Envoi de la consigne d'arrêt de sécurité...")
    envoyer_commande_vitesse(0, 0, 0, 0, afficher=True)
finally:
    ser.close()