import serial
import time

# --- Configuration du port série UART (/dev/serial0 sur Pins 8 & 10) ---
try:
    ser = serial.Serial(
        port='/dev/serial0',
        baudrate=115200,
        timeout=1
    )
except Exception as e:
    print(f"❌ Erreur lors de l'ouverture du port série : {e}")
    exit(1)


def envoyer_commande_vitesse(v0, v1, v2, v3):
    """Envoie une consigne de vitesse aux 4 moteurs : <V,v0,v1,v2,v3>"""
    trame = f"<V,{v0},{v1},{v2},{v3}>\n"
    ser.write(trame.encode('utf-8'))
    print(f"[TX -> ESP32] Vitesse envoyée : {trame.strip()}")


def envoyer_masque_capteurs(i0, i1, i2, i3):
    """Envoie un masque d'activation/désactivation des capteurs US : <I,i0,i1,i2,i3>"""
    trame = f"<I,{i0},{i1},{i2},{i3}>\n"
    ser.write(trame.encode('utf-8'))
    print(f"[TX -> ESP32] Masque capteurs envoyé : {trame.strip()}")


def lire_reponse_esp32():
    """Lit et décode la trame de télémesure reçue depuis l'ESP32"""
    if ser.in_waiting > 0:
        ligne = ser.readline().decode('utf-8', errors='ignore').strip()

        # Vérification du format d'encadrement <...>
        if ligne.startswith('<') and ligne.endswith('>'):
            contenu = ligne[1:-1]  # Retire les balises < et >
            elements = contenu.split(',')

            type_trame = elements[0]

            # Traitement de la trame Ultrasons <US,d1,d2,d3,d4>
            if type_trame == "US" and len(elements) == 5:
                try:
                    us1, us2, us3, us4 = [float(x) for x in elements[1:]]
                    print(
                        f"[RX <- ESP32] 📏 US -> "
                        f"Av:{us1}cm | Dr:{us2}cm | Ga:{us3}cm | Ar:{us4}cm"
                    )
                except ValueError:
                    print(f"⚠️ Erreur de conversion des données US : {ligne}")
            else:
                print(f"[RX <- ESP32] Trame inconnue reçue : {ligne}")
        elif ligne:
            # Affiche les logs texte de debug renvoyés par l'ESP32
            print(f"[RX <- ESP32 Debug] {ligne}")


print("=== Communication UART Raspberry Pi 4 <-> ESP32 ===")
print("Appuyez sur Ctrl+C pour arrêter le test.\n")

time.sleep(1)  # Laisser le temps à la connexion de se stabiliser
compteur = 0

try:
    while True:
        # 1. Écouter la télémesure en continu
        lire_reponse_esp32()

        # 2. Envoyer une commande toutes les 3 secondes (30 * 100ms)
        if compteur % 30 == 0:
            cycle = (compteur // 30) % 3

            if cycle == 0:
                # Test commande vitesse normale
                envoyer_commande_vitesse(100, 100, 80, 80)
            elif cycle == 1:
                # Test commande virage / marche arrière
                envoyer_commande_vitesse(-50, 50, -50, 50)
            elif cycle == 2:
                # Test désactivation du capteur Droit (capteur index 1 à True)
                envoyer_masque_capteurs(0, 1, 0, 0)

        compteur += 1
        time.sleep(0.1)  # Boucle de 100ms

except KeyboardInterrupt:
    print("\n[FIN] Arrêt du script par l'utilisateur.")
finally:
    ser.close()