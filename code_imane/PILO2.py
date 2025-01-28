import serial
import time
from colorama import Fore, Style

# Configuration du port série
PORT = "COM4"  # Port utilisé par la Nucleo
BAUDRATE = 9600

# Initialisation de la communication série
try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(Fore.GREEN + f"Connexion série établie avec {PORT} à {BAUDRATE} bps." + Style.RESET_ALL)
    time.sleep(2)  # Temps pour s'assurer que la connexion est prête
except Exception as e:
    print(Fore.RED + f"Erreur lors de la connexion série : {e}" + Style.RESET_ALL)
    exit(1)

# Variables globales pour le suivi
positions_mode1 = []
positions_mode2 = []
aller_retours_mode3 = 0
vitesse_actuelle = 0  # Stocke la vitesse actuelle
debut_test = time.time()  # Temps de début pour le feedback

def envoyer_commande(commande):
    """Envoyer une commande à la Nucleo et lire la réponse."""
    ser.write((commande + "\n").encode())
    time.sleep(0.1)
    while ser.in_waiting > 0:
        response = ser.readline().decode().strip()
        print(response)
        return response

def mode_manuel():
    """Mode manuel pour définir des positions spécifiques."""
    print("Mode manuel activé. Entrez une position en mm ou 'S' pour arrêter.")
    envoyer_commande('1')  # Activer le mode manuel
    while True:
        position = input("Entrez une position en mm (ou 'S' pour quitter) : ")
        if position.upper() == 'S':
            envoyer_commande('S')  # Stopper le mode manuel
            break
        try:
            pos = float(position)
            envoyer_commande(f'P{pos}')  # Envoyer la position
            positions_mode1.append(pos)  # Sauvegarder la position
        except ValueError:
            print(Fore.RED + "Veuillez entrer un nombre valide." + Style.RESET_ALL)
    print(Fore.YELLOW + f"Mode manuel terminé. Positions atteintes : {positions_mode1}" + Style.RESET_ALL)

def mode_aleatoire():
    """Mode aléatoire pour des déplacements aléatoires."""
    global positions_mode2
    positions_mode2 = []  # Réinitialiser les positions pour chaque exécution
    envoyer_commande('2')  # Activer le mode aléatoire
    time.sleep(0.1)
    deplacements = input("Entrez le nombre de déplacements aléatoires (entre 1 et 16) : ")
    if deplacements.isdigit() and 1 <= int(deplacements) <= 16:
        envoyer_commande(deplacements)  # Envoyer le nombre de déplacements
        print(f"Mode aléatoire activé avec {deplacements} déplacements.")
        compteur = 0
        while compteur < int(deplacements):
            response = ser.readline().decode().strip()
            if response.startswith("Déplacement vers :"):
                pos = response.split(":")[1].strip()
                positions_mode2.append(float(pos))
                print(Fore.CYAN + f"Position atteinte : {pos}" + Style.RESET_ALL)
                compteur += 1
        print(Fore.YELLOW + f"Mode aléatoire terminé. Positions atteintes : {positions_mode2}" + Style.RESET_ALL)
    else:
        print(Fore.RED + "Erreur : Nombre invalide. Réessayez avec un nombre entre 1 et 16." + Style.RESET_ALL)

def mode_aller_retour():
    """Mode aller-retour entre deux positions limites."""
    global aller_retours_mode3
    envoyer_commande('3')  # Activer le mode aller-retour
    print("Mode aller-retour activé. Appuyez sur 'S' pour stopper.")
    while True:
        commande = input("Appuyez sur 'S' pour arrêter : ").upper()
        if commande == 'S':
            envoyer_commande('S')  # Stopper le mode aller-retour
            break
        else:
            response = ser.readline().decode().strip()
            if response and "Aller-retour terminé" in response:
                aller_retours_mode3 += 1
                print(Fore.CYAN + f"Aller-retours terminés : {aller_retours_mode3}" + Style.RESET_ALL)

def afficher_menu():
    """Afficher le menu principal."""
    print(Fore.CYAN + "\n--- Menu principal ---")
    print("1 : Activer le mode manuel")
    print("2 : Activer le mode aléatoire")
    print("3 : Activer le mode aller-retour")
    print("V : Ajuster la vitesse")
    print("R : Retourner à l'état initial")
    print("S : Stopper le chariot")
    print("Q : Quitter" + Style.RESET_ALL)

def ajuster_vitesse():
    """Ajuster la vitesse de déplacement."""
    global vitesse_actuelle
    vitesse = input("Nouvelle vitesse (mm/s) : ")
    try:
        vitesse_actuelle = float(vitesse)
        envoyer_commande(f'V{vitesse_actuelle}')
        print(Fore.GREEN + f"Vitesse réglée à : {vitesse_actuelle} mm/s." + Style.RESET_ALL)
    except ValueError:
        print(Fore.RED + "Erreur : Veuillez entrer une valeur valide." + Style.RESET_ALL)

def retourner_etat_initial():
    """Retourner le chariot à la position initiale."""
    print(Fore.YELLOW + "Retour à la position initiale..." + Style.RESET_ALL)
    envoyer_commande('R')
    print(Fore.GREEN + "Retour à l'état initial effectué." + Style.RESET_ALL)

def afficher_feedback():
    """Afficher un récapitulatif des actions effectuées."""
    print(Fore.CYAN + "\n--- Récapitulatif des tests ---")
    print(f"Positions atteintes en mode manuel : {positions_mode1}")
    print(f"Positions atteintes en mode aléatoire : {positions_mode2}")
    print(f"Nombre d'aller-retours effectués en mode 3 : {aller_retours_mode3}")
    print(f"Vitesse actuelle : {vitesse_actuelle} mm/s")
    print(f"Durée totale du test : {round(time.time() - debut_test, 2)} secondes")
    print("--- Fin du récapitulatif ---" + Style.RESET_ALL)

try:
    while True:
        afficher_menu()
        choix = input("Entrez une commande : ").upper()
        if choix == '1':
            mode_manuel()
        elif choix == '2':
            mode_aleatoire()
        elif choix == '3':
            mode_aller_retour()
        elif choix == 'V':
            ajuster_vitesse()
        elif choix == 'R':
            retourner_etat_initial()
        elif choix == 'S':
            envoyer_commande('S')  # Stopper le chariot
        elif choix == 'Q':
            retourner_etat_initial()
            afficher_feedback()
            print(Fore.GREEN + "Fermeture de la connexion." + Style.RESET_ALL)
            break
        else:
            print(Fore.RED + "Commande inconnue. Veuillez réessayer." + Style.RESET_ALL)
finally:
    ser.close()
    print(Fore.GREEN + "Connexion série fermée." + Style.RESET_ALL)
