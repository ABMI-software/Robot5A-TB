#define STEP_PIN 2
#define DIR_PIN 5
#define EN_PIN 8

#define DISTANCE_PAR_TOUR 75.0
#define PAS_PAR_TOUR 3200
#define PAS_PAR_MM (PAS_PAR_TOUR / DISTANCE_PAR_TOUR)
#define LIMITE_MIN 0.0
#define LIMITE_MAX 800.0

float positionActuelle = 0.0;  // Position actuelle en mm
int vitesseDelay = 500;        // Temps entre deux pas (en microsecondes)
bool stopDemande = false;      // Flag pour arrêter le mode en cours

// Variables pour la gestion des modes
bool modeActif = false;
int modeCourant = 0;  // 1: manuel, 2: aléatoire, 3: aller-retour

void setup() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // Activer le moteur au démarrage

    Serial.begin(9600);
    afficherMessageBienvenue();
}

void loop() {
    if (Serial.available()) {
        char commande = Serial.read();
        stopDemande = false;

        switch (commande) {
            case 'M': afficherMenu(); break;
            case '1': activerMode(1); break;
            case '2': activerMode(2); break;
            case '3': activerMode(3); break;
            case 'V': ajusterVitesse(); break;
            case 'R': retournerEtatInitial(); break;
            case 'S': stopDemande = true; Serial.println("Chariot stoppé."); break;
            case 'Q': quitter(); break;
            default:
                nettoyerSerial();
                Serial.println("Commande inconnue. Tapez 'M' pour afficher les options.");
                break;
        }
    }

    if (modeActif) {
        switch (modeCourant) {
            case 1: mode1_pilotageManuel(); break;
            case 2: mode2_deplacementAleatoire(); break;
            case 3: mode3_allerRetour(); break;
        }
    }
}

// --- Afficher le message de bienvenue ---
void afficherMessageBienvenue() {
    Serial.println("Bienvenue dans le système de pilotage !");
    Serial.println("Tapez 'M' pour afficher les différents modes disponibles.");
}

// --- Afficher le menu ---
void afficherMenu() {
    Serial.println("\n--- Modes disponibles ---");
    Serial.println(" 1 : Pilotage manuel");
    Serial.println(" 2 : Déplacement aléatoire");
    Serial.println(" 3 : Aller-retour");
    Serial.println(" V : Ajuster la vitesse (mm/s)");
    Serial.println(" R : Retourner à l'état initial");
    Serial.println(" S : Stopper le chariot");
    Serial.println(" Q : Quitter et remettre à zéro");
}

// --- Activer un mode ---
void activerMode(int mode) {
    modeCourant = mode;
    modeActif = true;
    stopDemande = false;
    Serial.print("Mode ");
    Serial.print(mode);
    Serial.println(" activé.");
}

// --- Mode 1 : Pilotage manuel ---
void mode1_pilotageManuel() {
    Serial.println("Mode 1 : Entrez 'P <position_mm>' pour définir une position ou 'S' pour arrêter.");
    while (!stopDemande) {
        if (Serial.available()) {
            String input = Serial.readStringUntil('\n');
            input.trim();

            if (input.startsWith("P")) {
                float position = input.substring(1).toFloat();
                if (position >= LIMITE_MIN && position <= LIMITE_MAX) {
                    deplacerVers(position);
                } else {
                    Serial.println("Erreur : Position hors limites.");
                }
            } else if (input == "S") {
                stopDemande = true;
                modeActif = false;
                Serial.println("Mode manuel stoppé.");
            }
        }
    }
}

// --- Mode 2 : Déplacement aléatoire ---
void mode2_deplacementAleatoire() {
    static int nombreDeplacements = -1; // Nombre total de déplacements
    static int compteur = 0;           // Nombre de déplacements effectués

    if (nombreDeplacements == -1) { // Première exécution : demande du nombre de déplacements
        Serial.println("Entrez le nombre de déplacements aléatoires :");
        while (!Serial.available()); // Attendre l'entrée utilisateur
        nombreDeplacements = Serial.parseInt(); // Lire le nombre

        if (nombreDeplacements <= 0) { // Validation de l'entrée
            Serial.println("Erreur : Nombre invalide. Réessayez.");
            nombreDeplacements = -1;
            return;
        }

        compteur = 0; // Réinitialiser le compteur
        Serial.print("Nombre de déplacements : ");
        Serial.println(nombreDeplacements);
    }

    // Effectuer les déplacements
    if (!stopDemande && compteur < nombreDeplacements) {
        // Générer une position multiple de 50 mm entre 50 et 800 mm
        float positionCible = random(1, 17) * 50; // Génère un multiple de 50 (1*50 à 16*50)

        Serial.print("Déplacement vers : ");
        Serial.println(positionCible);
        deplacerVers(positionCible); // Appeler la fonction de déplacement
        compteur++;
    }

    // Vérifier si tous les déplacements ont été effectués ou si l'utilisateur a demandé un arrêt
    if (compteur >= nombreDeplacements || stopDemande) {
        Serial.println("Mode aléatoire terminé.");
        modeActif = false;
        nombreDeplacements = -1; // Réinitialiser pour une nouvelle utilisation

        // Demander une nouvelle entrée ou quitter
        Serial.println("Tapez 'R' pour relancer ou 'M' pour retourner au menu principal.");
        while (!Serial.available());
        char choix = Serial.read();
        if (choix == 'R') {
            activerMode(2);
        } else if (choix == 'M') {
            modeActif = false;
        }
    }
}

// --- Mode 3 : Aller-retour ---
void mode3_allerRetour() {
    static int compteurAllerRetour = 0;
    static bool directionVersMax = true;  // Indique si on va vers LIMITE_MAX ou LIMITE_MIN

    while (!stopDemande) {
        if (directionVersMax && positionActuelle < LIMITE_MAX) {
            deplacerVers(LIMITE_MAX);  // Déplacer vers la limite MAX
            directionVersMax = false;  // Inverser la direction
        } else if (!directionVersMax && positionActuelle > LIMITE_MIN) {
            deplacerVers(LIMITE_MIN);  // Déplacer vers la limite MIN
            directionVersMax = true;   // Inverser la direction
            compteurAllerRetour++;     // Compter un cycle complet
        }

        if (Serial.available()) {
            char commande = Serial.read();
            if (commande == 'S') {
                stopDemande = true;
                Serial.print("Allers-retours effectués : ");
                Serial.println(compteurAllerRetour);
                modeActif = false;
                break;
            }
        }
    }

    if (stopDemande) {
        Serial.println("Mode 3 arrêté.");
    }
}

// --- Déplacer vers une position ---
void deplacerVers(float positionCible) {
    if (positionCible < LIMITE_MIN || positionCible > LIMITE_MAX) return;

    int steps = abs((positionCible - positionActuelle) * PAS_PAR_MM);
    digitalWrite(DIR_PIN, positionCible > positionActuelle ? HIGH : LOW);

    for (int i = 0; i < steps && !stopDemande; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(vitesseDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(vitesseDelay);
    }
    positionActuelle = positionCible;
}

// --- Ajuster la vitesse ---
void ajusterVitesse() {
    Serial.println("Entrez une nouvelle vitesse (mm/s) :");
    while (!Serial.available());
    float vitesse = Serial.parseFloat();
    if (vitesse > 0 && vitesse <= 200) {
        vitesseDelay = (0.0234 * 1e6) / vitesse;
        Serial.print("Vitesse réglée à : ");
        Serial.println(vitesse);
    } else {
        Serial.println("Erreur : Vitesse invalide.");
    }
}

// --- Retourner à l'état initial ---
void retournerEtatInitial() {
    Serial.println("Retour à l'état initial (0 mm).");
    deplacerVers(0.0);
}

// --- Quitter et réinitialiser ---
void quitter() {
    Serial.println("Fin des tests. Remise à zéro...");
    retournerEtatInitial();
    modeActif = false;
    stopDemande = false;
    digitalWrite(EN_PIN, HIGH);
    Serial.println("Système remis à zéro.");
}

// --- Nettoyer le Serial ---
void nettoyerSerial() {
    while (Serial.available()) Serial.read();
}
