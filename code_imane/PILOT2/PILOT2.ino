#define STEP_PIN 2
#define DIR_PIN 5
#define EN_PIN 8


#define DISTANCE_PAR_TOUR 75.0
#define PAS_PAR_TOUR 3200
#define PAS_PAR_MM (PAS_PAR_TOUR / DISTANCE_PAR_TOUR)
#define LIMITE_MIN 0.0
#define LIMITE_MAX 800.0

float positionActuelle = 0.0;  // Current position in mm
int vitesseDelay = 500;        // Time between steps (in microseconds)

void setup() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // Enable the motor at startup

    Serial.begin(9600);
    Serial.println("Welcome to the control system!");
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("P")) {
            float position = command.substring(1).toFloat();
            moveToPosition(position);
        } else if (command.startsWith("V")) {
            float speed = command.substring(1).toFloat();
            changeSpeed(speed);
        } else {
            Serial.println("Unknown command. Use 'P<position>' to move or 'V<speed>' to change speed.");
        }
    }
}

// Move to a specified position
void moveToPosition(float targetPosition) {
    if (targetPosition < LIMITE_MIN || targetPosition > LIMITE_MAX) {
        Serial.println("Error: Position out of limits.");
        return;
    }

    Serial.println("Starting movement...");
    digitalWrite(DIR_PIN, targetPosition > positionActuelle ? HIGH : LOW);
    int steps = abs((targetPosition - positionActuelle) * (PAS_PAR_MM)); 

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(vitesseDelay);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(vitesseDelay);
    }

    positionActuelle = targetPosition;
    Serial.println("Reached target position: " + String(positionActuelle));
}

// Change the speed
void changeSpeed(float newSpeed) {
    if (newSpeed > 0 && newSpeed <= 200) {
        vitesseDelay = (0.0234 * 1e6) / newSpeed;
        Serial.print("Speed set to: ");
        Serial.println(newSpeed);
    } else {
        Serial.println("Error: Invalid speed.");
    }
}