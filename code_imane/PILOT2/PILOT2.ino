#define STEP_PIN 2
#define DIR_PIN 5
#define EN_PIN 8


#define DISTANCE_PER_TURN 75.0
#define STEPS_PER_TURN 3200
#define STEPS_PER_MM (STEPS_PER_TURN / DISTANCE_PER_TURN)
#define LIMITE_MIN 0.0
#define LIMITE_MAX 800.0

float currentPosition = 0.0;  // Current position in mm
int delaySpeed = 500;        // Time between steps (in microseconds)

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
    digitalWrite(DIR_PIN, targetPosition > currentPosition ? HIGH : LOW);
    int steps = abs((targetPosition - currentPosition) * (STEPS_PER_MM)); 

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delaySpeed);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delaySpeed);
    }

    currentPosition = targetPosition;
    Serial.println("Reached target position: " + String(currentPosition));
}

// Change the speed
void changeSpeed(float newSpeed) {
    if (newSpeed > 0 && newSpeed <= 200) {
        delaySpeed = (0.0234 * 1e6) / newSpeed;
        Serial.print("Speed set to: ");
        Serial.println(newSpeed);
    } else {
        Serial.println("Error: Invalid speed.");
    }
}