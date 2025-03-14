#define STEP_PIN 2
#define DIR_PIN 5
#define EN_PIN 8

#define DISTANCE_PER_TURN 75.0
#define STEPS_PER_TURN 3200
#define STEPS_PER_MM (STEPS_PER_TURN / DISTANCE_PER_TURN)
#define LIMIT_MIN 0.0
#define LIMIT_MAX 800.0

float currentPosition = 0.0;  // Current position in mm
int delaySpeed = 117;         // Time between steps (in microseconds)
unsigned long lastPrintTime = 0;  // To track the last time we printed
const unsigned long printInterval = 100;  // Print interval in milliseconds (Match to FPS)

void setup() {
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // Enable the motor at startup

    Serial.begin(9600);
    Serial.println("Welcome to the control system!");
}

void loop() {
    // Check for serial commands
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

    // Print current position periodically
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= printInterval) {
        Serial.print("Current Position: ");
        Serial.print(currentPosition);
        lastPrintTime = currentTime;  // Update the last print time
    }
}

// Move to a specified position
void moveToPosition(float targetPosition) {
    if (targetPosition < LIMIT_MIN || targetPosition > LIMIT_MAX) {
        Serial.println("Error: Position out of limits.");
        return;
    }

    Serial.println("Starting movement...");
    digitalWrite(DIR_PIN, targetPosition > currentPosition ? HIGH : LOW);
    int steps = abs((targetPosition - currentPosition) * STEPS_PER_MM); 

    for (int i = 0; i < steps; i++) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delaySpeed);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delaySpeed);

        // Update current position incrementally during movement
        currentPosition += (targetPosition > currentPosition ? 1.0 : -1.0) / STEPS_PER_MM;
    }

    // Ensure final position is exact due to possible floating-point errors
    currentPosition = targetPosition;
    Serial.println("Reached target position: " + String(currentPosition));
}

// Change the speed (mm/s)
void changeSpeed(float newSpeed) {
    if (newSpeed > 0 && newSpeed <= 400) {
        delaySpeed = (0.0234 * 1e6) / newSpeed;
        Serial.print("Speed set to: ");
        Serial.println(newSpeed);
    } else {
        Serial.println("Error: Invalid speed.");
    }
}