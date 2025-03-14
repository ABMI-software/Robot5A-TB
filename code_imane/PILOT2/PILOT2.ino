#define STEP_PIN 2          // Step pin for the stepper motor driver
#define DIR_PIN 5           // Direction pin for the stepper motor driver
#define EN_PIN 8            // Enable pin for the stepper motor driver

#define DISTANCE_PER_TURN 75.0   // Distance traveled per full turn in mm
#define STEPS_PER_TURN 3200      // Steps required for one full turn
#define STEPS_PER_MM (STEPS_PER_TURN / DISTANCE_PER_TURN)  // Steps per millimeter
#define LIMIT_MIN 0.0            // Minimum position limit in mm
#define LIMIT_MAX 800.0          // Maximum position limit in mm

float currentPosition = 0.0;     // Current position in mm
int delaySpeed = 117;            // Time between steps (in microseconds)
unsigned long lastPrintTime = 0; // To track the last time we printed
const unsigned long printInterval = 100;  // Print interval in milliseconds (10 FPS)

void setup() {
    pinMode(STEP_PIN, OUTPUT);   // Set step pin as output
    pinMode(DIR_PIN, OUTPUT);    // Set direction pin as output
    pinMode(EN_PIN, OUTPUT);     // Set enable pin as output
    digitalWrite(EN_PIN, LOW);   // Enable the motor at startup (LOW = enabled)

    Serial.begin(9600);          // Start serial communication at 9600 baud
    while (!Serial) {            // Wait for serial port to connect (needed for some boards)
        ; // Do nothing
    }
    Serial.println("Welcome to the control system!");  // Initial message
    Serial.flush();              // Ensure initial message is sent
    delay(1000);                 // Wait 1 second to allow ROS to catch up
}

void loop() {
    // Check for serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n'); // Read command until newline
        command.trim();                                // Remove leading/trailing whitespace

        if (command.startsWith("P")) {                 // Position command
            float position = command.substring(1).toFloat(); // Extract position value
            moveToPosition(position);                  // Move to specified position
        } else if (command.startsWith("V")) {          // Speed command
            float speed = command.substring(1).toFloat(); // Extract speed value
            changeSpeed(speed);                        // Change motor speed
        } else {
            Serial.println("Unknown command. Use 'P<position>' to move or 'V<speed>' to change speed.");
            Serial.flush();                            // Ensure error message is sent
        }
    }

    // Print current position periodically
    unsigned long currentTime = millis();          // Get current time in milliseconds
    if (currentTime - lastPrintTime >= printInterval) { // Check if it's time to print
        Serial.print("Current Position: ");        // Print position label
        Serial.print(currentPosition);             // Print position value
        Serial.print("\n");                        // Explicitly add newline
        Serial.flush();                            // Force send to prevent buffer overflow
        lastPrintTime = currentTime;               // Update last print time
    }
}

// Move to a specified position
void moveToPosition(float targetPosition) {
    if (targetPosition < LIMIT_MIN || targetPosition > LIMIT_MAX) { // Check position limits
        Serial.println("Error: Position out of limits.");
        Serial.flush();                            // Ensure error message is sent
        return;
    }

    Serial.println("Starting movement...");        // Indicate movement start
    Serial.flush();                                // Ensure message is sent
    digitalWrite(DIR_PIN, targetPosition > currentPosition ? HIGH : LOW); // Set direction
    int steps = abs((targetPosition - currentPosition) * STEPS_PER_MM);   // Calculate steps needed

    for (int i = 0; i < steps; i++) {              // Step the motor
        digitalWrite(STEP_PIN, HIGH);              // Pulse high
        delayMicroseconds(delaySpeed);             // Wait
        digitalWrite(STEP_PIN, LOW);               // Pulse low
        delayMicroseconds(delaySpeed);             // Wait
        currentPosition += (targetPosition > currentPosition ? 1.0 : -1.0) / STEPS_PER_MM; // Update position incrementally
    }

    currentPosition = targetPosition;              // Set final position to target (corrects floating-point drift)
    Serial.println("Reached target position: " + String(currentPosition)); // Confirm arrival
    Serial.flush();                                // Ensure message is sent
}

// Change the speed (mm/s)
void changeSpeed(float newSpeed) {
    if (newSpeed > 0 && newSpeed <= 400) {         // Validate speed range
        delaySpeed = (0.0234 * 1e6) / newSpeed;    // Calculate delay in microseconds (derived from speed)
        Serial.print("Speed set to: ");            // Confirm speed change
        Serial.println(newSpeed);
        Serial.flush();                            // Ensure message is sent
    } else {
        Serial.println("Error: Invalid speed.");   // Report invalid speed
        Serial.flush();                            // Ensure message is sent
    }
}