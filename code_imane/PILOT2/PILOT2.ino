// Define pin assignments for the stepper motor driver
#define STEP_PIN 2          // Step pin: sends pulses to move the motor one step
#define DIR_PIN 5           // Direction pin: HIGH for forward, LOW for backward
#define EN_PIN 8            // Enable pin: LOW enables the motor, HIGH disables it

// Define constants for motor movement calculations
#define DISTANCE_PER_TURN 75.0   // Distance (mm) the chariot moves per full motor turn
#define STEPS_PER_TURN 3200      // Number of steps required for one full motor turn
#define STEPS_PER_MM (STEPS_PER_TURN / DISTANCE_PER_TURN)  // Steps needed to move 1 mm

// Define position limits in millimeters
#define LIMIT_MIN 0.0            // Minimum allowed position (start of track)
#define LIMIT_MAX 800.0          // Maximum allowed position (end of track)

// Global variables
float currentPosition = 0.0;     // Tracks the chariot’s current position in mm
int delaySpeed = 117;            // Delay between steps (microseconds), controls speed
unsigned long lastPrintTime = 0; // Timestamp of the last position print (ms)
const unsigned long printInterval = 100;  // Interval (ms) for periodic position updates (10 FPS)

void setup() {
    // Configure pins as outputs for the stepper motor driver
    pinMode(STEP_PIN, OUTPUT);   // Step pin sends pulses to move the motor
    pinMode(DIR_PIN, OUTPUT);    // Direction pin sets movement direction
    pinMode(EN_PIN, OUTPUT);     // Enable pin controls motor power state
    
    // Enable the motor by setting EN_PIN low (LOW = enabled, HIGH = disabled)
    digitalWrite(EN_PIN, LOW);

    // Start serial communication at 9600 baud for talking to ROS
    Serial.begin(9600);
    
    // Wait until the serial connection is established (important for some Arduino boards)
    while (!Serial) {
        ; // Do nothing, just wait
    }
    
    // Send an initial message to confirm the system is ready
    Serial.println("Welcome to the control system!");
    Serial.flush();              // Ensure the message is fully sent before proceeding
    delay(1000);                 // Wait 1 second to let ROS initialize and avoid overlap
}

void loop() {
    // Check if there’s a command from ROS via serial
    if (Serial.available()) {
        // Read the command until a newline is received
        String command = Serial.readStringUntil('\n');
        command.trim();          // Remove any leading/trailing whitespace
        
        // Handle position commands (e.g., "P200" to move to 200 mm)
        if (command.startsWith("P")) {
            float position = command.substring(1).toFloat(); // Extract the number after "P"
            moveToPosition(position); // Move the chariot to the specified position
        }
        // Handle speed commands (e.g., "V240" to set speed to 240 mm/s)
        else if (command.startsWith("V")) {
            float speed = command.substring(1).toFloat(); // Extract the number after "V"
            changeSpeed(speed);      // Adjust the motor speed
        }
        // If the command is unrecognized, send an error message
        else {
            Serial.println("Unknown command. Use 'P<position>' to move or 'V<speed>' to change speed.");
            Serial.flush();          // Ensure the error message is sent
        }
    }

    // Periodically print the current position when not moving
    unsigned long currentTime = millis(); // Get the current time in milliseconds
    if (currentTime - lastPrintTime >= printInterval) { // Check if it’s time to print
        Serial.print("Current Position: "); // Label for the position message
        Serial.println(currentPosition);    // Print the current position value
        Serial.flush();                     // Force the message to send immediately
        lastPrintTime = currentTime;        // Update the last print time
    }
}

// Function to move the chariot to a target position
void moveToPosition(float targetPosition) {
    // Check if the target position is within allowed limits
    if (targetPosition < LIMIT_MIN || targetPosition > LIMIT_MAX) {
        Serial.println("Error: Position out of limits."); // Send error if out of bounds
        Serial.flush();                                   // Ensure the error is sent
        return;                                           // Exit the function
    }

    // Notify ROS that movement is starting
    Serial.println("Starting movement...");
    Serial.flush();                                   // Ensure the message is sent
    
    // Set the direction: HIGH for forward (increasing position), LOW for backward
    digitalWrite(DIR_PIN, targetPosition > currentPosition ? HIGH : LOW);
    
    // Calculate the number of steps needed to reach the target
    int steps = abs((targetPosition - currentPosition) * STEPS_PER_MM);
    
    // Variables to track position updates during movement
    unsigned long lastUpdateTime = millis();          // Time of the last position update
    const unsigned long updateInterval = 100;         // Update position every 100 ms

    // Step the motor the required number of times
    for (int i = 0; i < steps; i++) {
        // Send a pulse to move one step
        digitalWrite(STEP_PIN, HIGH);                 // Start pulse
        delayMicroseconds(delaySpeed);                // Wait (controls speed)
        digitalWrite(STEP_PIN, LOW);                  // End pulse
        delayMicroseconds(delaySpeed);                // Wait again

        // Update the current position incrementally after each step
        currentPosition += (targetPosition > currentPosition ? 1.0 : -1.0) / STEPS_PER_MM;

        // Check if it’s time to send a position update during movement
        unsigned long currentTime = millis();
        if (currentTime - lastUpdateTime >= updateInterval) {
            Serial.print("Current Position: ");       // Label for the position
            Serial.println(currentPosition);          // Print the current position
            Serial.flush();                           // Ensure the message is sent
            lastUpdateTime = currentTime;             // Update the last print time
        }
    }

    // Set the final position exactly to the target (corrects any floating-point drift)
    currentPosition = targetPosition;
    Serial.print("Reached target position: ");        // Confirm the move is complete
    Serial.println(currentPosition);                  // Print the final position
    Serial.flush();                                   // Ensure the message is sent
}

// Function to change the motor speed (in mm/s)
void changeSpeed(float newSpeed) {
    // Validate the speed is positive and within a reasonable range (up to 400 mm/s)
    if (newSpeed > 0 && newSpeed <= 400) {
        // Convert speed (mm/s) to delay between steps (microseconds)
        delaySpeed = (0.0234 * 1e6) / newSpeed;      // Derived from motor timing
        Serial.print("Speed set to: ");               // Confirm the new speed
        Serial.println(newSpeed);                     // Print the speed value
        Serial.flush();                               // Ensure the message is sent
    } else {
        Serial.println("Error: Invalid speed.");      // Send error if speed is invalid
        Serial.flush();                               // Ensure the error is sent
    }
}