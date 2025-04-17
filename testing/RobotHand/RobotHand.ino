 #include <Servo.h>

const int servoPins[] = {3, 5, 6, 9, 10};  // Pins for servos
const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);
Servo servos[numServos];  // Create servo objects
int lastPositions[numServos] = {90, 90, 90, 90, 90};  // Store last known positions

void setup() {
    Serial.begin(9600);
    
    for (int i = 0; i < numServos; i++) {
        servos[i].attach(servoPins[i]);  // Attach servos to their pins
        servos[i].write(lastPositions[i]);  // Initialize to default 90 degrees
    }

    Serial.println("Enter servo ID (1-5) and angle (0-180) as: ID, value");
    Serial.println("Or type 'cycle' to run the predefined motion sequence.");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read input until newline
        input.trim();  // Remove any trailing whitespace

        if (input.equalsIgnoreCase("cycle")) {
            runCycleSequence();
        } else {
            int id, value;
            if (sscanf(input.c_str(), "%d, %d", &id, &value) == 2) {  // Parse input
                if (id >= 1 && id <= numServos && value >= 0 && value <= 180) {
                    if (value != lastPositions[id - 1]) {  // Only update if value changed
                        lastPositions[id - 1] = value;
                        servos[id - 1].write(value);  // Set servo to new position
                        Serial.print("Servo ");
                        Serial.print(id);
                        Serial.print(" set to angle ");
                        Serial.println(value);
                    }
                } else {
                    Serial.println("Invalid input. ID must be 1-5, value must be 0-180.");
                }
            } else {
                Serial.println("Invalid format. Use: ID, value (e.g., 1, 90) or 'cycle'.");
            }
        }
    }
}

// Function to execute the full motion cycle sequence
void runCycleSequence() {
    Serial.println("Starting cycle sequence...");

    // **Step 1: Initial Setup**
    servos[0].write(180); // ID 1 → 180
    for (int i = 1; i < numServos; i++) {
        servos[i].write(0); // ID 2-5 → 0
    }
    delay(1000);

    // **Step 2: First sequence (Each servo moves one at a time)**
    for (int i = 0; i < numServos; i++) {
        if (i == 0) {
            moveServoSmooth(i, 180, 0);
            moveServoSmooth(i, 0, 180);
        } else {
            moveServoSmooth(i, 0, 180);
            moveServoSmooth(i, 180, 0);
        }
    }

    // **Step 3: One-at-a-time transition**
    moveServoSmooth(0, 180, 0);  // Servo 1 → 0
    for (int i = 1; i < numServos; i++) {
        moveServoSmooth(i, 0, 180);  // Servo 2-5 → 180 (one at a time)
    }

    // **Step 4: Simultaneous Movement - Move all joints at once**
    moveAllServosSmooth(180, 0);  // Servo 1 → 180, Servos 2-5 → 0
    moveAllServosSmooth(0, 180);  // Servo 1 → 0, Servos 2-5 → 180

    Serial.println("Cycle sequence complete.");
}

// Function to move a single servo smoothly from start to end position
void moveServoSmooth(int servoIndex, int startPos, int endPos) {
    int step = (startPos < endPos) ? 1 : -1;
    
    for (int pos = startPos; pos != endPos + step; pos += step) {
        servos[servoIndex].write(pos);
        delay(10);  // Adjust for smoother motion
    }
}

// Function to move all servos simultaneously to their positions
void moveAllServosSmooth(int thumbTarget, int fingersTarget) {
    int thumbStart = lastPositions[0]; // Current thumb position
    int fingersStart = lastPositions[1]; // Assume all fingers start from same position

    int step = (thumbStart < thumbTarget) ? 1 : -1;
    int stepFingers = (fingersStart < fingersTarget) ? 1 : -1;
    
    int steps = abs(thumbTarget - thumbStart);  // Number of steps required
    int stepsFingers = abs(fingersTarget - fingersStart);  // Number of steps required for fingers
    int maxSteps = max(steps, stepsFingers); // Determine longest movement

    for (int i = 0; i <= maxSteps; i++) {
        // Gradually move each servo towards the target position
        if (i <= steps) servos[0].write(thumbStart + i * step);
        for (int j = 1; j < numServos; j++) {
            if (i <= stepsFingers) servos[j].write(fingersStart + i * stepFingers);
        }
        delay(10);  // Smooth motion delay
    }

    // Save new positions
    lastPositions[0] = thumbTarget;
    for (int j = 1; j < numServos; j++) {
        lastPositions[j] = fingersTarget;
    }
}