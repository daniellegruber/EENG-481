#include <Servo.h>

const int servoPins[] = {3, 5, 6, 9, 10};  // Pins for servos
const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);
Servo servos[numServos];  // Create servo objects

void setup() {
    Serial.begin(9600);
    
    // Attach servos and initialize to default positions (optional)
    for (int i = 0; i < numServos; i++) {
        servos[i].attach(servoPins[i]);
        // Optionally set initial positions
        servos[i].write(90);
    }
    
    Serial.println("Awaiting pattern command (integer 1-10) from Python...");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();  // Remove any whitespace/newlines
        
        // Try converting the input to an integer.
        int pattern = input.toInt();
        if (pattern >= 1 && pattern <= 10) {
            applyPattern(pattern);
        } else {
            // Handle non-numeric or out-of-range inputs.
            Serial.println("Invalid input. Please send an integer between 1 and 10.");
        }
    }
}

// Function to set servo positions based on the received pattern number.
void applyPattern(int pattern) {
    // For demonstration, only pattern 1 is defined.
    // You can add more cases for pattern 2 to 10.
    switch (pattern) {
        case 1:
            // For pattern 1: servo 1 and servo 2 -> 0, servo 3, servo 4, servo 5 -> 180
            servos[0].write(0);   // ID 1
            servos[1].write(0);   // ID 2
            servos[2].write(180); // ID 3
            servos[3].write(180); // ID 4
            servos[4].write(180); // ID 5
            Serial.println("Applied pattern 1");
            break;
        // Example: Define additional patterns as needed.
        case 2:
            // Example pattern 2 (customize as desired)
            servos[0].write(0);
            servos[1].write(0);
            servos[2].write(0);
            servos[3].write(180);
            servos[4].write(180);
            Serial.println("Applied pattern 2");
            break;
        case 3:
            // Example pattern 2 (customize as desired)
            servos[0].write(180);
            servos[1].write(0);
            servos[2].write(0);
            servos[3].write(180);
            servos[4].write(180);
            Serial.println("Applied pattern 3");
            break;
        case 4:
            // For pattern 1: servo 1 and servo 2 -> 0, servo 3, servo 4, servo 5 -> 180
            servos[0].write(0);   // ID 1
            servos[1].write(0);   // ID 2
            servos[2].write(0); // ID 3
            servos[3].write(0); // ID 4
            servos[4].write(0); // ID 5
            Serial.println("Applied pattern 4");
            break;
        // Example: Define additional patterns as needed.
        case 5:
            // Example pattern 2 (customize as desired)
            servos[0].write(180);
            servos[1].write(0);
            servos[2].write(0);
            servos[3].write(0);
            servos[4].write(0);
            Serial.println("Applied pattern 2");
            break;
        case 6:
            // Example pattern 2 (customize as desired)
            servos[0].write(0);
            servos[1].write(0);
            servos[2].write(0);
            servos[3].write(0);
            servos[4].write(180);
            Serial.println("Applied pattern 2");
            break;
        // Add cases for 3 through 10.
        case 7:
            // For pattern 1: servo 1 and servo 2 -> 0, servo 3, servo 4, servo 5 -> 180
            servos[0].write(0);   // ID 1
            servos[1].write(0);   // ID 2
            servos[2].write(0); // ID 3
            servos[3].write(180); // ID 4
            servos[4].write(0); // ID 5
            Serial.println("Applied pattern 1");
            break;
        // Example: Define additional patterns as needed.
        case 8:
            // Example pattern 2 (customize as desired)
            servos[0].write(0);
            servos[1].write(0);
            servos[2].write(180);
            servos[3].write(0);
            servos[4].write(0);
            Serial.println("Applied pattern 2");
            break;
        case 9:
            // Example pattern 2 (customize as desired)
            servos[0].write(0);
            servos[1].write(180);
            servos[2].write(0);
            servos[3].write(0);
            servos[4].write(0);
            Serial.println("Applied pattern 2");
            break;
        default:
            Serial.println("Pattern not defined.");
            break;
    }
}