#include <Servo.h>

const int servoPins[] = {3, 5, 6, 9, 10};  // Servo output pins
const int numServos = sizeof(servoPins) / sizeof(servoPins[0]);
Servo servos[numServos];

void setup() {
    Serial.begin(9600);
    for (int i = 0; i < numServos; i++) {
        servos[i].attach(servoPins[i]);
        servos[i].write(90);  // Optionally initialize servos to 90°
    }
    Serial.println("Ready to receive servo angles (format: a,b,c,d,e)");
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');  // Read until newline
        input.trim();  // Remove any extraneous whitespace
        
        // Parse five comma-separated integers.
        int a, b, c, d, e;
        if (sscanf(input.c_str(), "%d,%d,%d,%d,%d", &a, &b, &c, &d, &e) == 5) {
            // Write each value to the corresponding servo.
            servos[0].write(a*1);  // Thumb servo
            servos[1].write(b*3);  // Index servo
            servos[2].write(c*2);  // Middle servo
            servos[3].write(d*2);  // Ring servo
            servos[4].write(e*2);  // Pinky servo
            
            Serial.print("Set servos to: ");
            Serial.print(a); Serial.print(", ");
            Serial.print(b); Serial.print(", ");
            Serial.print(c); Serial.print(", ");
            Serial.print(d); Serial.print(", ");
            Serial.println(e);
        } else {
            Serial.println("Invalid format. Expected: a,b,c,d,e");
        }
    }
}