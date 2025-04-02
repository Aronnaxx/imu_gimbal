#include <Servo.h>

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

// Current servo positions
int servo1_pos = 90;
int servo2_pos = 90;
int servo3_pos = 90;

void setup() {
  // Attach servos to digital pins
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);

  // Initialize servos to center position
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);

  // Initialize random seed
  randomSeed(analogRead(0));
}

void loop() {
  // Generate random positions between 0 and 180 degrees
  servo1_pos = random(0, 180);
  servo2_pos = random(0, 180);
  servo3_pos = random(0, 180);

  // Move servos to new positions
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);

  // Wait for 1 second before next movement
  delay(1000);
} 
