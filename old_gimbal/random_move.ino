#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Servo.h>

Servo servo1, servo2, servo3;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Adjust the I2C address if needed

// Current servo positions
int servo1_pos = 90;
int servo2_pos = 90;
int servo3_pos = 90;

// Target positions for smooth movement
int servo1_target = 90;
int servo2_target = 90;
int servo3_target = 90;

// Movement speed (degrees per update)
float movement_speed = 2.0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Attach servos to digital pins
  servo1.attach(8);
  servo2.attach(9);
  servo3.attach(10);

  // Initialize servos to center position
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);

  if (!bno.begin()) {
    Serial.println("BN055 not detected.");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Setup complete");
}

void setNewRandomTargets() {
  // Generate new targets at least 20 degrees away from current position
  int new_pos1, new_pos2, new_pos3;
  
  do {
    new_pos1 = random(0, 180);
  } while (abs(new_pos1 - servo1_pos) < 20);
  
  do {
    new_pos2 = random(0, 180);
  } while (abs(new_pos2 - servo2_pos) < 20);
  
  do {
    new_pos3 = random(0, 180);
  } while (abs(new_pos3 - servo3_pos) < 20);
  
  servo1_target = new_pos1;
  servo2_target = new_pos2;
  servo3_target = new_pos3;
  
  Serial.print("New random targets: ");
  Serial.print(servo1_target);
  Serial.print(", ");
  Serial.print(servo2_target);
  Serial.print(", ");
  Serial.println(servo3_target);
}

void updateServos() {
  // Smoothly interpolate between current position and target
  bool servo1_done = moveServoTowardTarget(&servo1_pos, servo1_target);
  bool servo2_done = moveServoTowardTarget(&servo2_pos, servo2_target);  
  bool servo3_done = moveServoTowardTarget(&servo3_pos, servo3_target);
  
  // Write the new positions to servos
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  
  // If all servos have reached targets and we're in random mode, set new targets
  if (servo1_done && servo2_done && servo3_done && Serial.available() <= 0) {
    setNewRandomTargets();
  }
}

bool moveServoTowardTarget(int* current, int target) {
  if (*current == target) {
    return true; // Already at target
  }
  
  // Move toward target at specified speed
  if (*current < target) {
    *current = min(target, *current + (int)movement_speed);
  } else {
    *current = max(target, *current - (int)movement_speed);
  }
  
  return (*current == target);
}

void processCommand(String command) {
  command.trim(); // Remove any whitespace or newlines
  
  if (command.startsWith("RANDOM")) {
    // Set new random targets
    setNewRandomTargets();
  }
  else if (command.startsWith("CONTROL")) {
    // Parse the command format: CONTROL,servo1,servo2,servo3
    int firstComma = command.indexOf(',');
    int secondComma = command.indexOf(',', firstComma + 1);
    int thirdComma = command.indexOf(',', secondComma + 1);
    
    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      servo1_target = command.substring(firstComma + 1, secondComma).toInt();
      servo2_target = command.substring(secondComma + 1, thirdComma).toInt();
      servo3_target = command.substring(thirdComma + 1).toInt();
      
      // Constrain servo positions to valid range
      servo1_target = constrain(servo1_target, 0, 180);
      servo2_target = constrain(servo2_target, 0, 180);
      servo3_target = constrain(servo3_target, 0, 180);
      
      Serial.print("Control mode targets: ");
      Serial.print(servo1_target);
      Serial.print(", ");
      Serial.print(servo2_target);
      Serial.print(", ");
      Serial.println(servo3_target);
    }
  }
  else if (command.startsWith("SPEED")) {
    // Allow adjusting the movement speed
    int comma = command.indexOf(',');
    if (comma != -1) {
      float new_speed = command.substring(comma + 1).toFloat();
      if (new_speed > 0) {
        movement_speed = new_speed;
        Serial.print("Movement speed set to: ");
        Serial.println(movement_speed);
      }
    }
  }
}

void loop() {
  // Check for commands from Python
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Update servo positions (smooth movement)
  updateServos();

  // Read Euler orientation data from the BN055
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // Print Euler angles (yaw, pitch, roll) to serial in a simple format
  Serial.print("Euler: ");
  Serial.print(orientationData.orientation.x); // Yaw
  Serial.print(", ");
  Serial.print(orientationData.orientation.y); // Pitch
  Serial.print(", ");
  Serial.println(orientationData.orientation.z); // Roll

  delay(20); // Update more frequently for smoother motion (50Hz)
}
