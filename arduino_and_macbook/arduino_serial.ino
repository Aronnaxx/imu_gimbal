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

// Movement state
bool random_mode_active = false;
bool movement_active = false;

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
  // Only update if movement is active
  if (!movement_active) {
    return;
  }
  
  // Smoothly interpolate between current position and target
  bool servo1_done = moveServoTowardTarget(&servo1_pos, servo1_target);
  bool servo2_done = moveServoTowardTarget(&servo2_pos, servo2_target);  
  bool servo3_done = moveServoTowardTarget(&servo3_pos, servo3_target);
  
  // Write the new positions to servos
  servo1.write(servo1_pos);
  servo2.write(servo2_pos);
  servo3.write(servo3_pos);
  
  // If all servos have reached targets and we're in random mode, set new targets
  if (servo1_done && servo2_done && servo3_done && random_mode_active) {
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
    // Enable random mode but don't start movement yet
    random_mode_active = true;
    movement_active = false;
    Serial.println("Random mode enabled. Use START to begin movement.");
  }
  else if (command.startsWith("CONTROL")) {
    // Switch to control mode
    random_mode_active = false;
    movement_active = true;
    
    // Parse the command format: CONTROL,servo1,servo2,servo3
    int firstComma = command.indexOf(',');
    
    if (firstComma != -1) {
      String rest = command.substring(firstComma + 1);
      int secondComma = rest.indexOf(',');
      
      if (secondComma != -1) {
        String servo1Str = rest.substring(0, secondComma);
        rest = rest.substring(secondComma + 1);
        int thirdComma = rest.indexOf(',');
        
        if (thirdComma != -1) {
          String servo2Str = rest.substring(0, thirdComma);
          String servo3Str = rest.substring(thirdComma + 1);
          
          // Convert to integers with proper error checking
          int s1 = servo1Str.toInt();
          int s2 = servo2Str.toInt();
          int s3 = servo3Str.toInt();
          
          servo1_target = constrain(s1, 0, 180);
          servo2_target = constrain(s2, 0, 180);
          servo3_target = constrain(s3, 0, 180);
          
          Serial.print("Control mode targets: ");
          Serial.print(servo1_target);
          Serial.print(", ");
          Serial.print(servo2_target);
          Serial.print(", ");
          Serial.println(servo3_target);
        }
      }
    }
  }
  else if (command.startsWith("START")) {
    // Start movement based on current mode
    if (random_mode_active) {
      setNewRandomTargets(); // Set initial random targets
    }
    movement_active = true;
    Serial.println("Movement started");
  }
  else if (command.startsWith("STOP")) {
    // Stop movement
    movement_active = false;
    Serial.println("Movement stopped");
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
  else if (command.startsWith("ZERO")) {
    // Zero out the IMU calibration
    bno.setExtCrystalUse(false);
    delay(10);
    bno.setExtCrystalUse(true);
    Serial.println("IMU reset requested");
  }
}

void loop() {
  // Process all available commands from Python
  while (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.length() > 0) {
      processCommand(command);
    }
  }

  // Update servo positions (smooth movement)
  updateServos();

  // Read Euler orientation data from the BN055
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // Print Euler angles (yaw, pitch, roll) to serial in a simple format
  Serial.print("Euler: ");
  Serial.print(orientationData.orientation.x, 2); // Yaw with 2 decimal places
  Serial.print(", ");
  Serial.print(orientationData.orientation.y, 2); // Pitch with 2 decimal places
  Serial.print(", ");
  Serial.println(orientationData.orientation.z, 2); // Roll with 2 decimal places

  delay(10); // Update more frequently for smoother motion (100Hz)
}
