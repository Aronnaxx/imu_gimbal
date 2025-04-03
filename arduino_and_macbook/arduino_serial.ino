#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // Adjust the I2C address if needed

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!bno.begin()) {
    Serial.println("BN055 not detected.");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Setup complete");
}

void processCommand(String command) {
  command.trim(); // Remove any whitespace or newlines
  
  if (command.startsWith("ZERO")) {
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
