#include <Servo.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Servo servos[3];
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28); // IMU I2C address

String input = "";

void setup() {
  Serial1.begin(115200);  // UART to Uno via slip ring
  servos[0].attach(5);
  servos[1].attach(6);
  servos[2].attach(9);

  Wire.begin();
  if (!bno.begin()) {
    Serial1.println("ERROR: IMU not detected");
  } else {
    bno.setExtCrystalUse(true);
    Serial1.println("IMU ready");
  }
}

void loop() {
  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      handleCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void handleCommand(String cmd) {
  cmd.trim();

  if (cmd.startsWith("<servo")) {
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int id = cmd.substring(firstComma + 1, secondComma).toInt();
    int angle = cmd.substring(secondComma + 1, cmd.indexOf('>')).toInt();
    if (id >= 1 && id <= 3) {
      servos[id - 1].write(angle);
      Serial1.println("OK");
    } else {
      Serial1.println("ERR: Invalid servo ID");
    }
  } else if (cmd.startsWith("<imu")) {
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    Serial1.print("Euler: ");
    Serial1.print(orientationData.orientation.x, 1); Serial1.print(", ");
    Serial1.print(orientationData.orientation.y, 1); Serial1.print(", ");
    Serial1.println(orientationData.orientation.z, 1);
  } else {
    Serial1.println("ERR: Unknown command");
  }
}
