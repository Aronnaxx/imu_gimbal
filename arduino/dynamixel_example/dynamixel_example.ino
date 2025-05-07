#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

const uint8_t DXL1_ID = 1;     // First Dynamixel ID
const uint8_t DXL2_ID = 2;     // Second Dynamixel ID
const uint8_t DXL3_ID = 3;     // Third Dynamixel ID
const float DXL_PROTOCOL_VERSION = 2.0;
const int SERVO_SPEED = 20;    // Speed for continuous rotation (0-1023)


DynamixelShield dxl;

//This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {
  // put your setup code here, to run once:
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);
  
  // Set Port baudrate to 57600bps. This has to match with DYNAMIXEL baudrate.
  dxl.begin(57600);
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // Configure Dynamixel servos
  uint8_t servo_ids[] = {DXL1_ID, DXL2_ID, DXL3_ID};
  for(int i = 0; i < 3; i++) {
    uint8_t id = servo_ids[i];
    if(dxl.ping(id)) {
      Serial.print("Found Dynamixel ID: ");
      Serial.println(id);
      // Set to velocity control mode
      dxl.torqueOff(id);
      dxl.setOperatingMode(id, OP_VELOCITY);
      dxl.torqueOn(id);
    } else {
      Serial.print("Failed to find Dynamixel ID: ");
      Serial.println(id);
    }
  }

  Serial.println("Setup complete");
}

void loop() {
  // Update Dynamixel servo speeds
  // Setting alternating directions for interesting motion
  dxl.setGoalVelocity(DXL1_ID, SERVO_SPEED);
  dxl.setGoalVelocity(DXL2_ID, -SERVO_SPEED);  // Opposite direction
  dxl.setGoalVelocity(DXL3_ID, SERVO_SPEED);   // Same as first servo

  // Print current velocities for monitoring
  Serial.print("Velocities (RPM) - DXL1: ");
  Serial.print(dxl.getPresentVelocity(DXL1_ID));
  Serial.print(", DXL2: ");
  Serial.print(dxl.getPresentVelocity(DXL2_ID));
  Serial.print(", DXL3: ");
  Serial.println(dxl.getPresentVelocity(DXL3_ID));

  delay(100); // Update at 10Hz, which is sufficient for monitoring
}