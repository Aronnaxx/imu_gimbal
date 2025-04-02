# IMU Gimbal Control System

A complete system for controlling a 3-axis gimbal using an Arduino and BNO055 IMU sensor with real-time 3D visualization.

## Overview

This project provides both hardware and software components to:
- Control a 3-axis gimbal using servo motors
- Read and visualize orientation data from a BNO055 IMU sensor
- Visualize the orientation in a 3D plot
- Control servo movement either manually or in random patterns
- Create smooth, continuous motion suitable for IMU testing

## Features

- **Dual Control Modes**:
  - **Random Mode**: Gimbal moves randomly with fluid motion between positions
  - **Control Mode**: Precisely position each servo axis
  
- **Smooth Movement**:
  - Continuous interpolation between positions
  - Adjustable movement speed
  - Random movements of at least 20 degrees for thorough testing
  
- **Real-time 3D Visualization**:
  - Trace of recent orientation history
  - Auto-resizing plot to fit data
  - Current orientation indicator
  
- **Intuitive Control Interface**:
  - Start/stop movement control
  - Fine and coarse servo adjustment buttons
  - IMU zeroing button
  - Plot controls

## Hardware Requirements

- Arduino (tested with Arduino Uno/Nano)
- Adafruit BNO055 9-DOF Orientation Sensor
- 3 servo motors
- Power supply suitable for servos
- Mechanical gimbal mount

## Wiring

Connect your hardware as follows:

1. **BNO055 to Arduino**:
   - VIN → 5V or 3.3V (check your sensor version)
   - GND → GND
   - SDA → A4
   - SCL → A5

2. **Servos to Arduino**:
   - Servo 1 (Yaw) signal → Pin 9
   - Servo 2 (Pitch) signal → Pin 10
   - Servo 3 (Roll) signal → Pin 11
   - All servo GND → Arduino GND
   - All servo power → External power supply (recommended)

## Software Requirements

- Arduino IDE
- Python 3.6+ with the following packages:
  - pyserial
  - matplotlib
  - numpy
  - tkinter (usually included with Python)

## Installation

1. **Upload Arduino Sketch**:
   - Install required libraries in Arduino IDE:
     - Adafruit BNO055
     - Adafruit Unified Sensor
     - Servo
   - Upload `arduino_serial.ino` to your Arduino board

2. **Setup Python Environment**:
   ```bash
   pip install pyserial matplotlib numpy
   ```

3. **Configure Serial Port**:
   - Edit the `PORT` variable in `imu_trace_3d.py` to match your Arduino's serial port
   - Default is `/dev/tty.usbmodem101` for macOS, use appropriate port for your OS

## Usage

1. **Start the Program**:
   ```bash
   python imu_trace_3d.py
   ```

2. **Control Panel Functions**:

   - **Mode Selection**:
     - **Random Mode**: Servo positions change randomly (at least 20° per change)
     - **Control Mode**: Manually control servo positions

   - **Start/Stop Controls**:
     - Click **Start** to begin movement in the selected mode
     - Click **Stop** to halt all servo movement

   - **Servo Controls**:
     - Use sliders to set precise positions
     - Use buttons for incremental adjustments:
       - `-10`/`+10`: Change by 10 degrees
       - `-1`/`+1`: Fine adjustment by 1 degree
       - `C`: Center the servo (90 degrees)

   - **Movement Speed**:
     - Adjust how quickly servos move between positions

   - **Plot Controls**:
     - Toggle **Auto-resize plot** to adapt axis limits to data
     - Use **Reset Plot** to clear the trace history

   - **IMU Controls**:
     - **Zero IMU** resets the sensor orientation reference

   - **Apply Settings**:
     - Click to send control mode and manual position updates to Arduino

3. **3D Visualization**:
   - The main plot shows the current orientation of the IMU
   - Red dot shows current position
   - Blue line shows orientation history
   - Plot automatically adjusts to show data range

## Troubleshooting

- **No Serial Connection**:
  - Check that the PORT variable matches your Arduino's port
  - Verify the Arduino is connected and powered

- **Erratic Servo Movement**:
  - Ensure servos have adequate power supply
  - Reduce speed if movements appear too sudden

- **IMU Not Detected**:
  - Check your wiring connections
  - Verify the I2C address (default 0x28, may be 0x29)

- **Matplotlib Display Issues**:
  - Try updating matplotlib: `pip install --upgrade matplotlib`

## Extending the Project

- Add recording capabilities for IMU data
- Implement specific motion patterns for testing
- Add PID-based stabilization using the IMU as feedback
- Add more visualization options (2D plots of individual axes)

## License

This project is licensed under the MIT License - see the LICENSE file for details. 
