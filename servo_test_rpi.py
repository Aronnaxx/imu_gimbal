import time
import random
from adafruit_servokit import ServoKit

# Set up 16-channel PCA9685
kit = ServoKit(channels=16)

# List of servo channels you're using
servo_channels = [0, 1, 2]  # Yaw, Pitch, Roll, Turntable

# Set pulse width range if needed (optional tuning for your servos)
for ch in servo_channels:
    kit.servo[ch].set_pulse_width_range(500, 2500)

print("Starting random servo movement test!")

try:
    while True:
        for ch in servo_channels:
            angle = random.randint(30, 150)
            print(f"Channel {ch} -> {angle}°")
            kit.servo[ch].angle = angle
        time.sleep(1.0)  # Wait a second between updates

except KeyboardInterrupt:
    print("\nTest stopped by user.")
