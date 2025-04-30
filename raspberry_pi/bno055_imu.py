import time
import board
import busio
import adafruit_bno055

class BNO055_IMU:
    def __init__(self):
        # Initialize I2C bus
        self.i2c = busio.I2C(board.SCL, board.SDA)
        try:
            # Initialize the BNO055 sensor
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
            print("BNO055 initialized successfully")
        except Exception as e:
            print(f"Error initializing BNO055: {e}")
            raise

    def read_euler(self):
        """Read Euler angles from the sensor.
        Returns:
            tuple: (yaw, pitch, roll) in degrees
        """
        try:
            euler = self.sensor.euler
            if euler is not None:
                # BNO055 returns (yaw, roll, pitch) but we want (yaw, pitch, roll)
                yaw, roll, pitch = euler
                # Ensure values are within expected ranges
                yaw = (yaw + 360) % 360  # Convert to 0-360 range
                return yaw, pitch, roll
            return None
        except Exception as e:
            print(f"Error reading Euler angles: {e}")
            return None

    def zero_imu(self):
        """Reset the IMU's orientation reference."""
        try:
            # The BNO055 doesn't have a direct "zero" command, but we can:
            # 1. Switch to CONFIG mode
            # 2. Wait a moment
            # 3. Switch back to NDOF mode
            self.sensor.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.1)
            self.sensor.mode = adafruit_bno055.NDOF_MODE
            time.sleep(0.1)
            print("IMU zeroed")
            return True
        except Exception as e:
            print(f"Error zeroing IMU: {e}")
            return False

    def get_calibration_status(self):
        """Get calibration status of the IMU.
        Returns:
            tuple: (system, gyro, accel, mag) calibration status (0-3, 3 is fully calibrated)
        """
        try:
            return self.sensor.calibration_status
        except Exception as e:
            print(f"Error getting calibration status: {e}")
            return (0, 0, 0, 0)

    def get_temperature(self):
        """Get the sensor's temperature.
        Returns:
            float: Temperature in degrees Celsius
        """
        try:
            return self.sensor.temperature
        except Exception as e:
            print(f"Error reading temperature: {e}")
            return None

    def close(self):
        """Clean up I2C resources."""
        try:
            self.i2c.deinit()
        except Exception as e:
            print(f"Error closing I2C: {e}") 
