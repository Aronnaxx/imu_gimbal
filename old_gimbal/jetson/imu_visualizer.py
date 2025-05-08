import sys
import time
import yaml
import tkinter as tk
from tkinter import ttk
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import numpy as np
from PIL import Image, ImageTk, ImageDraw
import math
import colorsys
import board
import busio
import adafruit_bno055

# Load configuration
CONFIG_FILE = 'config.yaml'
try:
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)
except Exception as e:
    print(f"Error loading config file: {e}")
    config = {}

# Constants
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"
SLIDER_COLOR = "#3498db"
SUCCESS_COLOR = "#2ecc71"
DANGER_COLOR = "#e74c3c"

# Performance settings
REDRAW_INTERVAL = 10  # ms between redraws
DATA_HISTORY_LENGTH = 200  # Number of data points to keep
QUIVER_SCALE = 30  # Scale of the direction arrow

class AngleUnwrapper:
    """Handles continuous angle tracking across 0/360 boundary"""
    def __init__(self):
        self.previous_angle = None
        self.offset = 0
        
    def unwrap(self, angle):
        if self.previous_angle is None:
            self.previous_angle = angle
            return angle
        
        diff = angle - self.previous_angle
        if diff > 180:
            self.offset -= 360
        elif diff < -180:
            self.offset += 360
            
        self.previous_angle = angle
        return angle + self.offset
    
    def reset(self):
        self.previous_angle = None
        self.offset = 0

class KalmanFilter3D:
    """3D Kalman filter for orientation data"""
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        self.state = np.zeros(6)  # [yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate]
        self.covariance = np.eye(6) * 1000
        self.Q = np.eye(6) * process_noise
        self.R = np.eye(3) * measurement_noise
        self.F = np.eye(6)
        self.F[0:3, 3:6] = np.eye(3)
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)
        self.dt = 0.01
        
    def predict(self):
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        
    def update(self, measurement):
        K = self.covariance @ self.H.T @ np.linalg.inv(self.H @ self.covariance @ self.H.T + self.R)
        innovation = measurement - self.H @ self.state
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        return self.state[0:3]

class BNO055_IMU:
    """Interface for BNO055 IMU sensor"""
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        try:
            self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)
            print("BNO055 initialized successfully")
        except Exception as e:
            print(f"Error initializing BNO055: {e}")
            raise

    def read_euler(self):
        try:
            euler = self.sensor.euler
            if euler is not None:
                yaw, roll, pitch = euler
                yaw = (yaw + 360) % 360
                return yaw, pitch, roll
            return None
        except Exception as e:
            print(f"Error reading Euler angles: {e}")
            return None

    def zero_imu(self):
        try:
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
        try:
            return self.sensor.calibration_status
        except Exception as e:
            print(f"Error getting calibration status: {e}")
            return (0, 0, 0, 0)

    def close(self):
        try:
            self.i2c.deinit()
        except Exception as e:
            print(f"Error closing I2C: {e}")

class IMUVisualizer:
    """Main visualization class"""
    def __init__(self, root):
        self.root = root
        self.root.title("IMU Orientation Visualizer")
        self.root.configure(bg=DARK_BG)
        
        # Initialize IMU
        try:
            self.imu = BNO055_IMU()
        except Exception as e:
            print(f"Failed to initialize BNO055: {e}")
            sys.exit(1)

        # Setup data storage
        self.x_data, self.y_data, self.z_data = [], [], []
        self.x_filtered, self.y_filtered, self.z_filtered = [], [], []
        self.kalman_filter = KalmanFilter3D()
        self.yaw_unwrapper = AngleUnwrapper()
        
        # Setup UI
        self.setup_ui()
        
        # Start update loop
        self.update_active = True
        self.root.after(10, self.update_loop)

    def setup_ui(self):
        """Setup the user interface"""
        # Main container
        self.main_frame = ttk.Frame(self.root, padding="10")
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        
        # Setup matplotlib figure
        self.setup_plot()
        
        # Setup controls
        self.setup_controls()
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=1)
        self.main_frame.rowconfigure(0, weight=1)

    def setup_plot(self):
        """Setup the matplotlib plot"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(8, 6), facecolor=DARK_BG)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(DARKER_BG)
        
        # Create visualization elements
        self.line, = self.ax.plot([], [], [], lw=2, color=HIGHLIGHT)
        self.filtered_line, = self.ax.plot([], [], [], lw=2, color=SUCCESS_COLOR)
        self.dot = self.ax.plot([], [], [], 'o', color=ACCENT_COLOR, markersize=8)[0]
        self.quiver = self.ax.quiver([0], [0], [0], [0], [0], [1], color=DANGER_COLOR,
                                   length=QUIVER_SCALE, normalize=True)
        
        # Set labels and limits
        self.ax.set_xlim(-180, 180)
        self.ax.set_ylim(-180, 180)
        self.ax.set_zlim(-180, 180)
        self.ax.set_xlabel("Yaw")
        self.ax.set_ylabel("Pitch")
        self.ax.set_zlabel("Roll")
        
        # Create canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.main_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().grid(row=0, column=1, sticky="nsew")

    def setup_controls(self):
        """Setup control panel"""
        control_frame = ttk.LabelFrame(self.main_frame, text="Controls", padding="10")
        control_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        
        # Continuous yaw tracking
        self.continuous_yaw_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(control_frame, text="Continuous Yaw",
                       variable=self.continuous_yaw_var).pack(pady=5)
        
        # Zero IMU button
        ttk.Button(control_frame, text="Zero IMU",
                  command=self.zero_imu).pack(pady=5)
        
        # Calibration status
        self.cal_status_var = tk.StringVar(value="Calibration Status: Unknown")
        ttk.Label(control_frame, textvariable=self.cal_status_var).pack(pady=5)
        
        # Current angles
        self.angles_var = tk.StringVar(value="Yaw: 0°\nPitch: 0°\nRoll: 0°")
        ttk.Label(control_frame, textvariable=self.angles_var).pack(pady=5)

    def update_loop(self):
        """Main update loop"""
        if not self.update_active:
            return
            
        # Read from IMU
        euler = self.imu.read_euler()
        if euler:
            yaw, pitch, roll = euler
            
            # Apply continuous yaw if enabled
            if self.continuous_yaw_var.get():
                yaw = self.yaw_unwrapper.unwrap(yaw)
            
            # Apply Kalman filter
            measurement = np.array([yaw, pitch, roll])
            self.kalman_filter.predict()
            filtered = self.kalman_filter.update(measurement)
            
            # Update data arrays
            self.x_data.append(yaw)
            self.y_data.append(pitch)
            self.z_data.append(roll)
            
            self.x_filtered.append(filtered[0])
            self.y_filtered.append(filtered[1])
            self.z_filtered.append(filtered[2])
            
            # Limit history
            if len(self.x_data) > DATA_HISTORY_LENGTH:
                self.x_data = self.x_data[-DATA_HISTORY_LENGTH:]
                self.y_data = self.y_data[-DATA_HISTORY_LENGTH:]
                self.z_data = self.z_data[-DATA_HISTORY_LENGTH:]
                self.x_filtered = self.x_filtered[-DATA_HISTORY_LENGTH:]
                self.y_filtered = self.y_filtered[-DATA_HISTORY_LENGTH:]
                self.z_filtered = self.z_filtered[-DATA_HISTORY_LENGTH:]
            
            # Update plot
            self.update_plot()
            
            # Update status displays
            self.update_status(filtered)
        
        # Schedule next update
        self.root.after(10, self.update_loop)

    def update_plot(self):
        """Update the plot with new data"""
        if len(self.x_data) > 0:
            # Update lines
            self.line.set_data(self.x_data, self.y_data)
            self.line.set_3d_properties(self.z_data)
            
            self.filtered_line.set_data(self.x_filtered, self.y_filtered)
            self.filtered_line.set_3d_properties(self.z_filtered)
            
            # Update current position dot
            self.dot.set_data([self.x_filtered[-1]], [self.y_filtered[-1]])
            self.dot.set_3d_properties([self.z_filtered[-1]])
            
            # Update direction arrow
            pos = np.array([[self.x_filtered[-1], self.y_filtered[-1], self.z_filtered[-1]]])
            direction = self.euler_to_vector(self.x_filtered[-1], self.y_filtered[-1], self.z_filtered[-1])
            self.quiver.set_segments([np.concatenate((pos, pos + direction * QUIVER_SCALE))])
            
            # Redraw
            self.canvas.draw()

    def update_status(self, filtered):
        """Update status displays"""
        # Update angles display
        self.angles_var.set(
            f"Yaw: {filtered[0]:.1f}°\n"
            f"Pitch: {filtered[1]:.1f}°\n"
            f"Roll: {filtered[2]:.1f}°"
        )
        
        # Update calibration status
        cal = self.imu.get_calibration_status()
        if cal:
            sys, gyro, accel, mag = cal
            self.cal_status_var.set(
                f"Calibration Status:\n"
                f"Sys: {sys}/3\n"
                f"Gyro: {gyro}/3\n"
                f"Accel: {accel}/3\n"
                f"Mag: {mag}/3"
            )

    def zero_imu(self):
        """Zero the IMU"""
        if self.imu.zero_imu():
            self.kalman_filter = KalmanFilter3D()
            self.yaw_unwrapper.reset()

    @staticmethod
    def euler_to_vector(yaw, pitch, roll):
        """Convert Euler angles to direction vector"""
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        x = math.cos(yaw_rad) * math.cos(pitch_rad)
        y = math.sin(yaw_rad) * math.cos(pitch_rad)
        z = math.sin(pitch_rad)
        
        return np.array([x, y, z])

    def cleanup(self):
        """Clean up resources"""
        self.update_active = False
        if hasattr(self, 'imu'):
            self.imu.close()
        print("IMU Visualizer closed")

def main():
    root = tk.Tk()
    app = IMUVisualizer(root)
    try:
        root.mainloop()
    finally:
        app.cleanup()

if __name__ == "__main__":
    main() 
