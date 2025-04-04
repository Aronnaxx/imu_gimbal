import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import numpy as np
import serial.tools.list_ports
from PIL import Image, ImageTk, ImageDraw  # For custom widget rendering
import math
import colorsys

# Kalman Filter implementation for 3D orientation
class KalmanFilter3D:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        # State vector: [yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1000  # Initial uncertainty
        
        # Process noise covariance
        self.Q = np.eye(6) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
        
        # State transition matrix (assuming constant velocity model)
        self.F = np.eye(6)
        self.F[0:3, 3:6] = np.eye(3)  # Position depends on velocity
        
        # Measurement matrix (we only measure position)
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)
        
        # Time step (in seconds)
        self.dt = 0.01  # 10ms update rate
        
    def predict(self):
        # Predict state
        self.state = self.F @ self.state
        
        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        
    def update(self, measurement):
        # Kalman gain
        K = self.covariance @ self.H.T @ np.linalg.inv(self.H @ self.covariance @ self.H.T + self.R)
        
        # Update state
        innovation = measurement - self.H @ self.state
        self.state = self.state + K @ innovation
        
        # Update covariance
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        
        # Return filtered measurement
        return self.state[0:3]

# Custom theme and style constants
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"
SLIDER_COLOR = "#3498db"
SUCCESS_COLOR = "#2ecc71"
DANGER_COLOR = "#e74c3c"

# Find Arduino port automatically
def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'Arduino' in port.description or 'usbmodem' in port.device:
            return port.device
    return None

# Custom styling and widgets for modern UI
def setup_styles():
    """Configure ttk styles for a modern dark theme"""
    style = ttk.Style()
    style.theme_use('clam')  # Base theme
    
    # Configure colors
    style.configure('TFrame', background=DARK_BG)
    style.configure('TLabel', background=DARK_BG, foreground=TEXT_COLOR)
    style.configure('TButton', background=DARKER_BG, foreground=TEXT_COLOR, borderwidth=0, 
                   focusthickness=0, padding=6)
    style.map('TButton', 
              background=[('active', HIGHLIGHT), ('disabled', DARKER_BG)],
              foreground=[('disabled', '#888888')])
    
    # Special buttons
    style.configure('Start.TButton', background=SUCCESS_COLOR)
    style.map('Start.TButton', background=[('active', '#27ae60')])
    
    style.configure('Stop.TButton', background=DANGER_COLOR)
    style.map('Stop.TButton', background=[('active', '#c0392b')])
    
    style.configure('TRadiobutton', background=DARK_BG, foreground=TEXT_COLOR)
    style.configure('TCheckbutton', background=DARK_BG, foreground=TEXT_COLOR)
    
    # Configure LabelFrame
    style.configure('TLabelframe', background=DARK_BG)
    style.configure('TLabelframe.Label', background=DARK_BG, foreground=TEXT_COLOR)
    
    # Configure Scale (slider)
    style.configure('TScale', background=DARK_BG, troughcolor=DARKER_BG)

# Try to find Arduino port automatically, otherwise use default
PORT = find_arduino_port() or '/dev/tty.usbmodem101'
BAUD = 115200

# Initialize serial connection
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"Connected to {PORT} at {BAUD} baud")
except serial.SerialException as e:
    print(f"Error connecting to serial port: {e}")
    print("Available ports:")
    for port in serial.tools.list_ports.comports():
        print(f" - {port.device}: {port.description}")
    raise

# Lists to hold Euler angle data: x (yaw), y (pitch), z (roll)
x_data, y_data, z_data = [], [], []
x_filtered, y_filtered, z_filtered = [], [], []

# Initialize Kalman filter
kalman_filter = KalmanFilter3D(process_noise=0.1, measurement_noise=1.0)

# Regular expression to parse serial data of the form: "Euler: 45.0, -30.0, 10.0"
euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

# Auto-resize plot flag
auto_resize = True
plot_range = 180  # Initial plot range

# Create main Tkinter window
root = tk.Tk()
root.title("IMU Orientation Visualizer")
root.geometry("1300x800")
root.configure(bg=DARK_BG)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Setup modern styles
setup_styles()

# Create frame to hold everything
main_frame = ttk.Frame(root)
main_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=10, pady=10)
main_frame.columnconfigure(0, weight=3)  # Plot area
main_frame.columnconfigure(1, weight=1)  # Control panel
main_frame.rowconfigure(0, weight=1)

# Create matplotlib figure with dark theme
plt.style.use('dark_background')
fig = plt.figure(figsize=(8, 6), facecolor=DARK_BG)
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor(DARKER_BG)
line, = ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
filtered_line, = ax.plot([], [], [], lw=2, label='Filtered Path', color=SUCCESS_COLOR)
dot = ax.plot([], [], [], marker='o', label='Current Orientation', color=ACCENT_COLOR, markersize=8)[0]

# Set initial axis limits
ax.set_xlim(-plot_range, plot_range)
ax.set_ylim(-plot_range, plot_range)
ax.set_zlim(-plot_range, plot_range)
ax.set_title("3D Orientation Trace (Yaw, Pitch, Roll)", color=TEXT_COLOR, fontsize=14)
ax.set_xlabel("Yaw", color=TEXT_COLOR)
ax.set_ylabel("Pitch", color=TEXT_COLOR)
ax.set_zlabel("Roll", color=TEXT_COLOR)
ax.tick_params(colors=TEXT_COLOR)
ax.grid(True, linestyle='--', alpha=0.3)
ax.legend(facecolor=DARKER_BG, edgecolor=HIGHLIGHT)

# Embed matplotlib figure in Tkinter
plot_frame = ttk.Frame(main_frame)
plot_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.draw()
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Create control panel with modern styling
control_panel = ttk.Frame(main_frame, padding="15")
control_panel.grid(column=1, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))

# Plot control frame
plot_frame_controls = ttk.LabelFrame(control_panel, text="Plot Controls", padding="10")
plot_frame_controls.pack(fill=tk.X, pady=10)

# Auto-resize toggle with better styling
auto_resize_var = tk.BooleanVar(value=auto_resize)
auto_resize_check = ttk.Checkbutton(plot_frame_controls, text="Auto-resize plot", 
                                   variable=auto_resize_var)
auto_resize_check.pack(anchor=tk.W, pady=5)

# Reset plot button
def reset_plot():
    global x_data, y_data, z_data, x_filtered, y_filtered, z_filtered
    x_data.clear()
    y_data.clear()
    z_data.clear()
    x_filtered.clear()
    y_filtered.clear()
    z_filtered.clear()
    update_plot_limits()
    canvas.draw_idle()

ttk.Button(plot_frame_controls, text="Reset Plot", command=reset_plot).pack(anchor=tk.W, pady=5, fill=tk.X)

# IMU control frame
imu_frame = ttk.LabelFrame(control_panel, text="IMU Controls", padding="10")
imu_frame.pack(fill=tk.X, pady=10)

# Zero IMU button with better styling
def zero_imu():
    ser.write(b"ZERO\n")
    ser.flush()
    print("Zeroing IMU")
    # Reset Kalman filter
    global kalman_filter
    kalman_filter = KalmanFilter3D(process_noise=0.1, measurement_noise=1.0)

ttk.Button(imu_frame, text="Zero IMU", command=zero_imu).pack(fill=tk.X, pady=5)

# Status frame with better visualization
status_frame = ttk.LabelFrame(control_panel, text="Status", padding="10")
status_frame.pack(fill=tk.X, pady=10)

# Current angles display with better styling
angles_display_frame = ttk.Frame(status_frame)
angles_display_frame.pack(fill=tk.X, pady=5)

# Variables for each angle
yaw_var = tk.DoubleVar(value=0.0)
pitch_var = tk.DoubleVar(value=0.0)
roll_var = tk.DoubleVar(value=0.0)

# Create better angle displays
angle_display = ttk.Frame(status_frame)
angle_display.pack(fill=tk.X, pady=5)
angle_display.columnconfigure(1, weight=1)

# Yaw display
ttk.Label(angle_display, text="Yaw:").grid(row=0, column=0, sticky=tk.W, pady=2)
yaw_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                              maximum=180, value=90)
yaw_progress.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=5, pady=2)
ttk.Label(angle_display, textvariable=yaw_var).grid(row=0, column=2, sticky=tk.E, pady=2)

# Pitch display
ttk.Label(angle_display, text="Pitch:").grid(row=1, column=0, sticky=tk.W, pady=2)
pitch_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                               maximum=180, value=90)
pitch_progress.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=5, pady=2)
ttk.Label(angle_display, textvariable=pitch_var).grid(row=1, column=2, sticky=tk.E, pady=2)

# Roll display
ttk.Label(angle_display, text="Roll:").grid(row=2, column=0, sticky=tk.W, pady=2)
roll_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                              maximum=180, value=90)
roll_progress.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=5, pady=2)
ttk.Label(angle_display, textvariable=roll_var).grid(row=2, column=2, sticky=tk.E, pady=2)

# Update angle display function
def update_angle_display(yaw, pitch, roll):
    """Update the angle display with current values"""
    # Update variables
    yaw_var.set(f"{yaw:.1f}°")
    pitch_var.set(f"{pitch:.1f}°")
    roll_var.set(f"{roll:.1f}°")
    
    # Update progress bars (adjust for visualization)
    # Map angles to 0-180 range for progress bars
    yaw_progress['value'] = (yaw + 90) % 180
    pitch_progress['value'] = (pitch + 90) % 180
    roll_progress['value'] = (roll + 90) % 180

# Function to update plot limits based on data
def update_plot_limits():
    if not auto_resize_var.get() or not x_data:
        return
    
    # Calculate needed range with some padding using filtered data
    x_min, x_max = min(x_filtered), max(x_filtered)
    y_min, y_max = min(y_filtered), max(y_filtered)
    z_min, z_max = min(z_filtered), max(z_filtered)
    
    # Add 10% padding
    x_range = max(abs(x_min), abs(x_max)) * 1.1
    y_range = max(abs(y_min), abs(y_max)) * 1.1
    z_range = max(abs(z_min), abs(z_max)) * 1.1
    
    # Use the largest range for all axes to maintain aspect ratio
    max_range = max(x_range, y_range, z_range, 20)  # Minimum range of 20 degrees
    
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)

# Function to update the plot
def update_plot():
    global x_data, y_data, z_data, x_filtered, y_filtered, z_filtered
    
    # Read all available data from the serial port
    while ser.in_waiting > 0:
        try:
            line_raw = ser.readline().decode('utf-8', errors='replace').strip()
            
            match = euler_regex.match(line_raw)
            
            if match:
                yaw = float(match.group(1))
                pitch = float(match.group(2))
                roll = float(match.group(3))
                
                # Apply Kalman filter
                measurement = np.array([yaw, pitch, roll])
                kalman_filter.predict()
                filtered = kalman_filter.update(measurement)
                
                # Store raw data
                x_data.append(yaw)
                y_data.append(pitch)
                z_data.append(roll)
                
                # Store filtered data
                x_filtered.append(filtered[0])
                y_filtered.append(filtered[1])
                z_filtered.append(filtered[2])
                
                # Update visual angle displays with filtered values
                update_angle_display(filtered[0], filtered[1], filtered[2])
                
                # Limit history to last 200 points for clarity
                if len(x_data) > 200:
                    x_data[:] = x_data[-200:]
                    y_data[:] = y_data[-200:]
                    z_data[:] = z_data[-200:]
                    x_filtered[:] = x_filtered[-200:]
                    y_filtered[:] = y_filtered[-200:]
                    z_filtered[:] = z_filtered[-200:]
                
                # Update the plotted lines and the current position dot
                if len(x_data) > 0:
                    line.set_data(x_data, y_data)
                    line.set_3d_properties(z_data)
                    filtered_line.set_data(x_filtered, y_filtered)
                    filtered_line.set_3d_properties(z_filtered)
                    dot.set_data(x_filtered[-1:], y_filtered[-1:])
                    dot.set_3d_properties(z_filtered[-1:])
                    
                    # Update plot limits if auto-resize is enabled
                    if len(x_data) > 1 and len(x_data) % 10 == 0:  # Only check every 10 points
                        update_plot_limits()
                    
                    canvas.draw_idle()
            else:
                # Print non-matching lines for debugging
                if line_raw and not line_raw.startswith("Euler:"):
                    print(f"Received: {line_raw}")
        except Exception as e:
            # Handle serial read errors
            print(f"Serial read error: {e}")
            # Try to flush the input buffer if there's an issue
            if ser.in_waiting > 100:  # If buffer is filling up with bad data
                ser.reset_input_buffer()
                print("Reset input buffer due to overflow")
    
    # Schedule the next update
    root.after(10, update_plot)

# Start the update process
root.after(10, update_plot)

# Run the Tkinter main loop
root.mainloop()

# Clean up when the window is closed
ser.close()
print("Serial connection closed.")
