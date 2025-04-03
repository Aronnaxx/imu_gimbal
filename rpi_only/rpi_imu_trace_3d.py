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
# Import Raspberry Pi specific libraries
import board
import busio
import adafruit_bno055
import adafruit_pca9685
from adafruit_servokit import ServoKit
import random

# Custom theme and style constants
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"
SLIDER_COLOR = "#3498db"
SUCCESS_COLOR = "#2ecc71"
DANGER_COLOR = "#e74c3c"

# Set up I2C and devices
try:
    # Initialize I2C bus
    i2c = board.I2C()    # Initialize BNO055 IMU sensor
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    # Initialize PCA9685 servo controller with ServoKit
    pca = ServoKit(channels=16)
    print("I2C devices initialized successfully")
except Exception as e:
    print(f"Error initializing I2C devices: {e}")
    raise

# Define servo channels
YAW_SERVO = 0    # Channel for yaw servo
PITCH_SERVO = 1  # Channel for pitch servo
ROLL_SERVO = 2   # Channel for roll servo
TURNTABLE_SERVO = 3  # Channel for turntable servo

# Set initial servo parameters (adjust as needed for your hardware)
SERVO_MIN_PULSE = 500
SERVO_MAX_PULSE = 2500
for channel in [YAW_SERVO, PITCH_SERVO, ROLL_SERVO, TURNTABLE_SERVO]:
    pca.servo[channel].set_pulse_width_range(SERVO_MIN_PULSE, SERVO_MAX_PULSE)

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

class CircularGauge(tk.Canvas):
    """A circular gauge widget for visualizing servo positions"""
    def __init__(self, parent, variable, size=100, label="", min_val=0, max_val=180, **kwargs):
        super().__init__(parent, width=size, height=size, bg=DARK_BG, 
                         highlightthickness=0, **kwargs)
        self.size = size
        self.variable = variable
        self.min_val = min_val
        self.max_val = max_val
        self.label = label
        
        # Bind to variable changes
        self.variable.trace_add('write', self.update_gauge)
        
        # Draw initial state
        self.update_gauge()
        
    def update_gauge(self, *args):
        """Update the gauge to reflect the current variable value"""
        self.delete("all")
        
        # Calculate parameters
        value = self.variable.get()
        angle = 225 - (value - self.min_val) / (self.max_val - self.min_val) * 270
        center_x = self.size / 2
        center_y = self.size / 2
        radius = self.size * 0.4
        
        # Draw background circle
        self.create_oval(center_x - radius - 5, center_y - radius - 5,
                         center_x + radius + 5, center_y + radius + 5,
                         fill=DARKER_BG, width=0)
        
        # Draw arc (track)
        self.create_arc(center_x - radius, center_y - radius,
                        center_x + radius, center_y + radius,
                        start=225, extent=270, style="arc",
                        outline="#555555", width=4)
        
        # Draw colored arc (value)
        self.create_arc(center_x - radius, center_y - radius,
                        center_x + radius, center_y + radius,
                        start=225, extent=-(225-angle), style="arc",
                        outline=HIGHLIGHT, width=5)
        
        # Draw center point
        self.create_oval(center_x - 3, center_y - 3, center_x + 3, center_y + 3,
                         fill=HIGHLIGHT, outline="")
        
        # Draw needle
        needle_length = radius * 0.8
        needle_x = center_x + needle_length * math.cos(math.radians(angle))
        needle_y = center_y + needle_length * math.sin(math.radians(angle))
        self.create_line(center_x, center_y, needle_x, needle_y,
                         fill=HIGHLIGHT, width=2)
        
        # Draw value text
        self.create_text(center_x, center_y + radius + 15,
                         text=f"{int(value)}°", fill=TEXT_COLOR, font=("Arial", 10))
        
        # Draw label text
        self.create_text(center_x, center_y - radius - 15,
                         text=self.label, fill=TEXT_COLOR, font=("Arial", 9))

class ModernSlider(ttk.Frame):
    """A more attractive slider with value display and buttons"""
    def __init__(self, parent, variable, min_val=0, max_val=180, label="", **kwargs):
        super().__init__(parent, **kwargs)
        self.variable = variable
        self.min_val = min_val
        self.max_val = max_val
        
        # Configure grid
        self.columnconfigure(1, weight=1)
        
        # Label
        ttk.Label(self, text=label).grid(column=0, row=0, sticky=tk.W, padx=(0, 10))
        
        # Button frame
        btn_frame = ttk.Frame(self)
        btn_frame.grid(column=2, row=0, sticky=tk.E)
        
        # Decrement buttons
        ttk.Button(btn_frame, text="-10", width=3, style='TButton',
                   command=lambda: self.change_value(-10)).pack(side=tk.LEFT, padx=1)
        ttk.Button(btn_frame, text="-1", width=2, style='TButton',
                   command=lambda: self.change_value(-1)).pack(side=tk.LEFT, padx=1)
        
        # Center button
        ttk.Button(btn_frame, text="C", width=2, style='TButton',
                   command=self.center_value).pack(side=tk.LEFT, padx=1)
        
        # Increment buttons
        ttk.Button(btn_frame, text="+1", width=2, style='TButton',
                   command=lambda: self.change_value(1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(btn_frame, text="+10", width=3, style='TButton',
                   command=lambda: self.change_value(10)).pack(side=tk.LEFT, padx=1)
        
        # Slider
        self.slider = ttk.Scale(self, from_=min_val, to=max_val, variable=variable, 
                               orient=tk.HORIZONTAL)
        self.slider.grid(column=0, row=1, columnspan=3, sticky=(tk.W, tk.E), pady=(5, 0))
    
    def change_value(self, amount):
        current = self.variable.get()
        new_value = max(self.min_val, min(self.max_val, current + amount))
        self.variable.set(new_value)
    
    def center_value(self):
        center = (self.max_val - self.min_val) / 2 + self.min_val
        self.variable.set(center)

# Current servo positions (degrees, 0-180)
servo1_pos = 90  # Yaw
servo2_pos = 90  # Pitch
servo3_pos = 90  # Roll
servo4_pos = 90  # Turntable

# Control mode
control_mode = "RANDOM"  # or "CONTROL"

# Movement speed
movement_speed = 2.0

# Movement state
is_movement_active = False

# Auto-resize plot flag
auto_resize = True
plot_range = 180  # Initial plot range

# Lists to hold Euler angle data: x (yaw), y (pitch), z (roll)
x_data, y_data, z_data = [], [], []

# Regular expression to parse serial data of the form: "Euler: 45.0, -30.0, 10.0"
euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

# Create main Tkinter window
root = tk.Tk()
root.title("IMU Gimbal Control - 3D Orientation Tracker")
root.geometry("1300x800")
root.configure(bg=DARK_BG)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Add an icon if you have one (uncomment if needed)
# icon = tk.PhotoImage(file="icon.png")
# root.iconphoto(True, icon)

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

# Mode selection
mode_frame = ttk.LabelFrame(control_panel, text="Mode Selection", padding="10")
mode_frame.pack(fill=tk.X, pady=10)

# Create a frame for the mode options
mode_options_frame = ttk.Frame(mode_frame)
mode_options_frame.pack(fill=tk.X, pady=5)

mode_var = tk.StringVar(value="RANDOM")

# Mode radio buttons with better styling
mode_random = ttk.Radiobutton(mode_options_frame, text="Random", variable=mode_var, value="RANDOM")
mode_random.pack(side=tk.LEFT, padx=(0, 20))
mode_control = ttk.Radiobutton(mode_options_frame, text="Control", variable=mode_var, value="CONTROL")
mode_control.pack(side=tk.LEFT)

# Start/Stop frame with modern buttons
start_stop_frame = ttk.Frame(mode_frame)
start_stop_frame.pack(fill=tk.X, pady=10)

def start_movement():
    global is_movement_active
    is_movement_active = True
    print("Movement started")
    movement_status_var.set("Running")
    status_indicator.config(background=SUCCESS_COLOR)
    
    # Apply current settings when starting
    send_control_command()

def stop_movement():
    global is_movement_active
    is_movement_active = False
    print("Movement stopped")
    movement_status_var.set("Stopped")
    status_indicator.config(background=DANGER_COLOR)

# Create more attractive buttons
start_button = ttk.Button(start_stop_frame, text="Start", command=start_movement, style='Start.TButton')
start_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

stop_button = ttk.Button(start_stop_frame, text="Stop", command=stop_movement, style='Stop.TButton')
stop_button.pack(side=tk.LEFT, padx=5, expand=True, fill=tk.X)

# Status indicator with visual feedback
status_frame = ttk.Frame(mode_frame)
status_frame.pack(fill=tk.X, pady=5)

movement_status_var = tk.StringVar(value="Stopped")
ttk.Label(status_frame, text="Status: ").pack(side=tk.LEFT)
ttk.Label(status_frame, textvariable=movement_status_var).pack(side=tk.LEFT)

# Visual status indicator
status_indicator = tk.Canvas(status_frame, width=15, height=15, background=DANGER_COLOR, 
                            highlightthickness=0)
status_indicator.pack(side=tk.RIGHT, padx=5)

# Servo control frame with gauges
servo_frame = ttk.LabelFrame(control_panel, text="Servo Control", padding="10")
servo_frame.pack(fill=tk.X, pady=10)

# Servo sliders and buttons
servo1_var = tk.IntVar(value=servo1_pos)
servo2_var = tk.IntVar(value=servo2_pos)
servo3_var = tk.IntVar(value=servo3_pos)
servo4_var = tk.IntVar(value=servo4_pos)

# Add event handlers for sliders
def on_slider_change(event=None):
    # Auto-apply settings when in control mode
    if mode_var.get() == "CONTROL" and is_movement_active:
        root.after(10, send_control_command)

# Servo control with visual gauges
servo_controls_frame = ttk.Frame(servo_frame)
servo_controls_frame.pack(fill=tk.X, pady=5)

# Add a frame for gauges
gauges_frame = ttk.Frame(servo_frame)
gauges_frame.pack(fill=tk.X, pady=5)

# Create and place the gauges in a row
gauge1 = CircularGauge(gauges_frame, servo1_var, size=110, label="Yaw")
gauge1.pack(side=tk.LEFT, padx=5, expand=True)

gauge2 = CircularGauge(gauges_frame, servo2_var, size=110, label="Pitch")
gauge2.pack(side=tk.LEFT, padx=5, expand=True)

gauge3 = CircularGauge(gauges_frame, servo3_var, size=110, label="Roll")
gauge3.pack(side=tk.LEFT, padx=5, expand=True)

gauge4 = CircularGauge(gauges_frame, servo4_var, size=110, label="Turntable")
gauge4.pack(side=tk.LEFT, padx=5, expand=True)

# Create modern sliders with better layout
slider_frame = ttk.Frame(servo_frame)
slider_frame.pack(fill=tk.X, pady=10)

# Create modern sliders
servo1_slider = ModernSlider(slider_frame, servo1_var, label="Servo 1 (Yaw)")
servo1_slider.pack(fill=tk.X, pady=5)
servo1_slider.slider.bind("<B1-Motion>", on_slider_change)
servo1_slider.slider.bind("<ButtonRelease-1>", on_slider_change)

servo2_slider = ModernSlider(slider_frame, servo2_var, label="Servo 2 (Pitch)")
servo2_slider.pack(fill=tk.X, pady=5)
servo2_slider.slider.bind("<B1-Motion>", on_slider_change)
servo2_slider.slider.bind("<ButtonRelease-1>", on_slider_change)

servo3_slider = ModernSlider(slider_frame, servo3_var, label="Servo 3 (Roll)")
servo3_slider.pack(fill=tk.X, pady=5)
servo3_slider.slider.bind("<B1-Motion>", on_slider_change)
servo3_slider.slider.bind("<ButtonRelease-1>", on_slider_change)

servo4_slider = ModernSlider(slider_frame, servo4_var, label="Servo 4 (Turntable)")
servo4_slider.pack(fill=tk.X, pady=5)
servo4_slider.slider.bind("<B1-Motion>", on_slider_change)
servo4_slider.slider.bind("<ButtonRelease-1>", on_slider_change)

# Speed control with better visualization
speed_frame = ttk.LabelFrame(control_panel, text="Movement Speed", padding="10")
speed_frame.pack(fill=tk.X, pady=10)

# Speed control with visual indicator
speed_var = tk.DoubleVar(value=movement_speed)

# Speed control container
speed_control_frame = ttk.Frame(speed_frame)
speed_control_frame.pack(fill=tk.X, pady=5)
speed_control_frame.columnconfigure(1, weight=1)

# Speed label and value display
ttk.Label(speed_control_frame, text="Speed:").grid(column=0, row=0, sticky=tk.W, padx=(0, 10))
ttk.Label(speed_control_frame, textvariable=speed_var).grid(column=2, row=0, padx=(5, 0))

# Modern speed slider
speed_slider = ttk.Scale(speed_control_frame, from_=0.5, to=10, variable=speed_var, orient=tk.HORIZONTAL)
speed_slider.grid(column=1, row=0, sticky=(tk.W, tk.E))

# Speed visualization
speed_viz_canvas = tk.Canvas(speed_frame, height=10, bg=DARKER_BG, highlightthickness=0)
speed_viz_canvas.pack(fill=tk.X, pady=5)

# Function to send control command to PCA9685
def send_control_command():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos, control_mode, movement_speed
    
    current_mode = mode_var.get()
    
    if current_mode != control_mode:
        control_mode = current_mode
        print(f"Switched to {control_mode} mode")
    
    if control_mode == "CONTROL":
        # Get current servo positions from sliders
        servo1_pos = servo1_var.get()
        servo2_pos = servo2_var.get()
        servo3_pos = servo3_var.get()
        servo4_pos = servo4_var.get()
        
        # Update the servos directly
        pca.servo[YAW_SERVO].angle = servo1_pos
        pca.servo[PITCH_SERVO].angle = servo2_pos
        pca.servo[ROLL_SERVO].angle = servo3_pos
        pca.servo[TURNTABLE_SERVO].angle = servo4_pos
        
        print(f"Set servo positions: {servo1_pos}, {servo2_pos}, {servo3_pos}, {servo4_pos}")
    
    # Also update speed if changed
    new_speed = speed_var.get()
    if abs(new_speed - movement_speed) > 0.1:
        movement_speed = new_speed
        print(f"Updated speed to {new_speed}")

# Update speed visualization when value changes
def update_speed_viz(*args):
    speed = speed_var.get()
    # Clear canvas
    speed_viz_canvas.delete("all")
    
    # Calculate width percentage based on speed
    width = speed_viz_canvas.winfo_width()
    if width <= 1:  # Not yet properly sized
        width = 200
    
    # Calculate fill percentage (0.5 to 10 maps to 5% to 100%)
    fill_percent = (speed - 0.5) / 9.5
    fill_width = int(width * fill_percent)
    
    # Choose color based on speed (green to yellow to red)
    if speed < 3:
        color = SUCCESS_COLOR
    elif speed < 7:
        color = ACCENT_COLOR
    else:
        color = DANGER_COLOR
    
    # Draw rectangle
    speed_viz_canvas.create_rectangle(0, 0, fill_width, 10, fill=color, outline="")

# Bind to changes
speed_var.trace_add("write", update_speed_viz)
speed_viz_canvas.bind("<Configure>", lambda e: update_speed_viz())

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
    global x_data, y_data, z_data
    x_data.clear()
    y_data.clear()
    z_data.clear()
    update_plot_limits()
    canvas.draw_idle()

ttk.Button(plot_frame_controls, text="Reset Plot", command=reset_plot).pack(anchor=tk.W, pady=5, fill=tk.X)

# IMU control frame
imu_frame = ttk.LabelFrame(control_panel, text="IMU Controls", padding="10")
imu_frame.pack(fill=tk.X, pady=10)

# Zero IMU by resetting the offset
def zero_imu():
    # Store the current reading as our zero reference
    # This is simulated here - with BNO055 we would usually
    # just trust its internal calibration
    print("Zeroing IMU")
    # No direct API for zeroing, but we can restart the sensor
    # or track offset values in software if needed

# Zero IMU button with better styling
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

# Apply settings button with better styling
apply_frame = ttk.Frame(control_panel)
apply_frame.pack(fill=tk.X, pady=15)

apply_button = ttk.Button(apply_frame, text="Apply Settings", command=send_control_command)
apply_button.pack(fill=tk.X, ipady=5)

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
    
    # Calculate needed range with some padding
    x_min, x_max = min(x_data), max(x_data)
    y_min, y_max = min(y_data), max(y_data)
    z_min, z_max = min(z_data), max(z_data)
    
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
    global x_data, y_data, z_data
    
    try:
        # Read Euler angles directly from the BNO055 sensor
        # Returns yaw, roll, pitch in degrees
        # Note: BNO055 returns in different order than our variables expect
        euler = sensor.euler
        if euler:
            # Map to our expected orientation
            # BNO055 returns: heading (yaw), roll, pitch
            yaw = euler[0]
            pitch = euler[2]  # Pitch is the third value
            roll = euler[1]   # Roll is the second value
            
            x_data.append(yaw)
            y_data.append(pitch)
            z_data.append(roll)
            
            # Update visual angle displays
            update_angle_display(yaw, pitch, roll)
            
            # Limit history to last 200 points for clarity
            if len(x_data) > 200:
                x_data[:] = x_data[-200:]
                y_data[:] = y_data[-200:]
                z_data[:] = z_data[-200:]
            
            # Update the plotted line and the current position dot
            if len(x_data) > 0:
                line.set_data(x_data, y_data)
                line.set_3d_properties(z_data)
                dot.set_data(x_data[-1:], y_data[-1:])
                dot.set_3d_properties(z_data[-1:])
                
                # Update plot limits if auto-resize is enabled
                if len(x_data) > 1 and len(x_data) % 10 == 0:  # Only check every 10 points
                    update_plot_limits()
                
                canvas.draw_idle()
    except Exception as e:
        # Handle sensor read errors
        print(f"Sensor read error: {e}")
    
    # Schedule the next update
    root.after(50, update_plot)  # Update every 50ms (20 Hz)

# Set up a periodic task to send control commands or move randomly depending on mode
def periodic_command_update():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos
    
    if is_movement_active:
        if mode_var.get() == "CONTROL":
            # In control mode, send the current slider positions
            send_control_command()
        elif mode_var.get() == "RANDOM":
            # In random mode, generate smooth random movement
            # Calculate new random target positions if needed
            if not hasattr(periodic_command_update, "targets"):
                # Initialize random targets and timers
                periodic_command_update.targets = [
                    random.randint(30, 150),  # yaw
                    random.randint(30, 150),  # pitch
                    random.randint(30, 150),  # roll
                    random.randint(30, 150)   # turntable
                ]
                periodic_command_update.target_time = time.time() + random.uniform(1.0, 3.0)
            
            # Check if it's time for new targets
            current_time = time.time()
            if current_time >= periodic_command_update.target_time:
                # Set new random targets and timer
                periodic_command_update.targets = [
                    random.randint(30, 150),  # yaw
                    random.randint(30, 150),  # pitch
                    random.randint(30, 150),  # roll
                    random.randint(30, 150)   # turntable
                ]
                # Time to reach the new position based on speed
                duration = random.uniform(2.0, 5.0) / movement_speed
                periodic_command_update.target_time = current_time + duration
            
            # Move servos smoothly toward the targets
            targets = periodic_command_update.targets
            step_size = movement_speed * 0.5  # Adjust step size based on speed
            
            # Update each servo position toward its target
            if abs(servo1_pos - targets[0]) > step_size:
                servo1_pos += step_size if servo1_pos < targets[0] else -step_size
            if abs(servo2_pos - targets[1]) > step_size:
                servo2_pos += step_size if servo2_pos < targets[1] else -step_size
            if abs(servo3_pos - targets[2]) > step_size:
                servo3_pos += step_size if servo3_pos < targets[2] else -step_size
            if abs(servo4_pos - targets[3]) > step_size:
                servo4_pos += step_size if servo4_pos < targets[3] else -step_size
                
            # Update the servos
            pca.servo[YAW_SERVO].angle = servo1_pos
            pca.servo[PITCH_SERVO].angle = servo2_pos
            pca.servo[ROLL_SERVO].angle = servo3_pos
            pca.servo[TURNTABLE_SERVO].angle = servo4_pos
            
            # Update the sliders to reflect current positions
            servo1_var.set(int(servo1_pos))
            servo2_var.set(int(servo2_pos))
            servo3_var.set(int(servo3_pos))
            servo4_var.set(int(servo4_pos))
    
    # Schedule the next update
    root.after(50, periodic_command_update)  # Update every 50ms

# Apply initial styling
update_speed_viz()  # Initialize speed visualization

# Start the periodic update
root.after(50, periodic_command_update)

# Start the update process for the plot
root.after(50, update_plot)

# Run the Tkinter main loop
root.mainloop()

# Clean up when the window is closed
print("Application closed.")
