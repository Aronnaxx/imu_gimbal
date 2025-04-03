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
from matplotlib.figure import Figure

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
        self.min_size = 80  # Minimum size to prevent too small gauges
        
        # Bind to variable changes
        self.variable.trace_add('write', self.update_gauge)
        
        # Bind to resize events
        self.bind("<Configure>", self.on_resize)
        
        # Draw initial state
        self.update_gauge()
    
    def on_resize(self, event):
        """Handle resize events by updating the gauge size"""
        # Get the smaller of width or height to maintain aspect ratio
        new_size = min(event.width, event.height)
        
        # Don't go below min_size
        if new_size < self.min_size:
            new_size = self.min_size
            
        if new_size != self.size:
            self.size = new_size
            self.update_gauge()
        
    def update_gauge(self, *args):
        """Update the gauge to reflect the current variable value"""
        self.delete("all")
        
        # Calculate parameters
        value = self.variable.get()
        angle = 225 - (value - self.min_val) / (self.max_val - self.min_val) * 270
        center_x = self.winfo_width() / 2
        center_y = self.winfo_height() / 2
        radius = min(center_x, center_y) * 0.8
        
        if radius <= 0:  # Protection against initial sizing issues
            return
        
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
ghost_x_data, ghost_y_data, ghost_z_data = [], [], []

# Data for second IMU (if connected)
second_imu_data = {"connected": False, "yaw": 0, "pitch": 0, "roll": 0}

# Regular expression to parse serial data of the form: "Euler: 45.0, -30.0, 10.0"
euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")
# New regex for second IMU data (if implemented)
imu2_regex = re.compile(r"IMU2:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

# Current servo positions
servo1_pos = 90
servo2_pos = 90
servo3_pos = 90

# Control mode
control_mode = "RANDOM"  # or "CONTROL" or "FOLLOW"

# Movement speed
movement_speed = 2.0

# Movement state
is_movement_active = False

# Auto-resize plot flag
auto_resize = True
plot_range = 180  # Initial plot range

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
main_frame.columnconfigure(0, weight=1)  # Plot area - equal weight
main_frame.columnconfigure(1, weight=1)  # Control panel - equal weight
main_frame.rowconfigure(0, weight=1)

# Create matplotlib figure with dark theme
plt.style.use('dark_background')
fig = plt.figure(figsize=(6, 6), facecolor=DARK_BG)  # Reduced figure size
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor(DARKER_BG)
line, = ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
dot = ax.plot([], [], [], marker='o', label='Current Orientation', color=ACCENT_COLOR, markersize=8)[0]
ghost_line, = ax.plot([], [], [], lw=1, linestyle='--', label='Ghost Input', color='#aaaaaa')

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

# Embed matplotlib figure in Tkinter with proper constraints
plot_frame = ttk.Frame(main_frame)
plot_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=(0, 10))
plot_frame.rowconfigure(0, weight=1)
plot_frame.columnconfigure(0, weight=1)

# Use bbox_inches='tight' for better fitting
fig.tight_layout(pad=2.0)
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.draw()
canvas_widget = canvas.get_tk_widget()
canvas_widget.grid(row=0, column=0, sticky=(tk.N, tk.W, tk.E, tk.S))

# Explicitly set the width and frame minimum size to prevent overflow
def enforce_plot_size(event=None):
    if event and event.widget == root:
        # Calculate desired plot width (fixed maximum percentage of window width)
        total_width = root.winfo_width()
        plot_width = min(int(total_width * 0.5), 600)  # 50% of window width, max 600px
        control_width = max(300, total_width - plot_width)  # Ensure controls have minimum width
        
        # Update column weights dynamically
        main_frame.columnconfigure(0, minsize=plot_width)
        main_frame.columnconfigure(1, minsize=control_width)
        
        # Force redraw
        canvas.draw_idle()

# Bind to root window size changes for plot resizing
root.bind("<Configure>", enforce_plot_size)

# Create control panel with modern styling and scrollbar
control_panel_container = ttk.Frame(main_frame)
control_panel_container.grid(column=1, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
control_panel_container.rowconfigure(0, weight=1)
control_panel_container.columnconfigure(0, weight=1)

# Add scrollbar
control_scrollbar = ttk.Scrollbar(control_panel_container, orient=tk.VERTICAL)
control_scrollbar.grid(row=0, column=1, sticky=(tk.N, tk.S))

# Create a canvas for scrolling
control_canvas = tk.Canvas(control_panel_container, bg=DARK_BG, highlightthickness=0,
                         yscrollcommand=control_scrollbar.set)
control_canvas.grid(row=0, column=0, sticky=(tk.N, tk.W, tk.E, tk.S))
control_scrollbar.config(command=control_canvas.yview)

# Create a frame inside the canvas for the actual controls
control_panel = ttk.Frame(control_canvas, padding="15")
control_panel_window = control_canvas.create_window((0, 0), window=control_panel, anchor=tk.NW, tags="control_panel")

# Configure the canvas to resize the inner frame when the window changes size
def on_canvas_configure(event):
    control_canvas.itemconfig("control_panel", width=event.width-5)  # -5 for padding
    control_canvas.config(scrollregion=control_canvas.bbox("all"))

control_canvas.bind("<Configure>", on_canvas_configure)

# Update the scrollregion when the control panel changes size
def update_scrollregion(event):
    control_canvas.configure(scrollregion=control_canvas.bbox("all"))
    
control_panel.bind("<Configure>", update_scrollregion)

# Update the main window's minimum size to prevent controls from being cut off
def set_window_min_size():
    # Ensure minimum width for comfortable viewing
    required_width = 1000
    required_height = 700
    current_width = root.winfo_width()
    current_height = root.winfo_height()
    
    if current_width < required_width or current_height < required_height:
        new_width = max(current_width, required_width)
        new_height = max(current_height, required_height)
        root.geometry(f"{new_width}x{new_height}")

# Call after window is fully loaded
root.after(100, set_window_min_size)

# Update the window title with resize instructions
root.title("IMU Gimbal Control - 3D Orientation Tracker (Resize window for better view)")

# Add a help message at the top of the control panel
help_frame = ttk.Frame(control_panel)
help_frame.pack(fill=tk.X, pady=(0, 10))
help_text = "If controls are cut off, resize window or scroll down"
help_label = ttk.Label(help_frame, text=help_text, foreground=ACCENT_COLOR)
help_label.pack(fill=tk.X)

# Mode selection
mode_frame = ttk.LabelFrame(control_panel, text="Mode Selection", padding="10")
mode_frame.pack(fill=tk.X, pady=10)

# Create a frame for the mode options
mode_options_frame = ttk.Frame(mode_frame)
mode_options_frame.pack(fill=tk.X, pady=5)

mode_var = tk.StringVar(value="RANDOM")

# Mode radio buttons with better styling
mode_random = ttk.Radiobutton(mode_options_frame, text="Random", variable=mode_var, value="RANDOM")
mode_random.pack(side=tk.LEFT, padx=(0, 10))
mode_control = ttk.Radiobutton(mode_options_frame, text="Control", variable=mode_var, value="CONTROL")
mode_control.pack(side=tk.LEFT, padx=(0, 10))
mode_follow = ttk.Radiobutton(mode_options_frame, text="Follow IMU", variable=mode_var, value="FOLLOW")
mode_follow.pack(side=tk.LEFT)

# Start/Stop frame with modern buttons
start_stop_frame = ttk.Frame(mode_frame)
start_stop_frame.pack(fill=tk.X, pady=10)

def start_movement():
    global is_movement_active
    is_movement_active = True
    ser.write(b"START\n")
    ser.flush()
    print("Movement started")
    movement_status_var.set("Running")
    status_indicator.config(background=SUCCESS_COLOR)
    
    # Apply current settings when starting
    send_control_command()

def stop_movement():
    global is_movement_active
    is_movement_active = False
    ser.write(b"STOP\n")
    ser.flush()
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
gauges_frame.columnconfigure(0, weight=1)
gauges_frame.columnconfigure(1, weight=1)
gauges_frame.columnconfigure(2, weight=1)

# Create and place the gauges in a row
gauge1 = CircularGauge(gauges_frame, servo1_var, size=110, label="Yaw")
gauge1.grid(row=0, column=0, padx=5, sticky=(tk.N, tk.S, tk.E, tk.W))

gauge2 = CircularGauge(gauges_frame, servo2_var, size=110, label="Pitch")
gauge2.grid(row=0, column=1, padx=5, sticky=(tk.N, tk.S, tk.E, tk.W))

gauge3 = CircularGauge(gauges_frame, servo3_var, size=110, label="Roll")
gauge3.grid(row=0, column=2, padx=5, sticky=(tk.N, tk.S, tk.E, tk.W))

# Add row and column configuration to make gauges resize
gauges_frame.rowconfigure(0, weight=1)

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

# Function to send control command to Arduino
def send_control_command():
    global servo1_pos, servo2_pos, servo3_pos, control_mode, movement_speed
    
    current_mode = mode_var.get()
    
    if current_mode != control_mode:
        control_mode = current_mode
        print(f"Switched to {control_mode} mode")
        ser.write(f"{control_mode}\n".encode())
        ser.flush()
        
        # When switching to random mode, start motion immediately if active
        if control_mode == "RANDOM" and is_movement_active:
            ser.write(b"START\n")
            ser.flush()
            print("Automatically starting random movement")
    
    if control_mode == "CONTROL":
        servo1_pos = servo1_var.get()
        servo2_pos = servo2_var.get()
        servo3_pos = servo3_var.get()
        command = f"CONTROL,{servo1_pos},{servo2_pos},{servo3_pos}\n"
        ser.write(command.encode())
        ser.flush()
        print(f"Sent control command: {servo1_pos}, {servo2_pos}, {servo3_pos}")
    elif control_mode == "FOLLOW" and second_imu_data["connected"]:
        # Use the second IMU data to control servos
        yaw = int(90 + second_imu_data["yaw"])
        pitch = int(90 + second_imu_data["pitch"])
        roll = int(90 + second_imu_data["roll"])
        
        # Constrain to valid servo range (0-180)
        yaw = max(0, min(180, yaw))
        pitch = max(0, min(180, pitch))
        roll = max(0, min(180, roll))
        
        # Update the UI
        servo1_var.set(yaw)
        servo2_var.set(pitch)
        servo3_var.set(roll)
        
        # Send command
        command = f"CONTROL,{yaw},{pitch},{roll}\n"
        ser.write(command.encode())
        ser.flush()
        print(f"Sent follow command: {yaw}, {pitch}, {roll}")
    
    # Also update speed if changed
    new_speed = speed_var.get()
    if abs(new_speed - movement_speed) > 0.1:
        movement_speed = new_speed
        speed_command = f"SPEED,{new_speed}\n"
        ser.write(speed_command.encode())
        ser.flush()
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

# Home servos button
def home_servos():
    ser.write(b"HOME\n")
    ser.flush()
    print("Homing servos to middle position")
    # Update the UI sliders to match
    servo1_var.set(90)
    servo2_var.set(90)
    servo3_var.set(90)

# Create frame for IMU control buttons
imu_buttons_frame = ttk.Frame(imu_frame)
imu_buttons_frame.pack(fill=tk.X, pady=5)
imu_buttons_frame.columnconfigure(0, weight=1)
imu_buttons_frame.columnconfigure(1, weight=1)

# Zero IMU button with better styling
def zero_imu():
    ser.write(b"ZERO\n")
    ser.flush()
    print("Zeroing IMU")

# Add both buttons side by side
ttk.Button(imu_buttons_frame, text="Zero IMU", command=zero_imu).grid(row=0, column=0, padx=(0, 5), sticky=(tk.W, tk.E))
ttk.Button(imu_buttons_frame, text="Home Servos", command=home_servos).grid(row=0, column=1, padx=(5, 0), sticky=(tk.W, tk.E))

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
    
    # Read all available data from the serial port
    while ser.in_waiting > 0:
        try:
            line_raw = ser.readline().decode('utf-8', errors='replace').strip()
            
            # Check for main IMU data
            match = euler_regex.match(line_raw)
            if match:
                yaw = float(match.group(1))
                pitch = float(match.group(2))
                roll = float(match.group(3))
                
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
            # Check for second IMU data (for ghost trace and follow mode)
            elif imu2_regex.match(line_raw):
                imu2_match = imu2_regex.match(line_raw)
                # Update second IMU data dictionary
                second_imu_data["yaw"] = float(imu2_match.group(1))
                second_imu_data["pitch"] = float(imu2_match.group(2))
                second_imu_data["roll"] = float(imu2_match.group(3))
                
                # Update ghost trace lists
                ghost_x_data.append(second_imu_data["yaw"])
                ghost_y_data.append(second_imu_data["pitch"])
                ghost_z_data.append(second_imu_data["roll"])
                if len(ghost_x_data) > 200:
                    ghost_x_data[:] = ghost_x_data[-200:]
                    ghost_y_data[:] = ghost_y_data[-200:]
                    ghost_z_data[:] = ghost_z_data[-200:]
                
                # Update ghost_line with new ghost trace data
                ghost_line.set_data(ghost_x_data, ghost_y_data)
                ghost_line.set_3d_properties(ghost_z_data)
                
                # If in follow mode, update the servos
                if mode_var.get() == "FOLLOW" and is_movement_active:
                    send_control_command()
                
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
    
    # Schedule the next update with a shorter interval for more frequent updates
    root.after(10, update_plot)

# Set up a periodic task to send control commands when in control mode
def periodic_command_update():
    if is_movement_active and mode_var.get() == "CONTROL":
        send_control_command()
    root.after(100, periodic_command_update)  # Update every 100ms

# Apply initial styling
update_speed_viz()  # Initialize speed visualization

# Start the periodic update
root.after(100, periodic_command_update)

# Start the update process with a shorter interval
root.after(10, update_plot)

# Send initial command to set up Arduino
send_control_command()

# Add a new section for second IMU status
second_imu_frame = ttk.LabelFrame(control_panel, text="Second IMU Status", padding="10")
second_imu_frame.pack(fill=tk.X, pady=10)

# Status label
second_imu_status_var = tk.StringVar(value="Not Connected")
second_imu_status_label = ttk.Label(second_imu_frame, textvariable=second_imu_status_var)
second_imu_status_label.pack(fill=tk.X, pady=5)

# Button to search for second IMU
def search_second_imu():
    # This is a placeholder - implement actual second IMU connection logic
    # For testing, we'll just simulate a connected IMU
    second_imu_data["connected"] = True
    second_imu_status_var.set("Connected (Simulated)")
    second_imu_status_label.config(foreground=SUCCESS_COLOR)

ttk.Button(second_imu_frame, text="Search for Second IMU", command=search_second_imu).pack(fill=tk.X, pady=5)

# Add bindings for mouse wheel scrolling on different platforms
def _on_mousewheel(event):
    # This will work on most Linux and Windows
    control_canvas.yview_scroll(int(-1*(event.delta/120)), "units")

def _on_mousewheel_darwin(event):
    # For MacOS
    control_canvas.yview_scroll(int(-1*event.delta), "units")

# Bind mousewheel events based on platform
if root.tk.call('tk', 'windowingsystem') == 'aqua':  # macOS
    control_canvas.bind_all("<MouseWheel>", _on_mousewheel_darwin)
else:
    control_canvas.bind_all("<MouseWheel>", _on_mousewheel)
    # For Linux with Xorg
    control_canvas.bind_all("<Button-4>", lambda e: control_canvas.yview_scroll(-1, "units"))
    control_canvas.bind_all("<Button-5>", lambda e: control_canvas.yview_scroll(1, "units"))

# Add a compact mode toggle at the top of the UI
compact_mode_var = tk.BooleanVar(value=False)

def toggle_compact_mode():
    is_compact = compact_mode_var.get()
    
    # Adjust layout based on compact mode
    if is_compact:
        # Make the gauges smaller and rearrange them
        for gauge in [gauge1, gauge2, gauge3]:
            gauge.min_size = 60
            gauge.update_gauge()
        
        # Collapse some sections if window is too small
        if root.winfo_width() < 1100:
            # Hide some optional controls
            second_imu_frame.pack_forget()
            plot_frame_controls.pack_forget()
            
            # Show them again when expanded
            compact_sections_btn.config(text="Show More Controls ▼")
            compact_sections_btn.pack(fill=tk.X, pady=5)
        
        # Update the toggle button text
        compact_mode_btn.config(text="Expand UI")
    else:
        # Reset gauge sizes to normal
        for gauge in [gauge1, gauge2, gauge3]:
            gauge.min_size = 80
            gauge.update_gauge()
        
        # Restore any hidden sections
        second_imu_frame.pack(fill=tk.X, pady=10)
        plot_frame_controls.pack(fill=tk.X, pady=10)
        
        # Hide the "more controls" button
        compact_sections_btn.pack_forget()
        
        # Update the toggle button text
        compact_mode_btn.config(text="Compact UI")
    
    # Update the UI
    control_panel.update_idletasks()
    control_canvas.configure(scrollregion=control_canvas.bbox("all"))

def toggle_compact_sections():
    # Toggle visibility of secondary control sections
    btn_text = compact_sections_btn.cget("text")
    
    if "Show" in btn_text:
        # Show hidden sections
        second_imu_frame.pack(fill=tk.X, pady=10)
        plot_frame_controls.pack(fill=tk.X, pady=10)
        compact_sections_btn.config(text="Hide Extra Controls ▲")
    else:
        # Hide sections
        second_imu_frame.pack_forget()
        plot_frame_controls.pack_forget()
        compact_sections_btn.config(text="Show More Controls ▼")
    
    # Update the canvas scrollregion
    control_panel.update_idletasks()
    control_canvas.configure(scrollregion=control_canvas.bbox("all"))

# Add compact mode toggle button at the top of the control panel
view_controls_frame = ttk.Frame(control_panel)
view_controls_frame.pack(fill=tk.X, pady=(0, 10))

compact_mode_btn = ttk.Button(view_controls_frame, text="Compact UI", command=toggle_compact_mode)
compact_mode_btn.pack(side=tk.LEFT, fill=tk.X, expand=True)

# Create a more/less button that's initially hidden
compact_sections_btn = ttk.Button(control_panel, text="Show More Controls ▼", command=toggle_compact_sections)
# It will be shown when needed by the toggle_compact_mode function

# Add a resize detection event that automatically switches to compact mode for small windows
def check_window_size(event=None):
    if event and event.widget == root:
        if root.winfo_width() < 1100 and not compact_mode_var.get():
            compact_mode_var.set(True)
            toggle_compact_mode()
        elif root.winfo_width() >= 1100 and compact_mode_var.get():
            compact_mode_var.set(False)
            toggle_compact_mode()

# Bind to root window size changes
root.bind("<Configure>", check_window_size)

# Run the Tkinter main loop
root.mainloop()

# Clean up when the window is closed
ser.close()
print("Serial connection closed.")
