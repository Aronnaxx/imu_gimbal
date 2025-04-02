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

# Set your Arduino's COM port (e.g., 'COM3' on Windows or '/dev/ttyUSB0' on Linux)
PORT = '/dev/tty.usbmodem101'
BAUD = 115200

# Initialize serial connection
ser = serial.Serial(PORT, BAUD, timeout=1)
print(f"Connected to {PORT} at {BAUD} baud")

# Lists to hold Euler angle data: x (yaw), y (pitch), z (roll)
x_data, y_data, z_data = [], [], []

# Regular expression to parse serial data of the form: "Euler: 45.0, -30.0, 10.0"
euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

# Current servo positions
servo1_pos = 90
servo2_pos = 90
servo3_pos = 90

# Control mode
control_mode = "RANDOM"  # or "CONTROL"

# Movement speed
movement_speed = 2.0

# Movement state
is_movement_active = False

# Auto-resize plot flag
auto_resize = True
plot_range = 180  # Initial plot range

# Create main Tkinter window
root = tk.Tk()
root.title("IMU Gimbal Control")
root.geometry("1200x800")
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Create frame to hold everything
main_frame = ttk.Frame(root)
main_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=10, pady=10)
main_frame.columnconfigure(0, weight=3)  # Plot area
main_frame.columnconfigure(1, weight=1)  # Control panel
main_frame.rowconfigure(0, weight=1)

# Create matplotlib figure
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
line, = ax.plot([], [], [], lw=2, label='Orientation Path')
dot = ax.plot([], [], [], 'ro', label='Current Orientation')[0]

# Set initial axis limits
ax.set_xlim(-plot_range, plot_range)
ax.set_ylim(-plot_range, plot_range)
ax.set_zlim(-plot_range, plot_range)
ax.set_title("3D Orientation Trace (Yaw, Pitch, Roll)")
ax.set_xlabel("Yaw")
ax.set_ylabel("Pitch")
ax.set_zlabel("Roll")
ax.legend()

# Embed matplotlib figure in Tkinter
plot_frame = ttk.Frame(main_frame)
plot_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
canvas = FigureCanvasTkAgg(fig, master=plot_frame)
canvas.draw()
canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

# Create control panel
control_panel = ttk.Frame(main_frame, padding="10")
control_panel.grid(column=1, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))

# Mode selection
mode_frame = ttk.LabelFrame(control_panel, text="Mode Selection", padding="10")
mode_frame.pack(fill=tk.X, pady=10)

mode_var = tk.StringVar(value="RANDOM")
ttk.Radiobutton(mode_frame, text="Random", variable=mode_var, value="RANDOM").pack(anchor=tk.W)
ttk.Radiobutton(mode_frame, text="Control", variable=mode_var, value="CONTROL").pack(anchor=tk.W)

# Start/Stop frame
start_stop_frame = ttk.Frame(mode_frame)
start_stop_frame.pack(fill=tk.X, pady=5)

def start_movement():
    global is_movement_active
    is_movement_active = True
    ser.write(b"START\n")
    ser.flush()
    print("Movement started")
    movement_status_var.set("Status: Running")

def stop_movement():
    global is_movement_active
    is_movement_active = False
    ser.write(b"STOP\n")
    ser.flush()
    print("Movement stopped")
    movement_status_var.set("Status: Stopped")

ttk.Button(start_stop_frame, text="Start", command=start_movement).pack(side=tk.LEFT, padx=5)
ttk.Button(start_stop_frame, text="Stop", command=stop_movement).pack(side=tk.LEFT, padx=5)

# Movement status
movement_status_var = tk.StringVar(value="Status: Stopped")
ttk.Label(mode_frame, textvariable=movement_status_var).pack(anchor=tk.W, pady=5)

# Servo control frame
servo_frame = ttk.LabelFrame(control_panel, text="Servo Control", padding="10")
servo_frame.pack(fill=tk.X, pady=10)

# Servo sliders and buttons
servo1_var = tk.IntVar(value=servo1_pos)
servo2_var = tk.IntVar(value=servo2_pos)
servo3_var = tk.IntVar(value=servo3_pos)

# Helper functions for servo control buttons
def increment_servo(servo_var, amount):
    current = servo_var.get()
    servo_var.set(min(180, current + amount))

def decrement_servo(servo_var, amount):
    current = servo_var.get()
    servo_var.set(max(0, current - amount))

def center_servo(servo_var):
    servo_var.set(90)

# Servo 1 controls
ttk.Label(servo_frame, text="Servo 1 (Yaw):").grid(column=0, row=0, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo1_var, orient=tk.HORIZONTAL).grid(column=1, row=0, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo1_var).grid(column=2, row=0)

servo1_btn_frame = ttk.Frame(servo_frame)
servo1_btn_frame.grid(column=3, row=0, sticky=(tk.W, tk.E))
ttk.Button(servo1_btn_frame, text="-10", width=3, 
           command=lambda: decrement_servo(servo1_var, 10)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo1_btn_frame, text="-1", width=2,
           command=lambda: decrement_servo(servo1_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo1_btn_frame, text="C", width=2,
           command=lambda: center_servo(servo1_var)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo1_btn_frame, text="+1", width=2,
           command=lambda: increment_servo(servo1_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo1_btn_frame, text="+10", width=3,
           command=lambda: increment_servo(servo1_var, 10)).pack(side=tk.LEFT, padx=1)

# Servo 2 controls
ttk.Label(servo_frame, text="Servo 2 (Pitch):").grid(column=0, row=1, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo2_var, orient=tk.HORIZONTAL).grid(column=1, row=1, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo2_var).grid(column=2, row=1)

servo2_btn_frame = ttk.Frame(servo_frame)
servo2_btn_frame.grid(column=3, row=1, sticky=(tk.W, tk.E))
ttk.Button(servo2_btn_frame, text="-10", width=3, 
           command=lambda: decrement_servo(servo2_var, 10)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo2_btn_frame, text="-1", width=2,
           command=lambda: decrement_servo(servo2_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo2_btn_frame, text="C", width=2,
           command=lambda: center_servo(servo2_var)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo2_btn_frame, text="+1", width=2,
           command=lambda: increment_servo(servo2_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo2_btn_frame, text="+10", width=3,
           command=lambda: increment_servo(servo2_var, 10)).pack(side=tk.LEFT, padx=1)

# Servo 3 controls
ttk.Label(servo_frame, text="Servo 3 (Roll):").grid(column=0, row=2, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo3_var, orient=tk.HORIZONTAL).grid(column=1, row=2, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo3_var).grid(column=2, row=2)

servo3_btn_frame = ttk.Frame(servo_frame)
servo3_btn_frame.grid(column=3, row=2, sticky=(tk.W, tk.E))
ttk.Button(servo3_btn_frame, text="-10", width=3, 
           command=lambda: decrement_servo(servo3_var, 10)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo3_btn_frame, text="-1", width=2,
           command=lambda: decrement_servo(servo3_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo3_btn_frame, text="C", width=2,
           command=lambda: center_servo(servo3_var)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo3_btn_frame, text="+1", width=2,
           command=lambda: increment_servo(servo3_var, 1)).pack(side=tk.LEFT, padx=1)
ttk.Button(servo3_btn_frame, text="+10", width=3,
           command=lambda: increment_servo(servo3_var, 10)).pack(side=tk.LEFT, padx=1)

# Speed control
speed_frame = ttk.LabelFrame(control_panel, text="Movement Speed", padding="10")
speed_frame.pack(fill=tk.X, pady=10)

speed_var = tk.DoubleVar(value=movement_speed)
ttk.Label(speed_frame, text="Speed:").grid(column=0, row=0, sticky=tk.W)
ttk.Scale(speed_frame, from_=0.5, to=10, variable=speed_var, orient=tk.HORIZONTAL).grid(column=1, row=0, sticky=(tk.W, tk.E))
ttk.Label(speed_frame, textvariable=speed_var).grid(column=2, row=0)

# Plot control frame
plot_frame_controls = ttk.LabelFrame(control_panel, text="Plot Controls", padding="10")
plot_frame_controls.pack(fill=tk.X, pady=10)

# Auto-resize toggle
auto_resize_var = tk.BooleanVar(value=auto_resize)
ttk.Checkbutton(plot_frame_controls, text="Auto-resize plot", variable=auto_resize_var).pack(anchor=tk.W)

# Reset plot button
def reset_plot():
    global x_data, y_data, z_data
    x_data.clear()
    y_data.clear()
    z_data.clear()
    update_plot_limits()
    canvas.draw_idle()

ttk.Button(plot_frame_controls, text="Reset Plot", command=reset_plot).pack(anchor=tk.W, pady=5)

# IMU control frame
imu_frame = ttk.LabelFrame(control_panel, text="IMU Controls", padding="10")
imu_frame.pack(fill=tk.X, pady=10)

# Zero IMU button
def zero_imu():
    ser.write(b"ZERO\n")
    ser.flush()
    print("Zeroing IMU")

ttk.Button(imu_frame, text="Zero IMU", command=zero_imu).pack(anchor=tk.W)

# Status frame
status_frame = ttk.LabelFrame(control_panel, text="Status", padding="10")
status_frame.pack(fill=tk.X, pady=10)

current_angles_text = tk.StringVar(value="Yaw: 0.0, Pitch: 0.0, Roll: 0.0")
ttk.Label(status_frame, textvariable=current_angles_text).pack(anchor=tk.W)

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

# Function to send control command to Arduino
def send_control_command():
    global servo1_pos, servo2_pos, servo3_pos, control_mode, movement_speed
    
    current_mode = mode_var.get()
    
    if current_mode != control_mode:
        control_mode = current_mode
        print(f"Switched to {control_mode} mode")
        ser.write(f"{control_mode}\n".encode())
        ser.flush()
    
    if control_mode == "CONTROL":
        servo1_pos = servo1_var.get()
        servo2_pos = servo2_var.get()
        servo3_pos = servo3_var.get()
        command = f"CONTROL,{servo1_pos},{servo2_pos},{servo3_pos}\n"
        ser.write(command.encode())
        ser.flush()
        print(f"Sent control command: {servo1_pos}, {servo2_pos}, {servo3_pos}")
    
    # Also update speed if changed
    new_speed = speed_var.get()
    if abs(new_speed - movement_speed) > 0.1:
        movement_speed = new_speed
        speed_command = f"SPEED,{new_speed}\n"
        ser.write(speed_command.encode())
        ser.flush()
        print(f"Updated speed to {new_speed}")

# Add control button
ttk.Button(control_panel, text="Apply Settings", command=send_control_command).pack(pady=10)

# Function to update the plot
def update_plot():
    global x_data, y_data, z_data
    
    # Read a line from the serial port
    if ser.in_waiting > 0:
        line_raw = ser.readline().decode('utf-8').strip()
        match = euler_regex.match(line_raw)
        
        if match:
            yaw = float(match.group(1))
            pitch = float(match.group(2))
            roll = float(match.group(3))
            
            x_data.append(yaw)
            y_data.append(pitch)
            z_data.append(roll)
            
            # Update status display
            current_angles_text.set(f"Yaw: {yaw:.1f}, Pitch: {pitch:.1f}, Roll: {roll:.1f}")
            
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
        else:
            # Print non-matching lines for debugging
            if line_raw and not line_raw.startswith("Euler:"):
                print(f"Received: {line_raw}")
    
    # Schedule the next update
    root.after(50, update_plot)

# Start the update process
root.after(100, update_plot)

# Send initial command to set up Arduino
send_control_command()

# Run the Tkinter main loop
root.mainloop()

# Clean up when the window is closed
ser.close()
print("Serial connection closed.")
