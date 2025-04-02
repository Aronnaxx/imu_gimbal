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

# Set axis limits
ax.set_xlim(-180, 180)
ax.set_ylim(-180, 180)
ax.set_zlim(-180, 180)
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

# Servo control frame
servo_frame = ttk.LabelFrame(control_panel, text="Servo Control", padding="10")
servo_frame.pack(fill=tk.X, pady=10)

# Servo sliders
servo1_var = tk.IntVar(value=servo1_pos)
servo2_var = tk.IntVar(value=servo2_pos)
servo3_var = tk.IntVar(value=servo3_pos)

ttk.Label(servo_frame, text="Servo 1 (Yaw):").grid(column=0, row=0, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo1_var, orient=tk.HORIZONTAL).grid(column=1, row=0, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo1_var).grid(column=2, row=0)

ttk.Label(servo_frame, text="Servo 2 (Pitch):").grid(column=0, row=1, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo2_var, orient=tk.HORIZONTAL).grid(column=1, row=1, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo2_var).grid(column=2, row=1)

ttk.Label(servo_frame, text="Servo 3 (Roll):").grid(column=0, row=2, sticky=tk.W)
ttk.Scale(servo_frame, from_=0, to=180, variable=servo3_var, orient=tk.HORIZONTAL).grid(column=1, row=2, sticky=(tk.W, tk.E))
ttk.Label(servo_frame, textvariable=servo3_var).grid(column=2, row=2)

# Speed control
speed_frame = ttk.LabelFrame(control_panel, text="Movement Speed", padding="10")
speed_frame.pack(fill=tk.X, pady=10)

speed_var = tk.DoubleVar(value=movement_speed)
ttk.Label(speed_frame, text="Speed:").grid(column=0, row=0, sticky=tk.W)
ttk.Scale(speed_frame, from_=0.5, to=10, variable=speed_var, orient=tk.HORIZONTAL).grid(column=1, row=0, sticky=(tk.W, tk.E))
ttk.Label(speed_frame, textvariable=speed_var).grid(column=2, row=0)

# Status frame
status_frame = ttk.LabelFrame(control_panel, text="Status", padding="10")
status_frame.pack(fill=tk.X, pady=10)

current_angles_text = tk.StringVar(value="Yaw: 0.0, Pitch: 0.0, Roll: 0.0")
ttk.Label(status_frame, textvariable=current_angles_text).pack(anchor=tk.W)

# Function to send control command to Arduino
def send_control_command():
    global servo1_pos, servo2_pos, servo3_pos, control_mode
    
    current_mode = mode_var.get()
    
    if current_mode != control_mode:
        control_mode = current_mode
        print(f"Switched to {control_mode} mode")
    
    if control_mode == "CONTROL":
        servo1_pos = servo1_var.get()
        servo2_pos = servo2_var.get()
        servo3_pos = servo3_var.get()
        command = f"CONTROL,{servo1_pos},{servo2_pos},{servo3_pos}\n"
        ser.write(command.encode())
        ser.flush()
    else:  # RANDOM mode
        ser.write(b"RANDOM\n")
        ser.flush()
    
    # Also update speed if changed
    new_speed = speed_var.get()
    if abs(new_speed - movement_speed) > 0.1:
        speed_command = f"SPEED,{new_speed}\n"
        ser.write(speed_command.encode())
        ser.flush()

# Add control button
ttk.Button(control_panel, text="Send Command", command=send_control_command).pack(pady=10)

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
