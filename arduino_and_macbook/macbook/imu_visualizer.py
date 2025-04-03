import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from mpl_toolkits.mplot3d import Axes3D
import serial.tools.list_ports
import numpy as np
import argparse
import sys
import time
from collections import deque

# Custom theme colors
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'Arduino' in port.description or 'usbmodem' in port.device:
            return port.device
    return None

def main():
    parser = argparse.ArgumentParser(description='IMU 3D Visualizer')
    parser.add_argument('--port', help='Serial port to use (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--duration', type=int, help='Run time in seconds (optional)')
    parser.add_argument('--log', help='Path to log file for saving data')
    args = parser.parse_args()

    # Find Arduino port
    port = args.port or find_arduino_port()
    if not port:
        print("No Arduino found. Available ports:")
        for port in serial.tools.list_ports.comports():
            print(f" - {port.device}: {port.description}")
        sys.exit(1)

    # Initialize serial connection
    try:
        ser = serial.Serial(port, args.baud, timeout=1)
        print(f"Connected to {port} at {args.baud} baud")
    except serial.SerialException as e:
        print(f"Error connecting to serial port: {e}")
        sys.exit(1)

    # Lists to hold Euler angle data
    x_data, y_data, z_data = [], [], []
    max_points = 200  # Maximum number of points to display

    # Regular expression to parse serial data
    euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

    # Create the figure and 3D axis
    plt.style.use('dark_background')
    fig = plt.figure(figsize=(10, 8), facecolor=DARK_BG)
    ax = fig.add_subplot(111, projection='3d')
    ax.set_facecolor(DARKER_BG)

    # Create the line and dot for visualization
    line, = ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
    dot = ax.plot([], [], [], marker='o', label='Current Orientation', color=ACCENT_COLOR, markersize=8)[0]

    # Set initial axis limits
    plot_range = 180
    ax.set_xlim(-plot_range, plot_range)
    ax.set_ylim(-plot_range, plot_range)
    ax.set_zlim(-plot_range, plot_range)

    # Configure the plot appearance
    ax.set_title("3D Orientation Trace (Yaw, Pitch, Roll)", color=TEXT_COLOR, fontsize=14)
    ax.set_xlabel("Yaw", color=TEXT_COLOR)
    ax.set_ylabel("Pitch", color=TEXT_COLOR)
    ax.set_zlabel("Roll", color=TEXT_COLOR)
    ax.tick_params(colors=TEXT_COLOR)
    ax.grid(True, linestyle='--', alpha=0.3)
    ax.legend(facecolor=DARKER_BG, edgecolor=HIGHLIGHT)

    def update_plot(frame):
        nonlocal x_data, y_data, z_data

        # Read all available data
        while ser.in_waiting > 0:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            match = euler_regex.match(line)
            if match:
                yaw = float(match.group(1))
                pitch = float(match.group(2))
                roll = float(match.group(3))

                # Add new data points
                x_data.append(yaw)
                y_data.append(pitch)
                z_data.append(roll)

                # Limit the number of points
                if len(x_data) > max_points:
                    x_data = x_data[-max_points:]
                    y_data = y_data[-max_points:]
                    z_data = z_data[-max_points:]

                # Update the line and dot
                if len(x_data) > 0:
                    line.set_data(x_data, y_data)
                    line.set_3d_properties(z_data)
                    dot.set_data(x_data[-1:], y_data[-1:])
                    dot.set_3d_properties(z_data[-1:])

                    # Update plot limits if needed
                    if len(x_data) > 1:
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

        return line, dot

    # Create the animation
    ani = FuncAnimation(fig, update_plot, interval=10, blit=True)

    # Show the plot
    try:
        plt.show()
    except KeyboardInterrupt:
        print("Exiting via Ctrl+C")
    finally:
        ser.close()
        if log_file:
            log_file.close()
        print("\nDisconnected")

if __name__ == "__main__":
    main() 
