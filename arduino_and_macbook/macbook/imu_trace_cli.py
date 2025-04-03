import serial
import re
import time
import serial.tools.list_ports
import argparse
import sys
from datetime import datetime

# Custom theme and style constants
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"
SLIDER_COLOR = "#3498db"
SUCCESS_COLOR = "#2ecc71"
DANGER_COLOR = "#e74c3c"

def find_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if 'Arduino' in port.description or 'usbmodem' in port.device:
            return port.device
    return None

def print_status(yaw, pitch, roll, mode="RANDOM", active=False):
    """Print a formatted status line with current angles and mode"""
    status = "RUNNING" if active else "STOPPED"
    color = SUCCESS_COLOR if active else DANGER_COLOR
    print(f"\rYaw: {yaw:6.1f}° | Pitch: {pitch:6.1f}° | Roll: {roll:6.1f}° | Mode: {mode:8} | Status: {status}", end="")

def main():
    parser = argparse.ArgumentParser(description='IMU Gimbal Control CLI')
    parser.add_argument('--port', help='Serial port to use (default: auto-detect)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--mode', choices=['RANDOM', 'CONTROL', 'FOLLOW'], default='RANDOM', help='Initial mode (default: RANDOM)')
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

    # Regular expression to parse serial data
    euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

    # Set initial mode
    ser.write(f"{args.mode}\n".encode())
    ser.flush()
    print(f"Set mode to {args.mode}")

    # Start movement
    ser.write(b"START\n")
    ser.flush()
    print("Started movement")

    try:
        while True:
            # Process all available data
            while ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').strip()
                match = euler_regex.match(line)
                if match:
                    yaw = float(match.group(1))
                    pitch = float(match.group(2))
                    roll = float(match.group(3))
                    print_status(yaw, pitch, roll, args.mode, True)
                elif line and not line.startswith("Euler:"):
                    print(f"\n{line}")

            # Check for user input
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                cmd = sys.stdin.readline().strip().upper()
                if cmd == 'Q':
                    break
                elif cmd == 'S':
                    ser.write(b"STOP\n")
                    ser.flush()
                    print("\nStopped movement")
                elif cmd == 'G':
                    ser.write(b"START\n")
                    ser.flush()
                    print("\nStarted movement")
                elif cmd == 'H':
                    ser.write(b"HOME\n")
                    ser.flush()
                    print("\nHoming servos")
                elif cmd == 'Z':
                    ser.write(b"ZERO\n")
                    ser.flush()
                    print("\nZeroing IMU")
                elif cmd.startswith('M '):
                    mode = cmd[2:]
                    if mode in ['RANDOM', 'CONTROL', 'FOLLOW']:
                        ser.write(f"{mode}\n".encode())
                        ser.flush()
                        args.mode = mode
                        print(f"\nSwitched to {mode} mode")
                elif cmd.startswith('SPEED '):
                    try:
                        speed = float(cmd[6:])
                        if 0.5 <= speed <= 10.0:
                            ser.write(f"SPEED,{speed}\n".encode())
                            ser.flush()
                            print(f"\nSet speed to {speed}")
                    except ValueError:
                        print("\nInvalid speed value")
                elif cmd == '?':
                    print("\nAvailable commands:")
                    print("  Q - Quit")
                    print("  S - Stop movement")
                    print("  G - Start movement")
                    print("  H - Home servos")
                    print("  Z - Zero IMU")
                    print("  M [RANDOM|CONTROL|FOLLOW] - Set mode")
                    print("  SPEED [0.5-10.0] - Set movement speed")
                    print("  ? - Show this help")

            time.sleep(0.01)  # Small delay to prevent CPU hogging

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        # Stop movement and close serial
        ser.write(b"STOP\n")
        ser.flush()
        ser.close()
        print("\nDisconnected")

if __name__ == "__main__":
    main() 
