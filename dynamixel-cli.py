import sys
import os
import yaml
import time
import threading
import glob
import logging
import argparse
import signal

# Conditional import for getch (not actively used in the main CLI input loop)
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    # For Linux/Raspberry Pi
    import sys as _sys, tty as _tty, termios as _termios # Use aliases to avoid conflict
    _fd_stdin = _sys.stdin.fileno()
    try:
        _old_settings_stdin = _termios.tcgetattr(_fd_stdin)
    except _termios.error: # Happens if not run from a real terminal (e.g. piped input)
        _old_settings_stdin = None

    def getch():
        if not _old_settings_stdin:
            return _sys.stdin.read(1) # Fallback if not a TTY
        try:
            _tty.setraw(_sys.stdin.fileno())
            ch = _sys.stdin.read(1)
        finally:
            _termios.tcsetattr(_fd_stdin, _termios.TCSADRAIN, _old_settings_stdin)
        return ch

# Dynamixel SDK import
try:
    from dynamixel_sdk import * # Uses Dynamixel SDK library
except ImportError:
    print("Error: Failed to import dynamixel_sdk.")
    print("Please install the library: pip3 install dynamixel-sdk")
    sys.exit(1)

# --- Configuration & Constants ---
CONFIG_FILE = 'config.yaml'
# This value represents how many RPM corresponds to one unit of the Dynamixel velocity.
# For XL430-W250, this is approximately 0.229 RPM per unit.
# YOU MAY NEED TO ADJUST THIS for your specific servo model and voltage for accurate RPM.
RPM_PER_UNIT_VELOCITY = 0.229

logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

# --- Load Configuration from YAML ---
config = None
try:
    with open(CONFIG_FILE, 'r') as f:
        config_full = yaml.safe_load(f)
        if 'dynamixel_settings' not in config_full:
            print(f"Error: 'dynamixel_settings' key not found at the top level of '{CONFIG_FILE}'.")
            sys.exit(1)
        config = config_full['dynamixel_settings']
    print(f"Loaded configuration from {CONFIG_FILE}")
except FileNotFoundError:
    print(f"Error: Configuration file '{CONFIG_FILE}' not found.")
    print("Please ensure 'config.yaml' is in the same directory and correctly set up.")
    sys.exit(1)
except yaml.YAMLError as e:
    print(f"Error parsing configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred while loading config: {e}")
    sys.exit(1)

# --- Dynamixel Settings from Config ---
try:
    PROTOCOL_VERSION        = float(config['PROTOCOL_VERSION'])
    BAUDRATE                = int(config['BAUDRATE'])
    DEVICENAME              = config.get('DEVICENAME', 'auto')
    SERVO_IDS               = config.get('SERVO_IDS', [])
    if not isinstance(SERVO_IDS, list) or not all(isinstance(sid, int) for sid in SERVO_IDS):
        raise ValueError("SERVO_IDS must be a list of integers in config.yaml.")

    ADDR_OPERATING_MODE     = int(config['ADDR_OPERATING_MODE'])
    ADDR_TORQUE_ENABLE      = int(config['ADDR_TORQUE_ENABLE'])
    ADDR_GOAL_VELOCITY      = int(config['ADDR_GOAL_VELOCITY'])
    ADDR_PRESENT_VELOCITY   = int(config.get('ADDR_PRESENT_VELOCITY', 128)) # Default for many X-series

    MODE_VELOCITY_CONTROL   = int(config['MODE_VELOCITY_CONTROL'])

    TORQUE_ENABLE           = int(config['TORQUE_ENABLE'])
    TORQUE_DISABLE          = int(config['TORQUE_DISABLE'])
    MAX_VELOCITY_UNIT       = int(config.get('MAX_VELOCITY_UNIT', 1023)) # Default for many X-series

    DEFAULT_START_RPM       = float(config.get('DEFAULT_START_RPM', 25.0))

except KeyError as e:
    print(f"Error: Missing required key {e} in 'dynamixel_settings' in '{CONFIG_FILE}'.")
    sys.exit(1)
except ValueError as e:
    print(f"Error: Invalid value type in configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred processing config values: {e}")
    sys.exit(1)

# --- Auto-detect Serial Port if Needed ---
def auto_detect_serial_port():
    # Linux: /dev/ttyUSB*, /dev/ttyACM*
    # macOS: /dev/tty.usbserial*, /dev/tty.usbmodem*
    candidates = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
    candidates += glob.glob('/dev/tty.usbserial*') + glob.glob('/dev/tty.usbmodem*')
    if not candidates:
        logging.error("No serial ports found matching /dev/ttyUSB*, /dev/ttyACM*, /dev/tty.usbserial*, or /dev/tty.usbmodem*. Please connect your U2D2 and try again.")
        return None
    if len(candidates) > 1:
        logging.warning(f"Multiple serial ports found: {candidates}. Using the first one: {candidates[0]}")
    else:
        logging.info(f"Auto-detected serial port: {candidates[0]}")
    return candidates[0]

if DEVICENAME == 'auto' or not DEVICENAME:
    detected_port = auto_detect_serial_port()
    if detected_port:
        DEVICENAME = detected_port
    else:
        print("Could not auto-detect a serial port. Exiting.")
        sys.exit(1)

# --- Communication Results ---
COMM_SUCCESS                = 0
COMM_TX_FAIL                = -1001

# --- Global lock for synchronizing Dynamixel communication ---
dxl_lock = threading.Lock()

# --- Initialize PortHandler and PacketHandler ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# --- Helper Functions ---
def check_comm_result(servo_id, dxl_comm_result, dxl_error, operation_name="Operation"):
    """Checks Dynamixel communication result and prints error if any."""
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Servo {servo_id}: {operation_name} failed. {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Servo {servo_id}: {operation_name} error. {packetHandler.getRxPacketError(dxl_error)}")
        return False
    return True

def set_torque_status(servo_id, enable):
    """Enable or disable torque for a specific servo."""
    value = TORQUE_ENABLE if enable else TORQUE_DISABLE
    action_str = "Enabling" if enable else "Disabling"
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
    if check_comm_result(servo_id, dxl_comm_result, dxl_error, f"{action_str} Torque"):
        status = "enabled" if enable else "disabled"
        # print(f"Servo {servo_id}: Torque {status}.") # Can be too verbose for some commands
        return True
    else:
        print(f"Servo {servo_id}: Failed to set torque to {action_str.lower()[:-3]}.")
        return False

def set_operating_mode_dxl(servo_id, mode):
    """Set the operating mode for a specific servo."""
    mode_name_map = {MODE_VELOCITY_CONTROL: "Velocity Control"}
    mode_name = mode_name_map.get(mode, f"Unknown Mode ({mode})")

    # print(f"Servo {servo_id}: Attempting to set operating mode to {mode_name}...")
    # Torque must be disabled before changing operating mode for many servos
    print(f"Servo {servo_id}: Ensuring torque is OFF before mode change...")
    set_torque_status(servo_id, False) # Attempt to disable torque first
    time.sleep(0.05) # Small delay recommended by some manuals

    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, mode)
    
    if check_comm_result(servo_id, dxl_comm_result, dxl_error, f"Set Operating Mode to {mode_name}"):
        print(f"Servo {servo_id}: Operating mode set to {mode_name}.")
        return True
    else:
        print(f"Servo {servo_id}: Failed to set operating mode.")
        return False

def set_goal_velocity_dxl(servo_id, dxl_velocity_value):
    """Set the goal velocity for a specific servo (expects Dynamixel units)."""
    clamped_velocity = max(-MAX_VELOCITY_UNIT, min(MAX_VELOCITY_UNIT, int(dxl_velocity_value)))
    
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_VELOCITY, clamped_velocity)
    
    if not check_comm_result(servo_id, dxl_comm_result, dxl_error, f"Set Goal Velocity to {clamped_velocity}"):
        # print(f"Servo {servo_id}: Failed to set goal velocity.") # Can be verbose
        return False
    return True

def get_present_velocity_dxl(servo_id):
    """Read the present velocity of a specific servo (returns Dynamixel units)."""
    with dxl_lock:
        present_velocity_dxl, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_VELOCITY)
    if check_comm_result(servo_id, dxl_comm_result, dxl_error, "Read Present Velocity"):
        # Convert 2's complement if value is large (negative) for 4-byte signed value
        if present_velocity_dxl > (2**31 -1) : # Max positive for 32-bit signed
             present_velocity_dxl -= 2**32
        return present_velocity_dxl
    return None

# --- RPM Conversion Functions ---
def rpm_to_dxl_velocity(rpm_value):
    """Converts RPM to Dynamixel velocity units."""
    if RPM_PER_UNIT_VELOCITY == 0:
        print("Error: RPM_PER_UNIT_VELOCITY is zero, cannot convert RPM.")
        return 0
    return int(round(float(rpm_value) / RPM_PER_UNIT_VELOCITY))

def dxl_velocity_to_rpm(dxl_velocity_value):
    """Converts Dynamixel velocity units to RPM."""
    return round(float(dxl_velocity_value) * RPM_PER_UNIT_VELOCITY, 2)

# --- Main CLI Application ---
def main_cli():
    # --- Open Port ---
    if portHandler.openPort():
        print(f"Succeeded to open the port: {DEVICENAME}")
    else:
        print(f"Failed to open the port: {DEVICENAME}")
        print("Check DEVICENAME in config.yaml and ensure U2D2 is connected.")
        print("On Linux, you might need permissions (e.g., add user to 'dialout' group).")
        return

    # --- Set Port Baudrate ---
    if portHandler.setBaudRate(BAUDRATE):
        print(f"Succeeded to change the baudrate to {BAUDRATE}")
    else:
        print(f"Failed to change the baudrate to {BAUDRATE}")
        portHandler.closePort()
        return

    # --- Initial Servo Setup ---
    if not SERVO_IDS:
        print("Warning: No SERVO_IDS defined in config.yaml. Nothing to control.")
    else:
        print(f"\nInitializing {len(SERVO_IDS)} servos to ~{DEFAULT_START_RPM} RPM...")
        initial_dxl_vel_unit = rpm_to_dxl_velocity(DEFAULT_START_RPM)

        for sid in SERVO_IDS:
            print(f"\n--- Initializing Servo ID: {sid} ---")
            if not set_operating_mode_dxl(sid, MODE_VELOCITY_CONTROL):
                print(f"Servo {sid}: CRITICAL - Failed to set velocity control mode. Skipping for this servo.")
                continue
            
            if not set_torque_status(sid, True):
                print(f"Servo {sid}: CRITICAL - Failed to enable torque. Skipping for this servo.")
                continue
            
            time.sleep(0.05) # Small delay after enabling torque

            current_target_dxl_vel = initial_dxl_vel_unit
            if sid == 2: # Servo 2 goes the opposite way
                current_target_dxl_vel = -initial_dxl_vel_unit
            
            rpm_val_for_log = dxl_velocity_to_rpm(current_target_dxl_vel)
            print(f"Servo {sid}: Setting initial speed to {rpm_val_for_log} RPM (DXL Unit: {current_target_dxl_vel}).")
            if not set_goal_velocity_dxl(sid, current_target_dxl_vel):
                print(f"Servo {sid}: Failed to set initial velocity.")
            else:
                print(f"Servo {sid}: Initial velocity set successfully.")

    print("\n--- Servo Initialization Complete ---")
    print("\nDynamixel CLI Controller")
    print("Commands:")
    print("  set <id> <rpm>            - Set target RPM (e.g., set 1 50)")
    print("  get <id>                  - Get current RPM (e.g., get 1)")
    print("  off <id>                  - Stop servo (set RPM to 0) (e.g., off 1)")
    print("  torque <id> <on|off>      - Enable/disable torque (e.g., torque 1 on)")
    print("  spin <rpm>                - Set ALL servos to <rpm> (servo 2 opposite)")
    print("  stopall                   - Stop ALL servos and disable torque")
    print("  statusall                 - Get current RPM for ALL servos")
    print("  ping <id>                 - Ping a servo to check communication")
    print("  exit                      - Close port and exit")

    try:
        while True:
            try:
                command_input = input("> ").strip().lower().split()
            except EOFError: # Handle piped input or Ctrl+D
                print("\nEOF received, exiting.")
                break
            if not command_input:
                continue

            cmd = command_input[0]

            if cmd == "exit":
                break
            
            elif cmd == "set":
                if len(command_input) == 3:
                    try:
                        servo_id = int(command_input[1])
                        rpm = float(command_input[2])
                        if servo_id not in SERVO_IDS:
                            print(f"Error: Servo ID {servo_id} not in configured SERVO_IDS: {SERVO_IDS}")
                            continue
                        dxl_vel = rpm_to_dxl_velocity(rpm)
                        print(f"Setting Servo {servo_id} to {rpm} RPM (DXL Unit: {dxl_vel})")
                        if not set_torque_status(servo_id, True): # Ensure torque is on
                             print(f"Servo {servo_id}: Warning - Could not ensure torque is on. Velocity command may not work.")
                        set_goal_velocity_dxl(servo_id, dxl_vel)
                    except ValueError:
                        print("Invalid input. Usage: set <servo_id> <rpm_value>")
                else:
                    print("Usage: set <servo_id> <rpm_value>")

            elif cmd == "get":
                if len(command_input) == 2:
                    try:
                        servo_id = int(command_input[1])
                        if servo_id not in SERVO_IDS:
                            print(f"Error: Servo ID {servo_id} not in configured SERVO_IDS: {SERVO_IDS}")
                            continue
                        dxl_vel = get_present_velocity_dxl(servo_id)
                        if dxl_vel is not None:
                            rpm = dxl_velocity_to_rpm(dxl_vel)
                            print(f"Servo {servo_id}: Present Velocity = {rpm} RPM (DXL Unit: {dxl_vel})")
                        else:
                            print(f"Servo {servo_id}: Failed to read present velocity.")
                    except ValueError:
                        print("Invalid input. Usage: get <servo_id>")
                else:
                    print("Usage: get <servo_id>")
            
            elif cmd == "off":
                if len(command_input) == 2:
                    try:
                        servo_id = int(command_input[1])
                        if servo_id not in SERVO_IDS:
                            print(f"Error: Servo ID {servo_id} not in configured SERVO_IDS: {SERVO_IDS}")
                            continue
                        print(f"Stopping Servo {servo_id} (setting RPM to 0).")
                        set_goal_velocity_dxl(servo_id, 0)
                    except ValueError:
                        print("Invalid input. Usage: off <servo_id>")
                else:
                    print("Usage: off <servo_id>")

            elif cmd == "torque":
                if len(command_input) == 3:
                    try:
                        servo_id = int(command_input[1])
                        state = command_input[2]
                        if servo_id not in SERVO_IDS:
                            print(f"Error: Servo ID {servo_id} not in configured SERVO_IDS: {SERVO_IDS}")
                            continue
                        if state == "on":
                            if set_torque_status(servo_id, True):
                                print(f"Servo {servo_id}: Torque enabled.")
                        elif state == "off":
                            if set_torque_status(servo_id, False):
                                print(f"Servo {servo_id}: Torque disabled.")
                        else:
                            print("Invalid state. Use 'on' or 'off'.")
                    except ValueError:
                        print("Invalid servo_id. Usage: torque <servo_id> <on|off>")
                else:
                    print("Usage: torque <servo_id> <on|off>")
            
            elif cmd == "spin":
                if len(command_input) == 2:
                    try:
                        rpm_target = float(command_input[1])
                        target_dxl_vel = rpm_to_dxl_velocity(rpm_target)
                        print(f"Setting all servos to target {rpm_target} RPM (Servo 2 opposite if applicable).")
                        for sid_loop in SERVO_IDS:
                            vel_to_set = target_dxl_vel
                            if sid_loop == 2: # Servo 2 opposite
                                vel_to_set = -target_dxl_vel
                            
                            rpm_val_for_log = dxl_velocity_to_rpm(vel_to_set)
                            print(f"  Servo {sid_loop}: target {rpm_val_for_log} RPM (DXL: {vel_to_set})")
                            if not set_torque_status(sid_loop, True):
                                print(f"  Servo {sid_loop}: Warning - Could not ensure torque is on.")
                            set_goal_velocity_dxl(sid_loop, vel_to_set)
                    except ValueError:
                        print("Invalid RPM. Usage: spin <rpm_value>")
                else:
                    print("Usage: spin <rpm_value>")

            elif cmd == "stopall":
                print("Stopping all servos and disabling torque...")
                for sid_loop in SERVO_IDS:
                    print(f"  Stopping Servo {sid_loop}...")
                    set_goal_velocity_dxl(sid_loop, 0)
                    time.sleep(0.02) 
                    set_torque_status(sid_loop, False)
                print("All servos should be stopped and torque disabled.")

            elif cmd == "statusall":
                print("Current status of all configured servos:")
                if not SERVO_IDS: print("  No servos configured.")
                for sid_loop in SERVO_IDS:
                    dxl_vel = get_present_velocity_dxl(sid_loop)
                    if dxl_vel is not None:
                        rpm = dxl_velocity_to_rpm(dxl_vel)
                        print(f"  Servo {sid_loop}: {rpm} RPM (DXL Unit: {dxl_vel})")
                    else:
                        print(f"  Servo {sid_loop}: Failed to read status.")
            
            elif cmd == "ping":
                if len(command_input) == 2:
                    try:
                        servo_id_to_ping = int(command_input[1])
                        # No lock needed for ping as it's a direct SDK call not writing data
                        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, servo_id_to_ping)
                        if dxl_comm_result != COMM_SUCCESS:
                            print(f"Ping Servo {servo_id_to_ping}: Failed. Result: {packetHandler.getTxRxResult(dxl_comm_result)}")
                        elif dxl_error != 0:
                            print(f"Ping Servo {servo_id_to_ping}: Error. Result: {packetHandler.getRxPacketError(dxl_error)}")
                        else:
                            print(f"Ping Servo {servo_id_to_ping}: Success. Model Number: {dxl_model_number}")
                    except ValueError:
                        print("Invalid servo_id. Usage: ping <servo_id>")
                else:
                    print("Usage: ping <servo_id>")
            else:
                print(f"Unknown command: {cmd}")

    except KeyboardInterrupt:
        print("\nExiting due to KeyboardInterrupt...")
    finally:
        print("\nCleaning up: stopping servos and closing port...")
        if portHandler.is_open and SERVO_IDS: # Only try if port was open and servos were configured
            for sid in SERVO_IDS:
                print(f"  Stopping servo {sid} and disabling torque...")
                try:
                    set_goal_velocity_dxl(sid, 0)
                    time.sleep(0.05) 
                    set_torque_status(sid, False)
                except Exception as e:
                    print(f"  Exception during cleanup for servo {sid}: {e}")
        
        if portHandler.is_open:
            portHandler.closePort()
            print("Port closed.")
        print("Application terminated.")

def run_service_mode():
    """Run the script in headless service mode: initialize servos, keep running, and clean up on SIGTERM/SIGINT."""
    def cleanup_and_exit(signum=None, frame=None):
        print("\n[Service Mode] Cleaning up: stopping servos and closing port...")
        if portHandler.is_open and SERVO_IDS:
            for sid in SERVO_IDS:
                print(f"  Stopping servo {sid} and disabling torque...")
                try:
                    set_goal_velocity_dxl(sid, 0)
                    time.sleep(0.05)
                    set_torque_status(sid, False)
                except Exception as e:
                    print(f"  Exception during cleanup for servo {sid}: {e}")
        if portHandler.is_open:
            portHandler.closePort()
            print("Port closed.")
        print("[Service Mode] Application terminated.")
        sys.exit(0)

    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGTERM, cleanup_and_exit)
    signal.signal(signal.SIGINT, cleanup_and_exit)

    # --- Open Port ---
    if portHandler.openPort():
        print(f"[Service Mode] Succeeded to open the port: {DEVICENAME}")
    else:
        print(f"[Service Mode] Failed to open the port: {DEVICENAME}")
        print("Check DEVICENAME in config.yaml and ensure U2D2 is connected.")
        return

    # --- Set Port Baudrate ---
    if portHandler.setBaudRate(BAUDRATE):
        print(f"[Service Mode] Succeeded to change the baudrate to {BAUDRATE}")
    else:
        print(f"[Service Mode] Failed to change the baudrate to {BAUDRATE}")
        portHandler.closePort()
        return

    # --- Initial Servo Setup ---
    if not SERVO_IDS:
        print("[Service Mode] Warning: No SERVO_IDS defined in config.yaml. Nothing to control.")
    else:
        print(f"\n[Service Mode] Initializing {len(SERVO_IDS)} servos to ~{DEFAULT_START_RPM} RPM...")
        initial_dxl_vel_unit = rpm_to_dxl_velocity(DEFAULT_START_RPM)
        for sid in SERVO_IDS:
            print(f"\n--- Initializing Servo ID: {sid} ---")
            if not set_operating_mode_dxl(sid, MODE_VELOCITY_CONTROL):
                print(f"Servo {sid}: CRITICAL - Failed to set velocity control mode. Skipping for this servo.")
                continue
            if not set_torque_status(sid, True):
                print(f"Servo {sid}: CRITICAL - Failed to enable torque. Skipping for this servo.")
                continue
            time.sleep(0.05)
            current_target_dxl_vel = initial_dxl_vel_unit
            if sid == 2:
                current_target_dxl_vel = -initial_dxl_vel_unit
            rpm_val_for_log = dxl_velocity_to_rpm(current_target_dxl_vel)
            print(f"Servo {sid}: Setting initial speed to {rpm_val_for_log} RPM (DXL Unit: {current_target_dxl_vel}).")
            if not set_goal_velocity_dxl(sid, current_target_dxl_vel):
                print(f"Servo {sid}: Failed to set initial velocity.")
            else:
                print(f"Servo {sid}: Initial velocity set successfully.")
    print("\n[Service Mode] Servo Initialization Complete. Running as a background service. Waiting for SIGTERM/SIGINT...")
    try:
        while True:
            time.sleep(1)
    except Exception as e:
        print(f"[Service Mode] Exception: {e}")
        cleanup_and_exit()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Dynamixel CLI and Service")
    parser.add_argument("--service", action="store_true", help="Run in headless service mode (for systemd)")
    args = parser.parse_args()

    if DEVICENAME.startswith("/dev/ttyACM") or DEVICENAME.lower().startswith("com"):
        print(f"Reminder: DEVICENAME is '{DEVICENAME}'. On Raspberry Pi, it's often '/dev/ttyUSB0' or similar.")
        print("Please ensure this is correct in your 'config.yaml'.")

    if args.service:
        run_service_mode()
    else:
        main_cli()