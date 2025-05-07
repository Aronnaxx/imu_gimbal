import sys
import os
import yaml
import time
import random
import threading
import argparse
import logging

# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] %(levelname)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger(__name__)

# Conditional import for Dynamixel SDK based on OS
if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Dynamixel SDK import
try:
    from dynamixel_sdk import *
except ImportError:
    logger.error("Failed to import dynamixel_sdk. Please install with: pip install dynamixel-sdk")
    sys.exit(1)

# --- Load Configuration from YAML ---
CONFIG_FILE = 'config.yaml'
try:
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)['dynamixel_settings']
    logger.info(f"Loaded configuration from {CONFIG_FILE}")
except FileNotFoundError:
    logger.error(f"Configuration file '{CONFIG_FILE}' not found.")
    sys.exit(1)
except yaml.YAMLError as e:
    logger.error(f"Error parsing configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except KeyError:
    logger.error(f"'dynamixel_settings' key not found in '{CONFIG_FILE}'.")
    sys.exit(1)
except Exception as e:
    logger.error(f"Unexpected error loading config: {e}")
    sys.exit(1)

# --- Dynamixel Settings from Config ---
try:
    PROTOCOL_VERSION        = float(config['PROTOCOL_VERSION'])
    BAUDRATE                = int(config['BAUDRATE'])
    DEVICENAME              = config['DEVICENAME']
    SERVO_IDS               = config['SERVO_IDS']
    if not isinstance(SERVO_IDS, list) or not all(isinstance(sid, int) for sid in SERVO_IDS):
        raise ValueError("SERVO_IDS must be a list of integers.")
    ADDR_OPERATING_MODE     = int(config['ADDR_OPERATING_MODE'])
    ADDR_TORQUE_ENABLE      = int(config['ADDR_TORQUE_ENABLE'])
    ADDR_PROFILE_VELOCITY   = int(config.get('ADDR_PROFILE_VELOCITY', 112))
    ADDR_GOAL_VELOCITY      = int(config['ADDR_GOAL_VELOCITY'])
    ADDR_GOAL_POSITION      = int(config['ADDR_GOAL_POSITION'])
    ADDR_PRESENT_POSITION   = int(config['ADDR_PRESENT_POSITION'])
    ADDR_PRESENT_VELOCITY   = int(config.get('ADDR_PRESENT_VELOCITY', 128))
    ADDR_PRESENT_TEMPERATURE = int(config.get('ADDR_PRESENT_TEMPERATURE', 129))
    ADDR_PRESENT_LOAD       = int(config.get('ADDR_PRESENT_LOAD', 130))
    MODE_VELOCITY_CONTROL   = int(config['MODE_VELOCITY_CONTROL'])
    MODE_POSITION_CONTROL   = int(config['MODE_POSITION_CONTROL'])
    TORQUE_ENABLE           = int(config['TORQUE_ENABLE'])
    TORQUE_DISABLE          = int(config['TORQUE_DISABLE'])
    MAX_VELOCITY_UNIT       = int(config.get('MAX_VELOCITY_UNIT', 1023))
    ENABLE_RANDOM_MOVEMENT    = bool(config.get('ENABLE_RANDOM_MOVEMENT', False))
    RANDOM_MIN_SPEED_PERCENT  = float(config.get('RANDOM_MIN_SPEED_PERCENT', 10))
    RANDOM_MAX_SPEED_PERCENT  = float(config.get('RANDOM_MAX_SPEED_PERCENT', 70))
    RANDOM_MIN_DURATION_S     = float(config.get('RANDOM_MIN_DURATION_S', 1.0))
    RANDOM_MAX_DURATION_S     = float(config.get('RANDOM_MAX_DURATION_S', 4.0))
except KeyError as e:
    logger.error(f"Missing required key '{e}' in configuration file '{CONFIG_FILE}'.")
    sys.exit(1)
except ValueError as e:
    logger.error(f"Invalid value type in configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except Exception as e:
    logger.error(f"Unexpected error processing config values: {e}")
    sys.exit(1)

COMM_SUCCESS = 0
COMM_TX_FAIL = -1001
stop_event = threading.Event()
dxl_lock = threading.Lock()

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if portHandler.openPort():
    logger.info(f"Opened port: {DEVICENAME}")
else:
    logger.error(f"Failed to open port: {DEVICENAME}")
    getch()
    sys.exit(1)

if portHandler.setBaudRate(BAUDRATE):
    logger.info(f"Set baudrate to {BAUDRATE}")
else:
    logger.error(f"Failed to set baudrate to {BAUDRATE}")
    getch()
    sys.exit(1)

def check_comm_result(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        logger.error(f"%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        logger.error(f"%s" % packetHandler.getRxPacketError(dxl_error))
        return False
    return True

def set_torque(servo_id, enable):
    value = TORQUE_ENABLE if enable else TORQUE_DISABLE
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
        if check_comm_result(dxl_comm_result, dxl_error):
            logger.info(f"Torque for Servo ID {servo_id} {'enabled' if enable else 'disabled'}.")
            return True
        else:
            logger.error(f"Failed to set torque for Servo ID {servo_id}.")
            return False

def set_operating_mode(servo_id, mode):
    with dxl_lock:
        dxl_comm_result_torque, dxl_error_torque = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not check_comm_result(dxl_comm_result_torque, dxl_error_torque):
            logger.warning(f"Failed to disable torque for Servo ID {servo_id} before changing mode, proceeding.")
        time.sleep(0.05)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, mode)
        if check_comm_result(dxl_comm_result, dxl_error):
            mode_name = "Velocity Control" if mode == MODE_VELOCITY_CONTROL else "Position Control"
            logger.info(f"Operating mode for Servo ID {servo_id} set to {mode_name}.")
            return True
        else:
            logger.error(f"Failed to set operating mode for Servo ID {servo_id}.")
            return False

def set_goal_position(servo_id, position):
    logger.info(f"Setting Servo ID {servo_id} Goal Position to {position}")
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, position)
        if not check_comm_result(dxl_comm_result, dxl_error):
            logger.error(f"Failed to set goal position for Servo ID {servo_id}.")

def set_goal_velocity(servo_id, velocity):
    logger.info(f"Setting Servo ID {servo_id} Goal Velocity to {velocity}")
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_VELOCITY, velocity)
        if not check_comm_result(dxl_comm_result, dxl_error):
            logger.error(f"Failed to set goal velocity for Servo ID {servo_id}.")

def read_present_velocity(servo_id):
    with dxl_lock:
        present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_VELOCITY)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_velocity
        return None

def read_present_temperature(servo_id):
    with dxl_lock:
        present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_TEMPERATURE)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_temp
        return None

def read_present_load(servo_id):
    with dxl_lock:
        present_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_LOAD)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_load
        return None

def random_move_thread_func():
    logger.info("Random movement thread started.")
    while not stop_event.is_set():
        for servo_id in SERVO_IDS:
            if stop_event.is_set(): break
            try:
                if not set_operating_mode(servo_id, MODE_VELOCITY_CONTROL):
                    logger.warning(f"[RandomMove] Failed to set Velocity Mode for {servo_id}. Skipping.")
                    stop_event.wait(0.1)
                    continue
                if not set_torque(servo_id, True):
                    logger.warning(f"[RandomMove] Failed to enable torque for {servo_id}. Skipping.")
                    stop_event.wait(0.1)
                    continue
                speed_percent = random.uniform(RANDOM_MIN_SPEED_PERCENT, RANDOM_MAX_SPEED_PERCENT)
                velocity = int((speed_percent / 100.0) * MAX_VELOCITY_UNIT)
                direction = random.choice([-1, 1])
                goal_velocity = max(-MAX_VELOCITY_UNIT, min(MAX_VELOCITY_UNIT, velocity * direction))
                logger.info(f"[RandomMove] Setting Servo {servo_id} velocity to {goal_velocity}")
                set_goal_velocity(servo_id, goal_velocity)
            except Exception as e:
                logger.error(f"[RandomMove] Error controlling servo {servo_id}: {e}")
                stop_event.wait(0.5)
            if stop_event.is_set(): break
        if stop_event.is_set(): break
        duration = random.uniform(RANDOM_MIN_DURATION_S, RANDOM_MAX_DURATION_S)
        logger.info(f"[RandomMove] Waiting for {duration:.2f} seconds...")
        stop_event.wait(duration)
    logger.info("Random movement thread stopping. Setting velocity to 0 and disabling torque.")
    for servo_id in SERVO_IDS:
        try:
            logger.info(f"[RandomMove Cleanup] Setting velocity 0 for {servo_id}")
            set_goal_velocity(servo_id, 0)
            logger.info(f"[RandomMove Cleanup] Disabling torque for {servo_id}")
            set_torque(servo_id, False)
        except Exception as e:
            logger.warning(f"[RandomMove] Error during cleanup for servo {servo_id}: {e}")
    logger.info("Random movement thread finished.")

def print_status(servo_id):
    velocity = read_present_velocity(servo_id)
    temp = read_present_temperature(servo_id)
    load = read_present_load(servo_id)
    print(f"Servo {servo_id} Status:")
    print(f"  Velocity: {velocity}")
    print(f"  Temperature: {temp}°C")
    if load is not None:
        load_percent = (load / 2048) * 100
        print(f"  Load: {load_percent:.1f}%")
    else:
        print(f"  Load: --")

def main():
    parser = argparse.ArgumentParser(description="Dynamixel CLI Control Tool")
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Enable/Disable Torque
    torque_parser = subparsers.add_parser("torque", help="Enable or disable torque")
    torque_parser.add_argument("servo_id", type=int, help="Servo ID")
    torque_parser.add_argument("state", choices=["on", "off"], help="Enable (on) or disable (off) torque")

    # Set Velocity
    vel_parser = subparsers.add_parser("velocity", help="Set velocity for a servo")
    vel_parser.add_argument("servo_id", type=int, help="Servo ID")
    vel_parser.add_argument("velocity", type=int, help="Velocity value (-MAX to MAX)")

    # Set Position
    pos_parser = subparsers.add_parser("position", help="Set position for a servo")
    pos_parser.add_argument("servo_id", type=int, help="Servo ID")
    pos_parser.add_argument("position", type=int, help="Position value (0-4095 typical)")
    pos_parser.add_argument("speed", type=int, default=MAX_VELOCITY_UNIT//2, nargs="?", help="Profile velocity (optional)")

    # Read Status
    status_parser = subparsers.add_parser("status", help="Read status of a servo")
    status_parser.add_argument("servo_id", type=int, help="Servo ID")

    # Random Movement
    rand_parser = subparsers.add_parser("random", help="Toggle random movement thread")
    rand_parser.add_argument("action", choices=["start", "stop"], help="Start or stop random movement")

    args = parser.parse_args()

    if args.command == "torque":
        set_torque(args.servo_id, args.state == "on")
    elif args.command == "velocity":
        set_operating_mode(args.servo_id, MODE_VELOCITY_CONTROL)
        set_torque(args.servo_id, True)
        set_goal_velocity(args.servo_id, args.velocity)
    elif args.command == "position":
        set_operating_mode(args.servo_id, MODE_POSITION_CONTROL)
        set_torque(args.servo_id, True)
        # Set profile velocity
        with dxl_lock:
            dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, args.servo_id, ADDR_PROFILE_VELOCITY, args.speed)
            check_comm_result(dxl_comm_result, dxl_error)
        set_goal_position(args.servo_id, args.position)
    elif args.command == "status":
        print_status(args.servo_id)
    elif args.command == "random":
        if args.action == "start":
            if not hasattr(main, "random_thread") or not main.random_thread.is_alive():
                stop_event.clear()
                main.random_thread = threading.Thread(target=random_move_thread_func, daemon=True)
                main.random_thread.start()
                logger.info("Random movement started.")
            else:
                logger.info("Random movement already running.")
        elif args.action == "stop":
            if hasattr(main, "random_thread") and main.random_thread.is_alive():
                stop_event.set()
                main.random_thread.join(timeout=1.0)
                logger.info("Random movement stopped.")
            else:
                logger.info("Random movement is not running.")

    # Clean up
    if portHandler.is_open:
        portHandler.closePort()

if __name__ == "__main__":
    main() 