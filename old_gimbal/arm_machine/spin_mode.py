import sys
import os
import time
import signal
import yaml
import logging

try:
    from dynamixel_sdk import *
except ImportError:
    print("Error: Failed to import dynamixel_sdk. Please install it with 'pip install dynamixel-sdk'.")
    sys.exit(1)

# --- Logging Setup ---
logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')
logger = logging.getLogger("spin_mode")

# --- Load Configuration ---
CONFIG_FILE = os.path.join(os.path.dirname(__file__), '../config.yaml')
try:
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)['dynamixel_settings']
    logger.info(f"Loaded configuration from {CONFIG_FILE}")
except Exception as e:
    logger.error(f"Failed to load config: {e}")
    sys.exit(1)

# --- Extract Config ---
PROTOCOL_VERSION = float(config['PROTOCOL_VERSION'])
BAUDRATE = int(config['BAUDRATE'])
DEVICENAME = config['DEVICENAME']
SERVO_IDS = config['SERVO_IDS']
ADDR_OPERATING_MODE = int(config['ADDR_OPERATING_MODE'])
ADDR_TORQUE_ENABLE = int(config['ADDR_TORQUE_ENABLE'])
ADDR_GOAL_VELOCITY = int(config['ADDR_GOAL_VELOCITY'])
MODE_VELOCITY_CONTROL = int(config['MODE_VELOCITY_CONTROL'])
TORQUE_ENABLE = int(config['TORQUE_ENABLE'])
TORQUE_DISABLE = int(config['TORQUE_DISABLE'])
MAX_VELOCITY_UNIT = int(config.get('MAX_VELOCITY_UNIT', 1023))

# --- Dynamixel SDK Setup ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    logger.error(f"Failed to open port: {DEVICENAME}")
    sys.exit(1)
logger.info(f"Opened port: {DEVICENAME}")

if not portHandler.setBaudRate(BAUDRATE):
    logger.error(f"Failed to set baudrate: {BAUDRATE}")
    portHandler.closePort()
    sys.exit(1)
logger.info(f"Set baudrate: {BAUDRATE}")

# --- Helper Functions ---
def clamp_velocity(velocity):
    if velocity > MAX_VELOCITY_UNIT:
        logger.warning(f"Velocity {velocity} exceeds MAX_VELOCITY_UNIT {MAX_VELOCITY_UNIT}, clamping.")
        return MAX_VELOCITY_UNIT
    elif velocity < -MAX_VELOCITY_UNIT:
        logger.warning(f"Velocity {velocity} below -MAX_VELOCITY_UNIT {-MAX_VELOCITY_UNIT}, clamping.")
        return -MAX_VELOCITY_UNIT
    return velocity

def check_comm_result(dxl_comm_result, dxl_error, context=""):
    if dxl_comm_result != 0:
        logger.error(f"{context} Comm error: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    if dxl_error != 0:
        logger.error(f"{context} Packet error: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    return True

def set_operating_mode(servo_id, mode):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    check_comm_result(dxl_comm_result, dxl_error, f"Servo {servo_id} disable torque")
    time.sleep(0.05)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, mode)
    if check_comm_result(dxl_comm_result, dxl_error, f"Servo {servo_id} set mode {mode}"):
        logger.info(f"Servo {servo_id} set to velocity control mode.")
        return True
    return False

def set_torque(servo_id, enable):
    value = TORQUE_ENABLE if enable else TORQUE_DISABLE
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
    if check_comm_result(dxl_comm_result, dxl_error, f"Servo {servo_id} set torque {enable}"):
        logger.info(f"Servo {servo_id} torque {'enabled' if enable else 'disabled'}.")
        return True
    return False

def set_goal_velocity(servo_id, velocity):
    velocity = clamp_velocity(velocity)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_VELOCITY, velocity)
    if check_comm_result(dxl_comm_result, dxl_error, f"Servo {servo_id} set velocity {velocity}"):
        logger.debug(f"Servo {servo_id} velocity set to {velocity}.")
        return True
    return False

# --- Spin Mode Logic ---
SPIN_PERCENT = 0.2  # 20% of max speed
SPIN_VELOCITY_UNIT = int(SPIN_PERCENT * MAX_VELOCITY_UNIT)
spin_velocities = {
    1: SPIN_VELOCITY_UNIT,   # Positive
    2: -SPIN_VELOCITY_UNIT,  # Negative
    3: SPIN_VELOCITY_UNIT    # Positive
}

# Only use IDs that are present in config
spin_velocities = {sid: spin_velocities.get(sid, SPIN_VELOCITY_UNIT) for sid in SERVO_IDS}

running = True
def shutdown_handler(signum, frame):
    global running
    logger.info("Caught signal, stopping...")
    running = False

signal.signal(signal.SIGINT, shutdown_handler)
signal.signal(signal.SIGTERM, shutdown_handler)

try:
    # Set all servos to velocity mode and enable torque
    for sid in SERVO_IDS:
        set_operating_mode(sid, MODE_VELOCITY_CONTROL)
        time.sleep(0.05)
        set_torque(sid, True)
        time.sleep(0.05)

    # Set velocities
    for sid, vel in spin_velocities.items():
        set_goal_velocity(sid, vel)
        logger.info(f"Servo {sid} spinning at velocity {vel}")

    logger.info("Spin mode active. Press Ctrl+C to stop.")
    while running:
        time.sleep(1)

finally:
    logger.info("Stopping all servos and disabling torque...")
    for sid in SERVO_IDS:
        set_goal_velocity(sid, 0)
        set_torque(sid, False)
    portHandler.closePort()
    logger.info("Shutdown complete.") 