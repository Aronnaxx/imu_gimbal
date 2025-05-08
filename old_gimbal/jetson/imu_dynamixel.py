import tkinter as tk
from tkinter import ttk
import sys
import os
import yaml # Import YAML library
import time # Import time library explicitly
import random # For random movement
import threading # For random movement thread
import queue # For thread-safe communication (if needed later)

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
# Ensure the dynamixel_sdk library is installed: pip install dynamixel-sdk
try:
    from dynamixel_sdk import * # Uses Dynamixel SDK library
except ImportError:
    print("Error: Failed to import dynamixel_sdk.")
    print("Please install the library: pip install dynamixel-sdk")
    sys.exit(1)

# --- Load Configuration from YAML ---
CONFIG_FILE = 'config.yaml'
try:
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)['dynamixel_settings']
    print(f"Loaded configuration from {CONFIG_FILE}")
except FileNotFoundError:
    print(f"Error: Configuration file '{CONFIG_FILE}' not found.")
    sys.exit(1)
except yaml.YAMLError as e:
    print(f"Error parsing configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except KeyError:
    print(f"Error: 'dynamixel_settings' key not found in '{CONFIG_FILE}'.")
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred while loading config: {e}")
    sys.exit(1)

# --- Dynamixel Settings from Config ---
try:
    # Connection & Protocol
    PROTOCOL_VERSION        = float(config['PROTOCOL_VERSION'])
    BAUDRATE                = int(config['BAUDRATE'])
    DEVICENAME              = config['DEVICENAME']
    # Servo IDs
    SERVO_IDS               = config['SERVO_IDS']
    if not isinstance(SERVO_IDS, list) or not all(isinstance(sid, int) for sid in SERVO_IDS):
        raise ValueError("SERVO_IDS must be a list of integers.")
    # Control Table Addresses
    ADDR_OPERATING_MODE     = int(config['ADDR_OPERATING_MODE'])
    ADDR_TORQUE_ENABLE      = int(config['ADDR_TORQUE_ENABLE'])
    ADDR_PROFILE_VELOCITY   = int(config.get('ADDR_PROFILE_VELOCITY', 112)) # Default if missing
    ADDR_GOAL_VELOCITY      = int(config['ADDR_GOAL_VELOCITY'])
    ADDR_GOAL_POSITION      = int(config['ADDR_GOAL_POSITION'])
    ADDR_PRESENT_POSITION   = int(config['ADDR_PRESENT_POSITION'])
    ADDR_PRESENT_VELOCITY   = int(config.get('ADDR_PRESENT_VELOCITY', 128)) # Default if missing
    ADDR_PRESENT_TEMPERATURE = int(config.get('ADDR_PRESENT_TEMPERATURE', 129)) # Default if missing
    ADDR_PRESENT_LOAD       = int(config.get('ADDR_PRESENT_LOAD', 130)) # Default if missing
    # Operating Modes
    MODE_VELOCITY_CONTROL   = int(config['MODE_VELOCITY_CONTROL'])
    MODE_POSITION_CONTROL   = int(config['MODE_POSITION_CONTROL'])
    # Torque Values
    TORQUE_ENABLE           = int(config['TORQUE_ENABLE'])
    TORQUE_DISABLE          = int(config['TORQUE_DISABLE'])
    # Optional Max Velocity Unit (for slider mapping)
    MAX_VELOCITY_UNIT       = int(config.get('MAX_VELOCITY_UNIT', 1023)) # Default if missing

    # Random Movement Config
    ENABLE_RANDOM_MOVEMENT    = bool(config.get('ENABLE_RANDOM_MOVEMENT', False))
    RANDOM_MIN_SPEED_PERCENT  = float(config.get('RANDOM_MIN_SPEED_PERCENT', 10))
    RANDOM_MAX_SPEED_PERCENT  = float(config.get('RANDOM_MAX_SPEED_PERCENT', 70))
    RANDOM_MIN_DURATION_S     = float(config.get('RANDOM_MIN_DURATION_S', 1.0))
    RANDOM_MAX_DURATION_S     = float(config.get('RANDOM_MAX_DURATION_S', 4.0))

except KeyError as e:
    print(f"Error: Missing required key '{e}' in configuration file '{CONFIG_FILE}'.")
    sys.exit(1)
except ValueError as e:
    print(f"Error: Invalid value type in configuration file '{CONFIG_FILE}': {e}")
    sys.exit(1)
except Exception as e:
    print(f"An unexpected error occurred processing config values: {e}")
    sys.exit(1)

# --- Other Constants ---
# (These might not need to be in config, but could be)
COMM_SUCCESS                = 0                 # Communication Success result value
COMM_TX_FAIL                = -1001             # Communication Tx Failed

# --- Global flag to signal threads to stop ---
stop_event = threading.Event()

# --- Global lock for synchronizing Dynamixel communication ---
dxl_lock = threading.Lock()

# --- Initialize PortHandler and PacketHandler ---
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# --- Open Port ---
if portHandler.openPort():
    print(f"Succeeded to open the port: {DEVICENAME}")
else:
    print(f"Failed to open the port: {DEVICENAME}")
    print("Press any key to terminate...")
    getch()
    sys.exit(1)

# --- Set Port Baudrate ---
if portHandler.setBaudRate(BAUDRATE):
    print(f"Succeeded to change the baudrate to {BAUDRATE}")
else:
    print(f"Failed to change the baudrate to {BAUDRATE}")
    print("Press any key to terminate...")
    getch()
    sys.exit(1)

# --- Helper Functions ---
def check_comm_result(dxl_comm_result, dxl_error):
    """Checks Dynamixel communication result and prints error if any."""
    if dxl_comm_result != COMM_SUCCESS:
        print(f"%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print(f"%s" % packetHandler.getRxPacketError(dxl_error))
        return False
    return True

def set_torque(servo_id, enable):
    """Enable or disable torque for a specific servo."""
    value = TORQUE_ENABLE if enable else TORQUE_DISABLE
    result = False
    with dxl_lock: # Acquire lock
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
        if check_comm_result(dxl_comm_result, dxl_error):
            status = "enabled" if enable else "disabled"
            print(f"Torque for Servo ID {servo_id} {status}.")
            result = True
        else:
            print(f"Failed to set torque for Servo ID {servo_id}.")
            result = False
    # Release lock automatically via 'with' statement
    return result

def set_operating_mode(servo_id, mode):
    """Set the operating mode for a specific servo."""
    # Important: Torque must be disabled before changing operating mode
    # We acquire the lock *before* potentially calling set_torque
    result = False
    with dxl_lock: # Acquire lock
        # Try to disable torque first (within the lock)
        dxl_comm_result_torque, dxl_error_torque = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not check_comm_result(dxl_comm_result_torque, dxl_error_torque):
             print(f"Warning: Failed to disable torque for Servo ID {servo_id} before changing mode, but proceeding.")
             # Proceed anyway, maybe torque was already off

        time.sleep(0.05) # Small delay - MUST be within the lock if it affects timing critical to comms

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, servo_id, ADDR_OPERATING_MODE, mode)
        if check_comm_result(dxl_comm_result, dxl_error):
            mode_name = "Velocity Control" if mode == MODE_VELOCITY_CONTROL else "Position Control"
            print(f"Operating mode for Servo ID {servo_id} set to {mode_name}.")
            result = True
            # Do NOT re-enable torque here automatically. Let the caller handle it.
        else:
            print(f"Failed to set operating mode for Servo ID {servo_id}.")
            result = False
    # Release lock automatically
    return result

def set_goal_position(servo_id, position):
    """Set the goal position for a specific servo (requires Position Control mode)."""
    print(f"Setting Servo ID {servo_id} Goal Position to {position}")
    with dxl_lock: # Acquire lock
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_POSITION, position)
        if not check_comm_result(dxl_comm_result, dxl_error):
            print(f"Failed to set goal position for Servo ID {servo_id}.")
    # Release lock automatically

def set_goal_velocity(servo_id, velocity):
    """Set the goal velocity for a specific servo (requires Velocity Control mode)."""
    print(f"Setting Servo ID {servo_id} Goal Velocity to {velocity}")
    # Ensure velocity is within valid range if necessary (e.g., for XL-430, 0-1023 often mapped)
    # The SDK handles the conversion to the appropriate byte format (4 bytes for XL-430 Goal Velocity)
    with dxl_lock: # Acquire lock
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_GOAL_VELOCITY, velocity)
        if not check_comm_result(dxl_comm_result, dxl_error):
            print(f"Failed to set goal velocity for Servo ID {servo_id}.")
    # Release lock automatically

def read_present_velocity(servo_id):
    """Read the present velocity of a specific servo."""
    with dxl_lock:
        present_velocity, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_VELOCITY)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_velocity
        return None

def read_present_temperature(servo_id):
    """Read the present temperature of a specific servo."""
    with dxl_lock:
        present_temp, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_TEMPERATURE)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_temp
        return None

def read_present_load(servo_id):
    """Read the present load of a specific servo."""
    with dxl_lock:
        present_load, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(
            portHandler, servo_id, ADDR_PRESENT_LOAD)
        if check_comm_result(dxl_comm_result, dxl_error):
            return present_load
        return None

# --- Tkinter GUI Setup ---
class DynamixelControlApp:
    def __init__(self, root):
        self.root = root
        root.title("Dynamixel Servo Control")

        self.servo_widgets = {} # To store widgets for each servo
        self.continuous_movement_active = {} # Track continuous movement state for each servo
        self.random_movement_enabled = tk.BooleanVar(value=ENABLE_RANDOM_MOVEMENT)
        self.update_status_active = True # Flag for status updates

        # Create global controls frame
        global_frame = ttk.LabelFrame(root, text="Global Controls", padding="10")
        global_frame.grid(row=0, column=0, columnspan=len(SERVO_IDS), padx=10, pady=5, sticky="ew")
        
        # Random Movement Toggle
        random_frame = ttk.Frame(global_frame)
        random_frame.pack(fill=tk.X, pady=5)
        ttk.Label(random_frame, text="Random Movement:").pack(side=tk.LEFT, padx=5)
        ttk.Checkbutton(random_frame, variable=self.random_movement_enabled, 
                       command=self.toggle_random_movement).pack(side=tk.LEFT)

        # Create frames for each servo
        for i, servo_id in enumerate(SERVO_IDS):
            frame = ttk.LabelFrame(root, text=f"Servo ID: {servo_id}", padding="10")
            frame.grid(row=1, column=i, padx=10, pady=10, sticky="nsew")
            self.create_servo_controls(frame, servo_id)
            self.continuous_movement_active[servo_id] = False

            # Initialize servo to velocity control mode
            if set_operating_mode(servo_id, MODE_VELOCITY_CONTROL):
                set_torque(servo_id, True)

        # Start status update thread
        self.status_thread = threading.Thread(target=self.update_status_loop, daemon=True)
        self.status_thread.start()

        root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def create_servo_controls(self, parent_frame, servo_id):
        """Creates the GUI controls for a single servo."""
        widgets = {}

        # --- Torque Control ---
        torque_frame = ttk.Frame(parent_frame)
        torque_frame.pack(fill=tk.X, pady=5)
        widgets['torque_label'] = ttk.Label(torque_frame, text="Torque:")
        widgets['torque_label'].pack(side=tk.LEFT, padx=5)
        widgets['torque_enable_button'] = ttk.Button(torque_frame, text="Enable", 
            command=lambda sid=servo_id: self.toggle_torque(sid, True))
        widgets['torque_enable_button'].pack(side=tk.LEFT, padx=2)
        widgets['torque_disable_button'] = ttk.Button(torque_frame, text="Disable", 
            command=lambda sid=servo_id: self.toggle_torque(sid, False))
        widgets['torque_disable_button'].pack(side=tk.LEFT, padx=2)

        # --- Velocity Control ---
        velocity_frame = ttk.LabelFrame(parent_frame, text="Velocity Control", padding="5")
        velocity_frame.pack(fill=tk.X, pady=5)

        # Velocity slider
        widgets['velocity_scale'] = ttk.Scale(velocity_frame, from_=-MAX_VELOCITY_UNIT, 
            to=MAX_VELOCITY_UNIT, orient=tk.HORIZONTAL)
        widgets['velocity_scale'].pack(fill=tk.X, padx=5, pady=5)
        widgets['velocity_scale'].set(0)  # Initialize at 0
        
        # Velocity value label
        widgets['velocity_value'] = ttk.Label(velocity_frame, text="Velocity: 0")
        widgets['velocity_value'].pack(pady=2)
        
        # Update velocity when slider changes
        widgets['velocity_scale'].configure(command=lambda val, 
            sid=servo_id: self.update_velocity(sid, float(val)))

        # --- Status Display ---
        status_frame = ttk.LabelFrame(parent_frame, text="Status", padding="5")
        status_frame.pack(fill=tk.X, pady=5)
        
        widgets['current_velocity'] = ttk.Label(status_frame, text="Current Velocity: --")
        widgets['current_velocity'].pack(fill=tk.X, pady=2)
        
        widgets['current_temp'] = ttk.Label(status_frame, text="Temperature: --°C")
        widgets['current_temp'].pack(fill=tk.X, pady=2)
        
        widgets['current_load'] = ttk.Label(status_frame, text="Load: --%")
        widgets['current_load'].pack(fill=tk.X, pady=2)

        self.servo_widgets[servo_id] = widgets

    def get_speed_value(self, servo_id):
        """Gets the speed value from the slider and maps it to Dynamixel units."""
        # Use MAX_VELOCITY_UNIT from config
        percentage = self.servo_widgets[servo_id]['speed_scale'].get()
        # Map 0-100 to 0-MAX_VELOCITY_UNIT (using value from config)
        return int((percentage / 100.0) * MAX_VELOCITY_UNIT)

    def toggle_torque(self, servo_id, enable):
        """Callback for torque buttons."""
        print(f"Button: Toggle Torque for Servo {servo_id} to {enable}")
        set_torque(servo_id, enable)
        # Optionally update GUI based on success/failure

    def go_to_pos(self, servo_id):
        """Callback for Go button (Position Control)."""
        print(f"Button: Go To Position for Servo {servo_id}")
        widgets = self.servo_widgets[servo_id]
        try:
            position = int(widgets['pos_entry'].get())
            # TODO: Get position range from config or servo?
            if 0 <= position <= 4095: # Validate position range (currently hardcoded for XM/XL-430)
                 # Ensure torque is on and mode is Position Control
                 # Use MODE_POSITION_CONTROL from config
                if set_operating_mode(servo_id, MODE_POSITION_CONTROL):
                    time.sleep(0.1) # Short delay after mode change
                    # Use TORQUE_ENABLE from config
                    if set_torque(servo_id, True): # Use True instead of TORQUE_ENABLE for clarity
                         time.sleep(0.1) # Short delay after torque enable
                         # Set speed first (Profile Velocity in Position Mode)
                         speed = self.get_speed_value(servo_id) # Use slider speed
                         # Use ADDR_PROFILE_VELOCITY from config
                         print(f"Setting Servo ID {servo_id} Profile Velocity to {speed}")
                         # Acquire lock for this direct packetHandler call
                         comm_success = False
                         with dxl_lock:
                             dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, servo_id, ADDR_PROFILE_VELOCITY, speed)
                             comm_success = check_comm_result(dxl_comm_result, dxl_error)
                         # Release lock automatically

                         if comm_success:
                             set_goal_position(servo_id, position) # This function handles its own lock
                         else:
                              print(f"Failed to set profile velocity for Servo ID {servo_id}")
                    else:
                        print(f"Failed to enable torque for Servo ID {servo_id} before setting position.")
                else:
                    print(f"Failed to set position control mode for Servo ID {servo_id}.")
            else:
                print(f"Error: Position value {position} out of range (0-4095).")
        except ValueError:
            print("Error: Invalid position value entered.")
        except KeyError:
             print(f"Error: Widgets for Servo ID {servo_id} not found.")

    def toggle_random_movement(self):
        """Toggle random movement on/off."""
        global stop_event
        is_enabled = self.random_movement_enabled.get()
        
        if is_enabled:
            # Start random movement
            if not hasattr(self, 'random_thread') or not self.random_thread.is_alive():
                stop_event.clear()
                self.random_thread = threading.Thread(target=random_move_thread_func, daemon=True)
                self.random_thread.start()
                print("Random movement enabled")
        else:
            # Stop random movement
            if hasattr(self, 'random_thread') and self.random_thread.is_alive():
                stop_event.set()
                self.random_thread.join(timeout=1.0)  # Wait up to 1 second for thread to stop
                print("Random movement disabled")

    def continuous_move(self, servo_id, direction):
        """Start continuous movement in specified direction."""
        print(f"Button: Continuous Move {direction.upper()} for Servo {servo_id}")
        
        # Update movement state
        self.continuous_movement_active[servo_id] = True
        
        # Configure servo for movement
        if set_operating_mode(servo_id, MODE_VELOCITY_CONTROL):
            time.sleep(0.1)
            if set_torque(servo_id, True):
                time.sleep(0.1)
                speed_value = self.get_speed_value(servo_id)
                velocity = speed_value if direction == 'cw' else -speed_value
                velocity = max(-MAX_VELOCITY_UNIT, min(MAX_VELOCITY_UNIT, velocity))
                
                # Update velocity whenever the slider changes
                def update_velocity(event=None):
                    if self.continuous_movement_active[servo_id]:
                        new_speed = self.get_speed_value(servo_id)
                        new_velocity = new_speed if direction == 'cw' else -new_speed
                        new_velocity = max(-MAX_VELOCITY_UNIT, min(MAX_VELOCITY_UNIT, new_velocity))
                        set_goal_velocity(servo_id, new_velocity)
                
                # Bind slider to velocity updates
                self.servo_widgets[servo_id]['speed_scale'].configure(command=update_velocity)
                
                # Set initial velocity
                set_goal_velocity(servo_id, velocity)
            else:
                print(f"Failed to enable torque for Servo ID {servo_id} before continuous move.")
        else:
            print(f"Failed to set velocity control mode for Servo ID {servo_id}.")

    def stop_move(self, servo_id):
        """Stop continuous movement."""
        print(f"Button: Stop Move for Servo {servo_id}")
        self.continuous_movement_active[servo_id] = False
        # Unbind slider updates
        self.servo_widgets[servo_id]['speed_scale'].configure(command=None)
        set_goal_velocity(servo_id, 0)

    def update_velocity(self, servo_id, velocity):
        """Update the velocity of a servo."""
        velocity = int(velocity)  # Convert to integer
        self.servo_widgets[servo_id]['velocity_value'].configure(
            text=f"Velocity: {velocity}")
        set_goal_velocity(servo_id, velocity)

    def update_status_loop(self):
        """Continuously update status information for all servos."""
        while self.update_status_active:
            for servo_id in SERVO_IDS:
                widgets = self.servo_widgets[servo_id]
                
                velocity = read_present_velocity(servo_id)
                if velocity is not None:
                    widgets['current_velocity'].configure(
                        text=f"Current Velocity: {velocity}")
                
                temp = read_present_temperature(servo_id)
                if temp is not None:
                    widgets['current_temp'].configure(
                        text=f"Temperature: {temp}°C")
                
                load = read_present_load(servo_id)
                if load is not None:
                    # Convert load to percentage (assuming 2048 is 100%)
                    load_percent = (load / 2048) * 100
                    widgets['current_load'].configure(
                        text=f"Load: {load_percent:.1f}%")
            
            time.sleep(0.1)  # Update every 100ms

    def on_closing(self):
        """Clean up when the application is closing."""
        self.update_status_active = False  # Stop status updates
        stop_event.set()  # Signal all threads to stop
        
        # Disable torque and close port
        for servo_id in SERVO_IDS:
            set_goal_velocity(servo_id, 0)  # Stop all movement
            set_torque(servo_id, False)  # Disable torque
        
        if portHandler.is_open:
            portHandler.closePort()
        
        self.root.destroy()

# --- Random Movement Thread Function ---
def random_move_thread_func():
    """Function executed by the random movement thread."""
    print("Random movement thread started.")
    # Initial setup for all servos - no longer needed here as main loop handles it
    # initial_setup_done = {sid: False for sid in SERVO_IDS}

    while not stop_event.is_set():
        for servo_id in SERVO_IDS:
            if stop_event.is_set(): break # Check stop flag frequently

            try:
                # --- Set Mode and Enable Torque Safely ---
                # Functions handle locking internally now
                if not set_operating_mode(servo_id, MODE_VELOCITY_CONTROL):
                    print(f"[RandomMove] Failed to set Velocity Mode for {servo_id}. Skipping.")
                    stop_event.wait(0.1) # Avoid busy-looping on failure
                    continue
                # No need for sleep here, lock release/acquire provides separation

                if not set_torque(servo_id, True):
                    print(f"[RandomMove] Failed to enable torque for {servo_id}. Skipping.")
                    stop_event.wait(0.1)
                    continue
                # No need for sleep here

                # --- Generate and Set Velocity ---
                speed_percent = random.uniform(RANDOM_MIN_SPEED_PERCENT, RANDOM_MAX_SPEED_PERCENT)
                velocity = int((speed_percent / 100.0) * MAX_VELOCITY_UNIT)
                direction = random.choice([-1, 1])
                goal_velocity = max(-MAX_VELOCITY_UNIT, min(MAX_VELOCITY_UNIT, velocity * direction))

                print(f"[RandomMove] Setting Servo {servo_id} velocity to {goal_velocity}")
                set_goal_velocity(servo_id, goal_velocity) # Handles locking internally

            except Exception as e:
                print(f"[RandomMove] Error controlling servo {servo_id}: {e}")
                # Avoid flooding logs, maybe wait longer after an error
                stop_event.wait(0.5)

            if stop_event.is_set(): break # Check again after potentially long operation

        if stop_event.is_set(): break # Check after finishing loop for one servo_id set

        # --- Wait for Random Duration ---
        duration = random.uniform(RANDOM_MIN_DURATION_S, RANDOM_MAX_DURATION_S)
        print(f"[RandomMove] Waiting for {duration:.2f} seconds...")
        # Use event wait for duration, allowing quicker exit if stop_event is set
        stop_event.wait(duration)

    # --- Cleanup on Thread Exit ---
    print("Random movement thread stopping. Setting velocity to 0 and disabling torque.")
    for servo_id in SERVO_IDS:
        try:
            print(f"[RandomMove Cleanup] Setting velocity 0 for {servo_id}")
            set_goal_velocity(servo_id, 0) # Handles locking
            # No need for sleep here
            print(f"[RandomMove Cleanup] Disabling torque for {servo_id}")
            set_torque(servo_id, False) # Handles locking
            # No need for sleep here
        except Exception as e:
            print(f"[RandomMove] Warning: Error during cleanup for servo {servo_id}: {e}")
    print("Random movement thread finished.")

# --- Main Execution ---
if __name__ == "__main__":
    # --- Set initial mode and disable torque ---
    # Torque MUST be disabled before changing Operating Mode.
    # Use SERVO_IDS, MODE_POSITION_CONTROL, TORQUE_DISABLE from config
    for servo_id in SERVO_IDS:
        print(f"\nInitializing Servo ID: {servo_id}")
        # Disable Torque First (handles lock)
        set_torque(servo_id, False) # Use False instead of TORQUE_DISABLE
        # No sleep needed here, lock release/acquire handles separation
        # Set to Position Control Mode initially (handles lock)
        set_operating_mode(servo_id, MODE_POSITION_CONTROL)
        # No sleep needed here
        # Torque remains disabled until user enables it via GUI

    print("\nStarting GUI...")
    root = tk.Tk()
    app = DynamixelControlApp(root)

    # --- Start Random Movement Thread if Enabled ---
    app.random_thread = None # Initialize attribute
    if ENABLE_RANDOM_MOVEMENT:
        if not SERVO_IDS:
            print("Warning: Random movement enabled, but no SERVO_IDS defined in config. Skipping.")
        else:
            stop_event.clear() # Ensure flag is clear before starting
            app.random_thread = threading.Thread(target=random_move_thread_func, daemon=True)
            app.random_thread.start()
    else:
        print("Random movement disabled in configuration.")

    root.mainloop()
    print("GUI closed.")
