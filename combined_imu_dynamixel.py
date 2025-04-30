import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import numpy as np
import serial.tools.list_ports
from PIL import Image, ImageTk, ImageDraw
import math
import colorsys
import yaml
import os
import sys

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
    print("Error: Failed to import dynamixel_sdk.")
    print("Please install the library: pip install dynamixel-sdk")
    sys.exit(1)

# Performance settings for IMU
REDRAW_INTERVAL = 10  # ms between redraws
DATA_HISTORY_LENGTH = 200  # Reduce history length to improve performance
QUIVER_SCALE = 30  # Scale of the direction arrow

# Load Dynamixel Configuration
CONFIG_FILE = 'config.yaml'
try:
    with open(CONFIG_FILE, 'r') as f:
        config = yaml.safe_load(f)['dynamixel_settings']
    print(f"Loaded configuration from {CONFIG_FILE}")
except FileNotFoundError:
    print(f"Error: Configuration file '{CONFIG_FILE}' not found.")
    sys.exit(1)
except Exception as e:
    print(f"Error loading config: {e}")
    sys.exit(1)

# Import Dynamixel settings from config
try:
    PROTOCOL_VERSION = float(config['PROTOCOL_VERSION'])
    BAUDRATE = int(config['BAUDRATE'])
    DXL_DEVICENAME = config['DEVICENAME']
    SERVO_IDS = config['SERVO_IDS']
    ADDR_OPERATING_MODE = int(config['ADDR_OPERATING_MODE'])
    ADDR_TORQUE_ENABLE = int(config['ADDR_TORQUE_ENABLE'])
    ADDR_GOAL_VELOCITY = int(config['ADDR_GOAL_VELOCITY'])
    ADDR_GOAL_POSITION = int(config['ADDR_GOAL_POSITION'])
    ADDR_PRESENT_POSITION = int(config['ADDR_PRESENT_POSITION'])
    MODE_VELOCITY_CONTROL = int(config['MODE_VELOCITY_CONTROL'])
    MODE_POSITION_CONTROL = int(config['MODE_POSITION_CONTROL'])
    TORQUE_ENABLE = int(config['TORQUE_ENABLE'])
    TORQUE_DISABLE = int(config['TORQUE_DISABLE'])
    MAX_VELOCITY_UNIT = int(config.get('MAX_VELOCITY_UNIT', 1023))
except Exception as e:
    print(f"Error processing config values: {e}")
    sys.exit(1)

# Constants
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"
SLIDER_COLOR = "#3498db"
SUCCESS_COLOR = "#2ecc71"
DANGER_COLOR = "#e74c3c"

# Global flags and locks
stop_event = threading.Event()
dxl_lock = threading.Lock()

# Initialize Dynamixel communication
portHandler = PortHandler(DXL_DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Helper class for IMU angle unwrapping
class AngleUnwrapper:
    def __init__(self):
        self.previous_angle = None
        self.offset = 0
        
    def unwrap(self, angle):
        if self.previous_angle is None:
            self.previous_angle = angle
            return angle
        
        diff = angle - self.previous_angle
        if diff > 180:
            self.offset -= 360
        elif diff < -180:
            self.offset += 360
            
        self.previous_angle = angle
        return angle + self.offset
    
    def reset(self):
        self.previous_angle = None
        self.offset = 0

# Kalman Filter for IMU
class KalmanFilter3D:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1000
        self.Q = np.eye(6) * process_noise
        self.R = np.eye(3) * measurement_noise
        self.F = np.eye(6)
        self.F[0:3, 3:6] = np.eye(3)
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)
        self.dt = 0.01
        
    def predict(self):
        self.state = self.F @ self.state
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        
    def update(self, measurement):
        K = self.covariance @ self.H.T @ np.linalg.inv(self.H @ self.covariance @ self.H.T + self.R)
        innovation = measurement - self.H @ self.state
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        return self.state[0:3]

# Dynamixel helper functions
def check_comm_result(dxl_comm_result, dxl_error):
    if dxl_comm_result != COMM_SUCCESS:
        print(f"%s" % packetHandler.getTxRxResult(dxl_comm_result))
        return False
    elif dxl_error != 0:
        print(f"%s" % packetHandler.getRxPacketError(dxl_error))
        return False
    return True

def set_torque(servo_id, enable):
    value = TORQUE_ENABLE if enable else TORQUE_DISABLE
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, servo_id, ADDR_TORQUE_ENABLE, value)
        if check_comm_result(dxl_comm_result, dxl_error):
            status = "enabled" if enable else "disabled"
            print(f"Torque for Servo ID {servo_id} {status}.")
            return True
        print(f"Failed to set torque for Servo ID {servo_id}.")
        return False

def set_operating_mode(servo_id, mode):
    with dxl_lock:
        dxl_comm_result_torque, dxl_error_torque = packetHandler.write1ByteTxRx(
            portHandler, servo_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if not check_comm_result(dxl_comm_result_torque, dxl_error_torque):
            print(f"Warning: Failed to disable torque for Servo ID {servo_id}")
        
        time.sleep(0.05)
        
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
            portHandler, servo_id, ADDR_OPERATING_MODE, mode)
        if check_comm_result(dxl_comm_result, dxl_error):
            mode_name = "Velocity Control" if mode == MODE_VELOCITY_CONTROL else "Position Control"
            print(f"Operating mode for Servo ID {servo_id} set to {mode_name}.")
            return True
        print(f"Failed to set operating mode for Servo ID {servo_id}.")
        return False

def set_goal_velocity(servo_id, velocity):
    print(f"Setting Servo ID {servo_id} Goal Velocity to {velocity}")
    with dxl_lock:
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
            portHandler, servo_id, ADDR_GOAL_VELOCITY, velocity)
        if not check_comm_result(dxl_comm_result, dxl_error):
            print(f"Failed to set goal velocity for Servo ID {servo_id}.")

# Main application class
class CombinedIMUDynamixelApp:
    def __init__(self, root):
        self.root = root
        root.title("IMU Visualization and Dynamixel Control")
        
        # Configure root window
        root.geometry("1200x800")  # Set initial size
        root.minsize(800, 600)     # Set minimum size
        root.configure(bg=DARK_BG)
        
        # Configure grid weights
        root.columnconfigure(0, weight=1)
        root.rowconfigure(0, weight=1)
        
        # Setup modern styles
        self.setup_styles()
        
        # Initialize IMU data
        self.x_data, self.y_data, self.z_data = [], [], []
        self.x_filtered, self.y_filtered, self.z_filtered = [], [], []
        self.kalman_filter = KalmanFilter3D()
        self.yaw_unwrapper = AngleUnwrapper()
        self.euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")
        
        # Initialize IMU serial connection
        self.imu_port = self.find_imu_port()
        if not self.imu_port:
            print("Error: IMU port not found")
            sys.exit(1)
        try:
            self.imu_serial = serial.Serial(self.imu_port, 115200, timeout=1)
            print(f"Connected to IMU on {self.imu_port}")
        except serial.SerialException as e:
            print(f"Error connecting to IMU: {e}")
            sys.exit(1)
        
        # Initialize plot update flags
        self.redraw_needed = False
        self.last_redraw_time = 0
        self.auto_resize = True
        self.continuous_yaw = True
        
        # Initialize Dynamixel data
        self.servo_widgets = {}
        self.continuous_movement_active = {}
        self.update_status_active = True
        
        # Add variables for angle display
        self.yaw_var = tk.DoubleVar(value=0.0)
        self.pitch_var = tk.DoubleVar(value=0.0)
        self.roll_var = tk.DoubleVar(value=0.0)
        
        # Add control variables
        self.auto_resize_var = tk.BooleanVar(value=True)
        self.continuous_yaw_var = tk.BooleanVar(value=True)
        
        # Setup main container
        self.setup_main_container()
        
        # Create notebook for tabs
        self.create_notebook()
        
        # Setup IMU visualization
        self.setup_imu_visualization()
        
        # Setup Dynamixel controls
        self.setup_dynamixel_controls()
        
        # Start update threads
        self.start_update_threads()
        
        # Setup closing handler
        root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def find_imu_port(self):
        """Find Arduino/IMU port automatically."""
        ports = list(serial.tools.list_ports.comports())
        for port in ports:
            if ('Arduino' in port.description or 
                'usbmodem' in port.device or 
                'FT232H' in port.description):
                return port.device
        return None

    def euler_to_vector(self, yaw, pitch, roll):
        """Convert Euler angles to a direction vector."""
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        x = math.cos(yaw_rad) * math.cos(pitch_rad)
        y = math.sin(yaw_rad) * math.cos(pitch_rad)
        z = math.sin(pitch_rad)
        
        return [x, y, z]

    def schedule_redraw(self):
        """Mark plot for redrawing."""
        self.redraw_needed = True

    def update_plot_limits(self):
        """Update plot limits based on data."""
        if not self.auto_resize or not self.x_filtered:
            return
        
        x_min, x_max = min(self.x_filtered), max(self.x_filtered)
        y_min, y_max = min(self.y_filtered), max(self.y_filtered)
        z_min, z_max = min(self.z_filtered), max(self.z_filtered)
        
        x_range = max(abs(x_min), abs(x_max)) * 1.1
        y_range = max(abs(y_min), abs(y_max)) * 1.1
        z_range = max(abs(z_min), abs(z_max)) * 1.1
        
        max_range = max(x_range, y_range, z_range, 20)
        
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_zlim(-max_range, max_range)
        
        self.schedule_redraw()

    def update_angle_display(self, yaw, pitch, roll):
        """Update the angle display with current values."""
        # Update variables
        self.yaw_var.set(f"{yaw:.1f}°")
        self.pitch_var.set(f"{pitch:.1f}°")
        self.roll_var.set(f"{roll:.1f}°")
        
        # Update progress bars (adjust for visualization)
        self.yaw_progress['value'] = (yaw + 90) % 180
        self.pitch_progress['value'] = (pitch + 90) % 180
        self.roll_progress['value'] = (roll + 90) % 180

    def update_imu(self):
        """Read and process IMU data."""
        while not stop_event.is_set():
            if self.imu_serial.in_waiting > 0:
                try:
                    line = self.imu_serial.readline().decode('utf-8', errors='replace').strip()
                    match = self.euler_regex.match(line)
                    
                    if match:
                        yaw = float(match.group(1))
                        pitch = float(match.group(2))
                        roll = float(match.group(3))
                        
                        if self.continuous_yaw:
                            yaw = self.yaw_unwrapper.unwrap(yaw)
                        
                        measurement = np.array([yaw, pitch, roll])
                        self.kalman_filter.predict()
                        filtered = self.kalman_filter.update(measurement)
                        
                        self.x_data.append(yaw)
                        self.y_data.append(pitch)
                        self.z_data.append(roll)
                        
                        self.x_filtered.append(filtered[0])
                        self.y_filtered.append(filtered[1])
                        self.z_filtered.append(filtered[2])
                        
                        # Update angle display
                        self.root.after(0, self.update_angle_display,
                            filtered[0], filtered[1], filtered[2])
                        
                        if len(self.x_data) > DATA_HISTORY_LENGTH:
                            self.x_data = self.x_data[-DATA_HISTORY_LENGTH:]
                            self.y_data = self.y_data[-DATA_HISTORY_LENGTH:]
                            self.z_data = self.z_data[-DATA_HISTORY_LENGTH:]
                            self.x_filtered = self.x_filtered[-DATA_HISTORY_LENGTH:]
                            self.y_filtered = self.y_filtered[-DATA_HISTORY_LENGTH:]
                            self.z_filtered = self.z_filtered[-DATA_HISTORY_LENGTH:]
                        
                        self.schedule_redraw()
                except Exception as e:
                    print(f"Error reading IMU data: {e}")
                    if self.imu_serial.in_waiting > 100:
                        self.imu_serial.reset_input_buffer()
            
            time.sleep(0.01)  # Small delay to prevent busy waiting

    def update_plot(self):
        """Update the plot visualization."""
        while not stop_event.is_set():
            current_time = time.time() * 1000
            
            if self.redraw_needed and (current_time - self.last_redraw_time) > REDRAW_INTERVAL:
                if len(self.x_data) > 0:
                    # Update lines
                    self.line.set_data(self.x_data, self.y_data)
                    self.line.set_3d_properties(self.z_data)
                    self.filtered_line.set_data(self.x_filtered, self.y_filtered)
                    self.filtered_line.set_3d_properties(self.z_filtered)
                    
                    # Update current position dot
                    self.dot.set_data([self.x_filtered[-1]], [self.y_filtered[-1]])
                    self.dot.set_3d_properties([self.z_filtered[-1]])
                    
                    # Update direction arrow
                    pos = np.array([[self.x_filtered[-1], self.y_filtered[-1], self.z_filtered[-1]]])
                    yaw_for_vector = self.x_filtered[-1] % 360 if self.continuous_yaw else self.x_filtered[-1]
                    direction = self.euler_to_vector(yaw_for_vector, self.y_filtered[-1], self.z_filtered[-1])
                    direction = np.array([[direction[0], direction[1], direction[2]]])
                    self.quiver.set_segments([np.concatenate((pos, pos + direction * QUIVER_SCALE))])
                    
                    # Update plot limits if needed
                    if len(self.x_data) % 10 == 0:
                        self.update_plot_limits()
                    
                    # Perform the redraw
                    self.figure_canvas.draw()
                    
                self.redraw_needed = False
                self.last_redraw_time = current_time
            
            time.sleep(0.01)  # Small delay to prevent busy waiting

    def setup_styles(self):
        """Configure ttk styles for a modern dark theme."""
        style = ttk.Style()
        style.theme_use('clam')  # Base theme
        
        # Configure colors
        style.configure('TFrame', background=DARK_BG)
        style.configure('TLabel', background=DARK_BG, foreground=TEXT_COLOR)
        style.configure('TButton',
            background=DARKER_BG,
            foreground=TEXT_COLOR,
            padding=(10, 5),
            font=('Helvetica', 9))
        style.map('TButton',
            background=[('active', HIGHLIGHT), ('disabled', DARKER_BG)],
            foreground=[('disabled', '#888888')])
        
        # Configure LabelFrame
        style.configure('TLabelframe',
            background=DARK_BG,
            foreground=TEXT_COLOR,
            padding=5)
        style.configure('TLabelframe.Label',
            background=DARK_BG,
            foreground=TEXT_COLOR,
            font=('Helvetica', 10, 'bold'))
        
        # Configure Notebook
        style.configure('TNotebook',
            background=DARK_BG,
            borderwidth=0)
        style.configure('TNotebook.Tab',
            background=DARKER_BG,
            foreground=TEXT_COLOR,
            padding=[10, 5],
            font=('Helvetica', 9))
        style.map('TNotebook.Tab',
            background=[('selected', HIGHLIGHT)],
            foreground=[('selected', TEXT_COLOR)])
        
        # Configure Canvas
        style.configure('Canvas',
            background=DARK_BG,
            borderwidth=0,
            highlightthickness=0)
        
        # Configure Scrollbar
        style.configure('Vertical.TScrollbar',
            background=DARK_BG,
            borderwidth=0,
            arrowcolor=TEXT_COLOR,
            troughcolor=DARKER_BG)
        
        # Configure Checkbutton
        style.configure('TCheckbutton',
            background=DARK_BG,
            foreground=TEXT_COLOR,
            font=('Helvetica', 9))
        style.map('TCheckbutton',
            background=[('active', DARK_BG)],
            foreground=[('disabled', '#888888')])
        
        # Configure Scale (slider)
        style.configure('Horizontal.TScale',
            background=DARK_BG,
            troughcolor=DARKER_BG,
            sliderlength=20)

    def setup_main_container(self):
        self.main_frame = ttk.Frame(self.root)
        self.main_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=10, pady=10)
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.rowconfigure(1, weight=1)
        
        # Add title bar
        title_frame = ttk.Frame(self.main_frame)
        title_frame.grid(column=0, row=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        # App title
        ttk.Label(title_frame, text="IMU Visualization and Dynamixel Control",
            font=('Helvetica', 14, 'bold')).pack(side=tk.LEFT, padx=5)
        
        # Add separator below title
        ttk.Separator(self.main_frame, orient=tk.HORIZONTAL).grid(
            column=0, row=0, sticky=(tk.W, tk.E), pady=(35, 0))

    def create_notebook(self):
        self.notebook = ttk.Notebook(self.main_frame)
        self.notebook.grid(row=1, column=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        
        # Create tabs
        self.readouts_tab = ttk.Frame(self.notebook)
        self.controls_tab = ttk.Frame(self.notebook)
        
        # Configure tab grid weights
        self.readouts_tab.columnconfigure(0, weight=1)
        self.readouts_tab.rowconfigure(0, weight=1)
        self.controls_tab.columnconfigure(0, weight=1)
        self.controls_tab.rowconfigure(0, weight=1)
        
        self.notebook.add(self.readouts_tab, text='Readouts')
        self.notebook.add(self.controls_tab, text='Controls')

    def setup_imu_visualization(self):
        # Create a frame for the visualization section
        self.viz_frame = ttk.Frame(self.readouts_tab)
        self.viz_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
        self.viz_frame.columnconfigure(0, weight=1)
        self.viz_frame.rowconfigure(1, weight=1)
        
        # Add angle displays at the top
        self.setup_angle_displays()
        
        # Create matplotlib figure
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(8, 6), facecolor=DARK_BG)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(DARKER_BG)
        
        # Create visualization elements
        self.line, = self.ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
        self.filtered_line, = self.ax.plot([], [], [], lw=2, label='Filtered Path', color=SUCCESS_COLOR)
        self.dot = self.ax.plot([], [], [], marker='o', label='Current Orientation',
                               color=ACCENT_COLOR, markersize=8)[0]
        self.quiver = self.ax.quiver([0], [0], [0], [0], [0], [1],
                                    color=DANGER_COLOR, length=QUIVER_SCALE,
                                    normalize=True, arrow_length_ratio=0.2)
        
        # Set initial plot properties
        self.ax.set_xlim(-180, 180)
        self.ax.set_ylim(-180, 180)
        self.ax.set_zlim(-180, 180)
        self.ax.set_xlabel("Yaw", color=TEXT_COLOR)
        self.ax.set_ylabel("Pitch", color=TEXT_COLOR)
        self.ax.set_zlabel("Roll", color=TEXT_COLOR)
        self.ax.tick_params(colors=TEXT_COLOR)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        
        # Embed matplotlib figure
        self.canvas_frame = ttk.Frame(self.viz_frame)
        self.canvas_frame.grid(column=0, row=1, sticky=(tk.N, tk.W, tk.E, tk.S))
        self.figure_canvas = FigureCanvasTkAgg(self.fig, master=self.canvas_frame)
        self.canvas_widget = self.figure_canvas.get_tk_widget()
        self.canvas_widget.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))

    def setup_angle_displays(self):
        """Setup the angle display widgets."""
        # Create angle displays frame
        angle_display_frame = ttk.LabelFrame(self.viz_frame, text="Angle Displays", padding="10")
        angle_display_frame.grid(column=0, row=0, sticky=(tk.W, tk.E), padx=5, pady=5)
        angle_display_frame.columnconfigure(1, weight=1)
        
        # Create colored progress bar styles
        style = ttk.Style()
        style.configure("Yaw.Horizontal.TProgressbar", background=HIGHLIGHT)
        style.configure("Pitch.Horizontal.TProgressbar", background=SUCCESS_COLOR)
        style.configure("Roll.Horizontal.TProgressbar", background=ACCENT_COLOR)
        
        # Yaw display
        ttk.Label(angle_display_frame, text="Yaw:", font=('Helvetica', 10, 'bold')).grid(
            row=0, column=0, sticky=tk.W, pady=4)
        self.yaw_progress = ttk.Progressbar(angle_display_frame, orient=tk.HORIZONTAL,
            mode='determinate', maximum=180, value=90, style="Yaw.Horizontal.TProgressbar")
        self.yaw_progress.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
        ttk.Label(angle_display_frame, textvariable=self.yaw_var,
            font=('Helvetica', 10, 'bold')).grid(row=0, column=2, sticky=tk.E, pady=4)
        
        # Pitch display
        ttk.Label(angle_display_frame, text="Pitch:", font=('Helvetica', 10, 'bold')).grid(
            row=1, column=0, sticky=tk.W, pady=4)
        self.pitch_progress = ttk.Progressbar(angle_display_frame, orient=tk.HORIZONTAL,
            mode='determinate', maximum=180, value=90, style="Pitch.Horizontal.TProgressbar")
        self.pitch_progress.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
        ttk.Label(angle_display_frame, textvariable=self.pitch_var,
            font=('Helvetica', 10, 'bold')).grid(row=1, column=2, sticky=tk.E, pady=4)
        
        # Roll display
        ttk.Label(angle_display_frame, text="Roll:", font=('Helvetica', 10, 'bold')).grid(
            row=2, column=0, sticky=tk.W, pady=4)
        self.roll_progress = ttk.Progressbar(angle_display_frame, orient=tk.HORIZONTAL,
            mode='determinate', maximum=180, value=90, style="Roll.Horizontal.TProgressbar")
        self.roll_progress.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
        ttk.Label(angle_display_frame, textvariable=self.roll_var,
            font=('Helvetica', 10, 'bold')).grid(row=2, column=2, sticky=tk.E, pady=4)

    def setup_dynamixel_controls(self):
        # Create main controls container with scrollbar
        control_container = ttk.Frame(self.controls_tab)
        control_container.pack(fill=tk.BOTH, expand=True)
        
        # Add canvas and scrollbar for scrolling
        canvas = tk.Canvas(control_container, bg=DARK_BG,
            highlightthickness=0)  # Remove border
        scrollbar = ttk.Scrollbar(control_container,
            orient="vertical", command=canvas.yview)
        scrollable_frame = ttk.Frame(canvas)
        
        # Make the scrollable frame expand to canvas width
        scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(
                scrollregion=canvas.bbox("all"),
                width=e.width))
        
        canvas.create_window((0, 0), window=scrollable_frame,
            anchor="nw", width=canvas.winfo_width())
        canvas.configure(yscrollcommand=scrollbar.set)
        
        # Configure canvas resize
        def on_canvas_configure(event):
            canvas.itemconfig(canvas.find_withtag("all")[0],
                width=event.width)
        canvas.bind("<Configure>", on_canvas_configure)
        
        # Pack scrollbar components
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        
        # Add IMU Controls
        self.setup_imu_controls(scrollable_frame)
        
        # Add separator
        ttk.Separator(scrollable_frame, orient='horizontal').pack(fill='x', pady=10)
        
        # Create frame for Dynamixel controls
        self.dynamixel_frame = ttk.LabelFrame(scrollable_frame, text="Dynamixel Controls")
        self.dynamixel_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Create controls for each servo
        for i, servo_id in enumerate(SERVO_IDS):
            frame = ttk.LabelFrame(self.dynamixel_frame, text=f"Servo ID: {servo_id}")
            frame.pack(fill=tk.X, padx=5, pady=5)
            self.create_servo_controls(frame, servo_id)
            self.continuous_movement_active[servo_id] = False
            
            # Initialize servo
            if set_operating_mode(servo_id, MODE_VELOCITY_CONTROL):
                set_torque(servo_id, True)

    def setup_imu_controls(self, parent):
        """Setup the IMU control section."""
        imu_frame = ttk.LabelFrame(parent, text="IMU Controls")
        imu_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Plot controls
        plot_frame = ttk.LabelFrame(imu_frame, text="Plot Controls", padding="5")
        plot_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Auto-resize toggle
        auto_resize_check = ttk.Checkbutton(plot_frame, text="Auto-resize plot",
            variable=self.auto_resize_var,
            command=self.toggle_auto_resize)
        auto_resize_check.pack(anchor=tk.W, pady=2)
        
        # Continuous yaw toggle
        continuous_yaw_check = ttk.Checkbutton(plot_frame, text="Continuous yaw (prevent 0/360 jumps)",
            variable=self.continuous_yaw_var,
            command=self.toggle_continuous_yaw)
        continuous_yaw_check.pack(anchor=tk.W, pady=2)
        
        # Reset plot button
        ttk.Button(plot_frame, text="Reset Plot",
            command=self.reset_plot).pack(fill=tk.X, pady=5)
        
        # IMU controls
        control_frame = ttk.LabelFrame(imu_frame, text="IMU Settings", padding="5")
        control_frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Zero IMU button
        ttk.Button(control_frame, text="Zero IMU",
            command=self.zero_imu).pack(fill=tk.X, pady=5)

    def toggle_auto_resize(self):
        """Toggle auto-resize plot feature."""
        self.auto_resize = self.auto_resize_var.get()
        if self.auto_resize:
            self.update_plot_limits()

    def toggle_continuous_yaw(self):
        """Toggle continuous yaw feature."""
        self.continuous_yaw = self.continuous_yaw_var.get()
        if not self.continuous_yaw:
            self.yaw_unwrapper.reset()

    def reset_plot(self):
        """Reset the plot and clear data."""
        self.x_data.clear()
        self.y_data.clear()
        self.z_data.clear()
        self.x_filtered.clear()
        self.y_filtered.clear()
        self.z_filtered.clear()
        self.yaw_unwrapper.reset()
        self.update_plot_limits()
        self.schedule_redraw()

    def zero_imu(self):
        """Zero the IMU."""
        try:
            self.imu_serial.write(b"ZERO\n")
            self.imu_serial.flush()
            print("Zeroing IMU")
            # Reset Kalman filter and angle unwrapper
            self.kalman_filter = KalmanFilter3D()
            self.yaw_unwrapper.reset()
            # Clear plot data
            self.reset_plot()
        except Exception as e:
            print(f"Error zeroing IMU: {e}")

    def create_servo_controls(self, parent_frame, servo_id):
        widgets = {}
        
        # Torque control
        torque_frame = ttk.Frame(parent_frame)
        torque_frame.pack(fill=tk.X, pady=5)
        widgets['torque_label'] = ttk.Label(torque_frame, text="Torque:")
        widgets['torque_label'].pack(side=tk.LEFT, padx=5)
        widgets['torque_enable_button'] = ttk.Button(torque_frame, text="Enable",
            command=lambda: self.toggle_torque(servo_id, True))
        widgets['torque_enable_button'].pack(side=tk.LEFT, padx=2)
        widgets['torque_disable_button'] = ttk.Button(torque_frame, text="Disable",
            command=lambda: self.toggle_torque(servo_id, False))
        widgets['torque_disable_button'].pack(side=tk.LEFT, padx=2)
        
        # Velocity control
        velocity_frame = ttk.LabelFrame(parent_frame, text="Velocity Control")
        velocity_frame.pack(fill=tk.X, pady=5)
        widgets['velocity_scale'] = ttk.Scale(velocity_frame, from_=-MAX_VELOCITY_UNIT,
            to=MAX_VELOCITY_UNIT, orient=tk.HORIZONTAL)
        widgets['velocity_scale'].pack(fill=tk.X, padx=5, pady=5)
        widgets['velocity_scale'].set(0)
        
        widgets['velocity_value'] = ttk.Label(velocity_frame, text="Velocity: 0")
        widgets['velocity_value'].pack(pady=2)
        
        widgets['velocity_scale'].configure(
            command=lambda val, sid=servo_id: self.update_velocity(sid, float(val)))
        
        self.servo_widgets[servo_id] = widgets

    def toggle_torque(self, servo_id, enable):
        set_torque(servo_id, enable)

    def update_velocity(self, servo_id, velocity):
        velocity = int(velocity)
        self.servo_widgets[servo_id]['velocity_value'].configure(
            text=f"Velocity: {velocity}")
        set_goal_velocity(servo_id, velocity)

    def start_update_threads(self):
        # Start IMU update thread
        self.imu_thread = threading.Thread(target=self.update_imu, daemon=True)
        self.imu_thread.start()
        
        # Start plot update thread
        self.plot_thread = threading.Thread(target=self.update_plot, daemon=True)
        self.plot_thread.start()

    def on_closing(self):
        """Clean up when the application is closing."""
        stop_event.set()
        self.update_status_active = False
        
        # Close IMU serial connection
        if hasattr(self, 'imu_serial') and self.imu_serial.is_open:
            self.imu_serial.close()
        
        # Cleanup Dynamixel
        for servo_id in SERVO_IDS:
            set_goal_velocity(servo_id, 0)
            set_torque(servo_id, False)
        
        if portHandler.is_open:
            portHandler.closePort()
        
        self.root.destroy()

if __name__ == "__main__":
    # Initialize Dynamixel port
    if not portHandler.openPort():
        print(f"Failed to open the Dynamixel port: {DXL_DEVICENAME}")
        sys.exit(1)
    
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to set the Dynamixel baudrate to {BAUDRATE}")
        sys.exit(1)
    
    # Initialize servos
    for servo_id in SERVO_IDS:
        set_torque(servo_id, False)
        set_operating_mode(servo_id, MODE_POSITION_CONTROL)
    
    # Start application
    root = tk.Tk()
    app = CombinedIMUDynamixelApp(root)
    root.mainloop() 
