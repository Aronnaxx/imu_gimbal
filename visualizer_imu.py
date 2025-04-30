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

# Performance settings
REDRAW_INTERVAL = 10  # ms between redraws (higher = less CPU usage but less smooth)
DATA_HISTORY_LENGTH = 200  # Reduce history length to improve performance
QUIVER_SCALE = 30  # Scale of the direction arrow

# Angle unwrapping for yaw (prevents discontinuities at 0/360)
class AngleUnwrapper:
    def __init__(self):
        self.previous_angle = None
        self.offset = 0
        
    def unwrap(self, angle):
        """Unwrap angle to avoid jumps when crossing 0/360 boundary"""
        # First angle we get, just return it
        if self.previous_angle is None:
            self.previous_angle = angle
            return angle
        
        # Detect if we've crossed the discontinuity
        diff = angle - self.previous_angle
        
        # If the difference is more than 180 degrees, we've wrapped around
        if diff > 180:
            self.offset -= 360
        elif diff < -180:
            self.offset += 360
            
        # Save current angle for next comparison
        self.previous_angle = angle
        
        # Return unwrapped angle
        return angle + self.offset
    
    def reset(self):
        """Reset the unwrapper"""
        self.previous_angle = None
        self.offset = 0

# Kalman Filter implementation for 3D orientation
class KalmanFilter3D:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        # State vector: [yaw, pitch, roll, yaw_rate, pitch_rate, roll_rate]
        self.state = np.zeros(6)
        self.covariance = np.eye(6) * 1000  # Initial uncertainty
        
        # Process noise covariance
        self.Q = np.eye(6) * process_noise
        
        # Measurement noise covariance
        self.R = np.eye(3) * measurement_noise
        
        # State transition matrix (assuming constant velocity model)
        self.F = np.eye(6)
        self.F[0:3, 3:6] = np.eye(3)  # Position depends on velocity
        
        # Measurement matrix (we only measure position)
        self.H = np.zeros((3, 6))
        self.H[0:3, 0:3] = np.eye(3)
        
        # Time step (in seconds)
        self.dt = 0.01  # 10ms update rate
        
    def predict(self):
        # Predict state
        self.state = self.F @ self.state
        
        # Predict covariance
        self.covariance = self.F @ self.covariance @ self.F.T + self.Q
        
    def update(self, measurement):
        # Kalman gain
        K = self.covariance @ self.H.T @ np.linalg.inv(self.H @ self.covariance @ self.H.T + self.R)
        
        # Update state
        innovation = measurement - self.H @ self.state
        self.state = self.state + K @ innovation
        
        # Update covariance
        self.covariance = (np.eye(6) - K @ self.H) @ self.covariance
        
        # Return filtered measurement
        return self.state[0:3]

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
    
    # Configure Notebook (tabbed interface)
    style.configure('TNotebook', background=DARK_BG, borderwidth=0)
    style.configure('TNotebook.Tab', background=DARKER_BG, foreground=TEXT_COLOR, padding=[10, 5])
    style.map('TNotebook.Tab',
              background=[('selected', HIGHLIGHT)],
              foreground=[('selected', TEXT_COLOR)])

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
x_filtered, y_filtered, z_filtered = [], [], []

# Initialize Kalman filter and angle unwrapper
kalman_filter = KalmanFilter3D(process_noise=0.1, measurement_noise=1.0)
yaw_unwrapper = AngleUnwrapper()

# Regular expression to parse serial data of the form: "Euler: 45.0, -30.0, 10.0"
euler_regex = re.compile(r"Euler:\s*([\d\.-]+),\s*([\d\.-]+),\s*([\d\.-]+)")

# Auto-resize plot flag
auto_resize = True
plot_range = 180  # Initial plot range
show_controls = True  # Control panel visibility

# Flags for optimization
redraw_needed = False
last_redraw_time = 0

# Create main Tkinter window
root = tk.Tk()
root.title("IMU Orientation Visualizer")
root.geometry("1300x800")
root.configure(bg=DARK_BG)
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)

# Setup modern styles
setup_styles()

# Create main container frame
main_frame = ttk.Frame(root)
main_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=10, pady=10)
main_frame.columnconfigure(0, weight=1)
main_frame.rowconfigure(0, weight=0)  # Title bar row
main_frame.rowconfigure(1, weight=1)  # Content row

# Create a title bar with legend
title_bar = ttk.Frame(main_frame, padding="5")
title_bar.grid(column=0, row=0, sticky=(tk.W, tk.E), pady=(0, 10))
title_bar.columnconfigure(0, weight=1)

# Add title and legend to title bar
title_frame = ttk.Frame(title_bar)
title_frame.pack(fill=tk.X, expand=True)

# App title on the left
ttk.Label(title_frame, text="IMU Orientation Visualizer", 
         font=('Helvetica', 14, 'bold')).pack(side=tk.LEFT, padx=5)

# Legend items on the right
legend_items = [
    {"label": "Orientation Path", "color": HIGHLIGHT},
    {"label": "Filtered Path", "color": SUCCESS_COLOR},
    {"label": "Current Position", "color": ACCENT_COLOR},
    {"label": "Direction Arrow", "color": DANGER_COLOR}
]

# Create a frame for legend items
legend_frame_top = ttk.Frame(title_frame)
legend_frame_top.pack(side=tk.RIGHT, padx=5)

# Add legend items horizontally
for i, item in enumerate(legend_items):
    frame = ttk.Frame(legend_frame_top)
    frame.pack(side=tk.LEFT, padx=10)
    
    # Color box
    canvas = tk.Canvas(frame, width=16, height=16, bg=DARK_BG, highlightthickness=0)
    canvas.create_rectangle(2, 2, 14, 14, fill=item["color"], outline="")
    canvas.pack(side=tk.LEFT, padx=2)
    
    # Label
    ttk.Label(frame, text=item["label"], font=('Helvetica', 9)).pack(side=tk.LEFT, padx=2)

# Add separator below title bar
ttk.Separator(main_frame, orient=tk.HORIZONTAL).grid(column=0, row=0, sticky=(tk.W, tk.E), pady=(35, 0))

# Create paned window for resizable panels in the content area
paned_window = ttk.PanedWindow(main_frame, orient=tk.HORIZONTAL)
paned_window.grid(column=0, row=1, sticky=(tk.N, tk.W, tk.E, tk.S))

# Left panel for plot
plot_frame = ttk.Frame(paned_window)
plot_frame.columnconfigure(0, weight=1)
plot_frame.rowconfigure(0, weight=1)

# Right panel for controls using notebook (tabs)
control_frame = ttk.Frame(paned_window)
control_frame.columnconfigure(0, weight=1)
control_frame.rowconfigure(0, weight=1)

# Add frames to paned window
paned_window.add(plot_frame, weight=3)
paned_window.add(control_frame, weight=1)

# Create matplotlib figure with dark theme
plt.style.use('dark_background')
fig = plt.figure(figsize=(8, 6), facecolor=DARK_BG)
ax = fig.add_subplot(111, projection='3d')
ax.set_facecolor(DARKER_BG)

# Create path lines for visualization - initialize with empty data
line, = ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
filtered_line, = ax.plot([], [], [], lw=2, label='Filtered Path', color=SUCCESS_COLOR)
dot = ax.plot([], [], [], marker='o', label='Current Orientation', color=ACCENT_COLOR, markersize=8)[0]

# Create arrow for direction visualization - will be updated in place
quiver = ax.quiver([0], [0], [0], [0], [0], [1], color=DANGER_COLOR, 
                  length=QUIVER_SCALE, normalize=True, arrow_length_ratio=0.2)

# Set initial axis limits
ax.set_xlim(-plot_range, plot_range)
ax.set_ylim(-plot_range, plot_range)
ax.set_zlim(-plot_range, plot_range)
ax.set_xlabel("Yaw", color=TEXT_COLOR)
ax.set_ylabel("Pitch", color=TEXT_COLOR)
ax.set_zlabel("Roll", color=TEXT_COLOR)
ax.tick_params(colors=TEXT_COLOR)
ax.grid(True, linestyle='--', alpha=0.3)

# Embed matplotlib figure in Tkinter
canvas_frame = ttk.Frame(plot_frame)
canvas_frame.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))
canvas_frame.columnconfigure(0, weight=1)
canvas_frame.rowconfigure(0, weight=1)

# Create the matplotlib canvas
figure_canvas = FigureCanvasTkAgg(fig, master=canvas_frame)
figure_canvas.draw()
canvas_widget = figure_canvas.get_tk_widget()
canvas_widget.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S))

# Create notebook (tabbed interface) for controls
notebook = ttk.Notebook(control_frame)
notebook.grid(column=0, row=0, sticky=(tk.N, tk.W, tk.E, tk.S), padx=5, pady=5)
notebook.columnconfigure(0, weight=1)
notebook.rowconfigure(0, weight=1)

# Create tabs - rename Legend tab to Readouts since it's focused on data display now
controls_tab = ttk.Frame(notebook)
readouts_tab = ttk.Frame(notebook)
notebook.add(readouts_tab, text='Readouts')
notebook.add(controls_tab, text='Controls')
# Select Readouts tab by default
notebook.select(0)

# Make the controls tab expand properly
controls_tab.columnconfigure(0, weight=1)
controls_tab.rowconfigure(0, weight=1)

# Ensure paned window initially divides space correctly (controls take 1/3)
def configure_paned_window(event=None):
    total_width = paned_window.winfo_width()
    if total_width > 10:  # Only adjust when there's enough width
        if show_controls:
            # Set control panel to take up 1/3 of the screen
            paned_window.sashpos(0, int(total_width * 2/3))
            
            # Force update of the readouts tab to ensure proper sizing
            readouts_frame.update_idletasks()
            # Update the XYZ arrows visualization
            if hasattr(xyz_arrows, '_last_yaw'):
                xyz_arrows.update_arrows(xyz_arrows._last_yaw, xyz_arrows._last_pitch, xyz_arrows._last_roll)

# Bind to configure event to ensure proper sizing
paned_window.bind("<Configure>", configure_paned_window)

# Toggle control panel visibility
def toggle_controls():
    global show_controls
    show_controls = not show_controls
    if show_controls:
        paned_window.add(control_frame, weight=1)
        toggle_btn.config(text="Hide Controls")
        # Force equal division
        root.after(100, configure_paned_window)
    else:
        paned_window.forget(control_frame)
        toggle_btn.config(text="Show Controls")

# Add toggle button to plot frame
toggle_btn = ttk.Button(plot_frame, text="Hide Controls", command=toggle_controls)
toggle_btn.grid(column=0, row=1, sticky=tk.SE, padx=5, pady=5)

# Add performance options
performance_frame = ttk.LabelFrame(controls_tab, text="Performance", padding="10")
performance_frame.pack(fill=tk.X, pady=10)
performance_frame.columnconfigure(0, weight=1)

# Redraw interval slider
redraw_var = tk.IntVar(value=REDRAW_INTERVAL)
ttk.Label(performance_frame, text="Redraw Interval (ms):").pack(anchor=tk.W)
redraw_slider = ttk.Scale(performance_frame, from_=10, to=500, variable=redraw_var, orient=tk.HORIZONTAL)
redraw_slider.pack(fill=tk.X, pady=5)
redraw_value_label = ttk.Label(performance_frame, textvariable=redraw_var)
redraw_value_label.pack(anchor=tk.E)

# Controls tab content
controls_tab.columnconfigure(0, weight=1)

# Plot control frame
plot_frame_controls = ttk.LabelFrame(controls_tab, text="Plot Controls", padding="10")
plot_frame_controls.pack(fill=tk.X, pady=10)
plot_frame_controls.columnconfigure(0, weight=1)

# Auto-resize toggle with better styling
auto_resize_var = tk.BooleanVar(value=auto_resize)
auto_resize_check = ttk.Checkbutton(plot_frame_controls, text="Auto-resize plot", 
                                   variable=auto_resize_var)
auto_resize_check.pack(anchor=tk.W, pady=5)

# Continuous yaw toggle
continuous_yaw_var = tk.BooleanVar(value=True)
continuous_yaw_check = ttk.Checkbutton(plot_frame_controls, text="Continuous yaw (prevent 0/360 jumps)", 
                                      variable=continuous_yaw_var)
continuous_yaw_check.pack(anchor=tk.W, pady=5)

# Reset plot button
def reset_plot():
    global x_data, y_data, z_data, x_filtered, y_filtered, z_filtered
    x_data.clear()
    y_data.clear()
    z_data.clear()
    x_filtered.clear()
    y_filtered.clear()
    z_filtered.clear()
    # Reset angle unwrapper
    yaw_unwrapper.reset()
    update_plot_limits()
    schedule_redraw()

ttk.Button(plot_frame_controls, text="Reset Plot", command=reset_plot).pack(anchor=tk.W, pady=5, fill=tk.X)

# IMU control frame
imu_frame = ttk.LabelFrame(controls_tab, text="IMU Controls", padding="10")
imu_frame.pack(fill=tk.X, pady=10)
imu_frame.columnconfigure(0, weight=1)

# Zero IMU button with better styling
def zero_imu():
    ser.write(b"ZERO\n")
    ser.flush()
    print("Zeroing IMU")
    # Reset Kalman filter
    global kalman_filter, yaw_unwrapper
    kalman_filter = KalmanFilter3D(process_noise=0.1, measurement_noise=1.0)
    yaw_unwrapper.reset()

ttk.Button(imu_frame, text="Zero IMU", command=zero_imu).pack(fill=tk.X, pady=5)

# Status frame with better visualization
status_frame = ttk.LabelFrame(controls_tab, text="Status", padding="10")
status_frame.pack(fill=tk.X, pady=10)
status_frame.columnconfigure(0, weight=1)

# Variables for each angle
yaw_var = tk.DoubleVar(value=0.0)
pitch_var = tk.DoubleVar(value=0.0)
roll_var = tk.DoubleVar(value=0.0)

# Custom legend in the Legend tab (removed since moved to title bar)
readouts_frame = ttk.Frame(readouts_tab, padding=10)
readouts_frame.pack(fill=tk.BOTH, expand=True)
readouts_frame.columnconfigure(0, weight=1)
readouts_frame.rowconfigure(0, weight=1)

# Create angle displays frame in the readouts tab
angle_display_frame = ttk.LabelFrame(readouts_frame, text="Angle Displays", padding="10")
angle_display_frame.pack(fill=tk.X, pady=10)
angle_display_frame.columnconfigure(0, weight=1)

# Create better angle displays
angle_display = ttk.Frame(angle_display_frame)
angle_display.pack(fill=tk.X, pady=5, expand=True)
angle_display.columnconfigure(1, weight=1)

# Create colored progress bar style for each angle
style = ttk.Style()
style.configure("Yaw.Horizontal.TProgressbar", background=HIGHLIGHT)
style.configure("Pitch.Horizontal.TProgressbar", background=SUCCESS_COLOR)
style.configure("Roll.Horizontal.TProgressbar", background=ACCENT_COLOR)

# Create a function to update font sizes based on window size
def update_angle_display_fonts(event=None):
    # Get the current width of the angle display frame
    width = angle_display.winfo_width()
    if width < 10:  # If width is not yet available, use a default
        width = 300
    
    # Calculate font size based on width (scale with window size)
    base_font_size = max(8, int(width / 40))
    label_font = ('Helvetica', base_font_size, 'bold')
    value_font = ('Helvetica', base_font_size, 'bold')
    
    # Update font for all labels
    yaw_label.configure(font=label_font)
    pitch_label.configure(font=label_font)
    roll_label.configure(font=label_font)
    
    # Update font for all value labels
    yaw_value.configure(font=value_font)
    pitch_value.configure(font=value_font)
    roll_value.configure(font=value_font)
    
    # Update padding based on width
    pad_x = max(4, int(width / 30))
    pad_y = max(2, int(width / 100))
    
    # Update padding for all widgets
    yaw_progress.grid(padx=pad_x, pady=pad_y)
    pitch_progress.grid(padx=pad_x, pady=pad_y)
    roll_progress.grid(padx=pad_x, pady=pad_y)
    
    yaw_label.grid(pady=pad_y)
    pitch_label.grid(pady=pad_y)
    roll_label.grid(pady=pad_y)
    
    yaw_value.grid(pady=pad_y)
    pitch_value.grid(pady=pad_y)
    roll_value.grid(pady=pad_y)

# Yaw display
yaw_label = ttk.Label(angle_display, text="Yaw:", font=('Helvetica', 10, 'bold'))
yaw_label.grid(row=0, column=0, sticky=tk.W, pady=4)
yaw_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                              maximum=180, value=90, style="Yaw.Horizontal.TProgressbar")
yaw_progress.grid(row=0, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
yaw_value = ttk.Label(angle_display, textvariable=yaw_var, font=('Helvetica', 10, 'bold'))
yaw_value.grid(row=0, column=2, sticky=tk.E, pady=4)

# Pitch display
pitch_label = ttk.Label(angle_display, text="Pitch:", font=('Helvetica', 10, 'bold'))
pitch_label.grid(row=1, column=0, sticky=tk.W, pady=4)
pitch_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                               maximum=180, value=90, style="Pitch.Horizontal.TProgressbar")
pitch_progress.grid(row=1, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
pitch_value = ttk.Label(angle_display, textvariable=pitch_var, font=('Helvetica', 10, 'bold'))
pitch_value.grid(row=1, column=2, sticky=tk.E, pady=4)

# Roll display
roll_label = ttk.Label(angle_display, text="Roll:", font=('Helvetica', 10, 'bold'))
roll_label.grid(row=2, column=0, sticky=tk.W, pady=4)
roll_progress = ttk.Progressbar(angle_display, orient=tk.HORIZONTAL, mode='determinate',
                              maximum=180, value=90, style="Roll.Horizontal.TProgressbar")
roll_progress.grid(row=2, column=1, sticky=(tk.W, tk.E), padx=8, pady=4)
roll_value = ttk.Label(angle_display, textvariable=roll_var, font=('Helvetica', 10, 'bold'))
roll_value.grid(row=2, column=2, sticky=tk.E, pady=4)

# Bind resize event to update fonts
angle_display.bind('<Configure>', update_angle_display_fonts)
angle_display_frame.bind('<Configure>', update_angle_display_fonts)

# Create a separator between angle bars and IMU visualization
ttk.Separator(readouts_frame, orient=tk.HORIZONTAL).pack(fill=tk.X, pady=15)

# Create XYZ Arrow visualization class
class XYZArrows(tk.Canvas):
    def __init__(self, parent, size=100, bg=DARK_BG, fg=TEXT_COLOR, highlightthickness=0):
        # Calculate size based on parent dimensions for better high-res display support
        parent_width = parent.winfo_width()
        parent_height = parent.winfo_height()
        
        # For high-res displays, use a larger minimum size
        # This ensures the widget is visible on 4K displays
        min_size = 300  # Minimum size for high-res displays
        
        # Use the larger of the calculated size or minimum size
        self.size = max(size, min_size)
        
        # If parent has valid dimensions, use them to calculate a better size
        if parent_width > 10 and parent_height > 10:
            # Use 80% of the smaller dimension, but not less than min_size
            self.size = max(min(parent_width, parent_height) * 0.8, min_size)
        
        super().__init__(parent, width=self.size, height=self.size, bg=bg, highlightthickness=highlightthickness)
        
        self.center_x = self.size // 2
        self.center_y = self.size // 2
        self.arrow_length = self.size // 3
        
        # Create background circle for better visibility
        self.create_oval(
            self.center_x - self.arrow_length - 15,
            self.center_y - self.arrow_length - 15,
            self.center_x + self.arrow_length + 15,
            self.center_y + self.arrow_length + 15,
            fill=DARKER_BG, outline=TEXT_COLOR, width=1
        )
        
        # Create initial arrows
        self.x_arrow = self.create_line(0, 0, 0, 0, fill='red', width=4, arrow=tk.LAST)
        self.y_arrow = self.create_line(0, 0, 0, 0, fill='green', width=4, arrow=tk.LAST)
        self.z_arrow = self.create_line(0, 0, 0, 0, fill='blue', width=4, arrow=tk.LAST)
        
        # Create labels with larger, bold font - scale font size with widget size
        font_size = max(14, int(self.size / 15))  # Scale font size with widget size
        self.create_text(self.center_x + self.arrow_length + 12, self.center_y, 
                        text="X", fill='red', font=('Helvetica', font_size, 'bold'))
        self.create_text(self.center_x, self.center_y - self.arrow_length - 12, 
                        text="Y", fill='green', font=('Helvetica', font_size, 'bold'))
        self.create_text(self.center_x + 12, self.center_y + 12, 
                        text="Z", fill='blue', font=('Helvetica', font_size, 'bold'))
        
        # Create a small legend in the corner with better spacing
        legend_x = 15
        legend_y = self.size - 80  # Move up to avoid overlap
        legend_font_size = max(10, int(self.size / 22))  # Scale legend font size
        legend_spacing = max(20, int(self.size / 20))  # Scale spacing with size
        
        self.create_text(legend_x, legend_y, text="X: Roll", fill='red', 
                        font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
        self.create_text(legend_x, legend_y + legend_spacing, text="Y: Pitch", fill='green', 
                        font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
        self.create_text(legend_x, legend_y + legend_spacing * 2, text="Z: Yaw", fill='blue', 
                        font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
        
        self.update_arrows(0, 0, 0)
        
        # Bind resize event to update the visualization
        self.bind('<Configure>', self.on_resize)
        
    def on_resize(self, event):
        """Handle resize events to update the visualization"""
        # Only update if the size has changed significantly
        if abs(event.width - self.size) > 10 or abs(event.height - self.size) > 10:
            # Ensure minimum size for high-res displays
            self.size = max(min(event.width, event.height), 300)
            self.center_x = event.width // 2
            self.center_y = event.height // 2
            self.arrow_length = self.size // 3
            
            # Clear and redraw
            self.delete("all")
            
            # Redraw background circle
            self.create_oval(
                self.center_x - self.arrow_length - 15,
                self.center_y - self.arrow_length - 15,
                self.center_x + self.arrow_length + 15,
                self.center_y + self.arrow_length + 15,
                fill=DARKER_BG, outline=TEXT_COLOR, width=1
            )
            
            # Redraw arrows with scaled width
            arrow_width = max(4, int(self.size / 55))  # Scale arrow width with size
            self.x_arrow = self.create_line(0, 0, 0, 0, fill='red', width=arrow_width, arrow=tk.LAST)
            self.y_arrow = self.create_line(0, 0, 0, 0, fill='green', width=arrow_width, arrow=tk.LAST)
            self.z_arrow = self.create_line(0, 0, 0, 0, fill='blue', width=arrow_width, arrow=tk.LAST)
            
            # Redraw labels with scaled font size
            font_size = max(14, int(self.size / 15))
            self.create_text(self.center_x + self.arrow_length + 12, self.center_y, 
                            text="X", fill='red', font=('Helvetica', font_size, 'bold'))
            self.create_text(self.center_x, self.center_y - self.arrow_length - 12, 
                            text="Y", fill='green', font=('Helvetica', font_size, 'bold'))
            self.create_text(self.center_x + 12, self.center_y + 12, 
                            text="Z", fill='blue', font=('Helvetica', font_size, 'bold'))
            
            # Redraw legend with scaled font size and spacing
            legend_x = 15
            legend_y = self.size - 80  # Move up to avoid overlap
            legend_font_size = max(10, int(self.size / 22))
            legend_spacing = max(20, int(self.size / 20))  # Scale spacing with size
            
            self.create_text(legend_x, legend_y, text="X: Roll", fill='red', 
                            font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
            self.create_text(legend_x, legend_y + legend_spacing, text="Y: Pitch", fill='green', 
                            font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
            self.create_text(legend_x, legend_y + legend_spacing * 2, text="Z: Yaw", fill='blue', 
                            font=('Helvetica', legend_font_size, 'bold'), anchor=tk.W)
            
            # Update arrows with current values
            if hasattr(self, '_last_yaw'):
                self.update_arrows(self._last_yaw, self._last_pitch, self._last_roll)
        
    def update_arrows(self, yaw, pitch, roll):
        """Update arrow positions based on IMU orientation"""
        # Store last values for resize handling
        self._last_yaw = yaw
        self._last_pitch = pitch
        self._last_roll = roll
        
        # Convert angles to radians
        yaw_rad = math.radians(yaw)
        pitch_rad = math.radians(pitch)
        roll_rad = math.radians(roll)
        
        # Create rotation matrices
        def rot_z(angle):  # yaw
            c = math.cos(angle)
            s = math.sin(angle)
            return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            
        def rot_y(angle):  # pitch
            c = math.cos(angle)
            s = math.sin(angle)
            return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
            
        def rot_x(angle):  # roll
            c = math.cos(angle)
            s = math.sin(angle)
            return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
        
        # Combined rotation matrix
        R = rot_z(yaw_rad) @ rot_y(pitch_rad) @ rot_x(roll_rad)
        
        # Base vectors
        x_base = np.array([1, 0, 0]) * self.arrow_length
        y_base = np.array([0, 1, 0]) * self.arrow_length
        z_base = np.array([0, 0, 1]) * self.arrow_length
        
        # Rotate vectors
        x_rot = R @ x_base
        y_rot = R @ y_base
        z_rot = R @ z_base
        
        # Update arrows (project 3D to 2D)
        # X arrow (red)
        self.coords(self.x_arrow,
                   self.center_x, self.center_y,
                   self.center_x + x_rot[0], self.center_y - x_rot[1])
        
        # Y arrow (green)
        self.coords(self.y_arrow,
                   self.center_x, self.center_y,
                   self.center_x + y_rot[0], self.center_y - y_rot[1])
        
        # Z arrow (blue)
        self.coords(self.z_arrow,
                   self.center_x, self.center_y,
                   self.center_x + z_rot[0], self.center_y - z_rot[1])

# Create XYZ arrows visualization with flexible resizing
arrows_frame = ttk.LabelFrame(readouts_frame, text="IMU Orientation", padding="10")
arrows_frame.pack(fill=tk.BOTH, expand=True, pady=10)
arrows_frame.columnconfigure(0, weight=1)
arrows_frame.rowconfigure(0, weight=1)

# Set a minimum height for the arrows frame to ensure it's visible on high-res displays
arrows_frame.update_idletasks()
min_height = 400  # Minimum height for high-res displays
if arrows_frame.winfo_height() < min_height:
    arrows_frame.configure(height=min_height)

arrows_container = ttk.Frame(arrows_frame)
arrows_container.pack(fill=tk.BOTH, expand=True, pady=5)
arrows_container.columnconfigure(0, weight=1)
arrows_container.rowconfigure(0, weight=1)

# Create a responsive XYZ arrows widget that will resize with the window
xyz_arrows = XYZArrows(arrows_container, size=300)  # Increased initial size for high-res displays
xyz_arrows.pack(fill=tk.BOTH, expand=True, pady=10)

# Make the readouts tab expand properly
readouts_tab.columnconfigure(0, weight=1)
readouts_tab.rowconfigure(0, weight=1)

# Function to update arrows frame size on window resize
def update_arrows_frame_size(event=None):
    # Get the current width of the readouts frame
    width = readouts_frame.winfo_width()
    if width < 10:  # If width is not yet available, use a default
        width = 300
    
    # Calculate minimum height based on width
    min_height = max(400, int(width * 0.8))
    
    # Update the arrows frame height if needed
    if arrows_frame.winfo_height() < min_height:
        arrows_frame.configure(height=min_height)
    
    # Force update of the arrows visualization
    if hasattr(xyz_arrows, '_last_yaw'):
        xyz_arrows.update_arrows(xyz_arrows._last_yaw, xyz_arrows._last_pitch, xyz_arrows._last_roll)

# Bind resize event to update arrows frame size
readouts_frame.bind('<Configure>', update_arrows_frame_size)

# Update angle display function without gauge references
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
    
    # Update XYZ arrows
    xyz_arrows.update_arrows(yaw, pitch, roll)

# Function to update plot limits based on data
def update_plot_limits():
    if not auto_resize_var.get() or not x_data:
        return
    
    # Calculate needed range with some padding using filtered data
    x_min, x_max = min(x_filtered), max(x_filtered)
    y_min, y_max = min(y_filtered), max(y_filtered)
    z_min, z_max = min(z_filtered), max(z_filtered)
    
    # Add 10% padding
    x_range = max(abs(x_min), abs(x_max)) * 1.1
    y_range = max(abs(y_min), abs(y_max)) * 1.1
    z_range = max(abs(z_min), abs(z_max)) * 1.1
    
    # Use the largest range for all axes to maintain aspect ratio
    max_range = max(x_range, y_range, z_range, 20)  # Minimum range of 20 degrees
    
    ax.set_xlim(-max_range, max_range)
    ax.set_ylim(-max_range, max_range)
    ax.set_zlim(-max_range, max_range)
    
    # Mark for redraw
    schedule_redraw()

# Function to convert Euler angles to direction vector
def euler_to_vector(yaw, pitch, roll):
    """Convert Euler angles to a direction vector."""
    # Convert angles from degrees to radians
    yaw_rad = math.radians(yaw)
    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)
    
    # Calculate direction vector (basic implementation)
    # This assumes yaw is rotation around Z, pitch around Y, roll around X
    x = math.cos(yaw_rad) * math.cos(pitch_rad)
    y = math.sin(yaw_rad) * math.cos(pitch_rad)
    z = math.sin(pitch_rad)
    
    return [x, y, z]

# Throttle redraws for better performance
def schedule_redraw():
    global redraw_needed
    redraw_needed = True

# Actual redraw function that runs periodically
def redraw_if_needed():
    global redraw_needed, last_redraw_time
    current_time = time.time() * 1000  # current time in ms
    
    if redraw_needed and (current_time - last_redraw_time) > redraw_var.get():
        figure_canvas.draw()
        redraw_needed = False
        last_redraw_time = current_time
    
    # Schedule next check
    root.after(10, redraw_if_needed)

# Function to update the plot
def update_plot():
    global x_data, y_data, z_data, x_filtered, y_filtered, z_filtered
    
    # Read all available data from the serial port
    data_updated = False
    
    while ser.in_waiting > 0:
        try:
            line_raw = ser.readline().decode('utf-8', errors='replace').strip()
            
            match = euler_regex.match(line_raw)
            
            if match:
                yaw = float(match.group(1))
                pitch = float(match.group(2))
                roll = float(match.group(3))
                
                # Apply angle unwrapping if enabled
                if continuous_yaw_var.get():
                    yaw = yaw_unwrapper.unwrap(yaw)
                
                # Apply Kalman filter
                measurement = np.array([yaw, pitch, roll])
                kalman_filter.predict()
                filtered = kalman_filter.update(measurement)
                
                # Store raw data
                x_data.append(yaw)
                y_data.append(pitch)
                z_data.append(roll)
                
                # Store filtered data
                x_filtered.append(filtered[0])
                y_filtered.append(filtered[1])
                z_filtered.append(filtered[2])
                
                # Update visual angle displays with filtered values
                # For display, convert back to standard 0-360 range
                display_yaw = filtered[0]
                if not continuous_yaw_var.get():
                    display_yaw = display_yaw % 360
                update_angle_display(display_yaw, filtered[1], filtered[2])
                
                # Limit history to reduce memory and processing
                if len(x_data) > DATA_HISTORY_LENGTH:
                    x_data = x_data[-DATA_HISTORY_LENGTH:]
                    y_data = y_data[-DATA_HISTORY_LENGTH:]
                    z_data = z_data[-DATA_HISTORY_LENGTH:]
                    x_filtered = x_filtered[-DATA_HISTORY_LENGTH:]
                    y_filtered = y_filtered[-DATA_HISTORY_LENGTH:]
                    z_filtered = z_filtered[-DATA_HISTORY_LENGTH:]
                
                data_updated = True
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
    
    # Update visualization if data changed
    if data_updated and len(x_data) > 0:
        # Update the plotted lines
        line.set_data(x_data, y_data)
        line.set_3d_properties(z_data)
        filtered_line.set_data(x_filtered, y_filtered)
        filtered_line.set_3d_properties(z_filtered)
        
        # Update the current position dot
        dot.set_data([x_filtered[-1]], [y_filtered[-1]])
        dot.set_3d_properties([z_filtered[-1]])
        
        # Update the direction arrow (more efficiently)
        if len(x_filtered) > 0:
            # Get current position
            pos = np.array([[x_filtered[-1], y_filtered[-1], z_filtered[-1]]])
            
            # For direction vector, use modular angles (0-360) for correct vector calculation
            # but keep the arrow at the unwrapped position
            yaw_for_vector = x_filtered[-1]
            if continuous_yaw_var.get():
                yaw_for_vector = yaw_for_vector % 360
            
            # Calculate direction vector
            direction = euler_to_vector(yaw_for_vector, y_filtered[-1], z_filtered[-1])
            direction = np.array([[direction[0], direction[1], direction[2]]])
            
            # Update quiver directly without recreating
            quiver.set_segments([np.concatenate((pos, pos + direction * QUIVER_SCALE))])
        
        # Update plot limits if auto-resize is enabled
        if len(x_data) > 1 and len(x_data) % 10 == 0:  # Only check every 10 points
            update_plot_limits()
        
        # Schedule a redraw (actual redraw happens in redraw_if_needed)
        schedule_redraw()
    
    # Schedule the next update
    root.after(10, update_plot)

# Start the update and redraw processes
root.after(10, update_plot)
root.after(10, redraw_if_needed)

# Call configure_paned_window after a delay to ensure proper initial sizing
root.after(100, configure_paned_window)

# Add window resize event handler
def on_window_resize(event):
    # Only process if it's a window resize event
    if event.widget == root:
        # Force update of the readouts tab to ensure proper sizing
        readouts_frame.update_idletasks()
        
        # Update the angle display fonts
        update_angle_display_fonts()
        
        # Update the arrows frame size
        update_arrows_frame_size()
        
        # Update the XYZ arrows visualization
        if hasattr(xyz_arrows, '_last_yaw'):
            xyz_arrows.update_arrows(xyz_arrows._last_yaw, xyz_arrows._last_pitch, xyz_arrows._last_roll)
        
        # Update the paned window
        configure_paned_window()
        
        # Force a redraw of the matplotlib figure to ensure it scales properly
        figure_canvas.draw()

# Bind the window resize event
root.bind('<Configure>', on_window_resize)

# Run the Tkinter main loop
root.mainloop()

# Clean up when the window is closed
ser.close()
print("Serial connection closed.")
