import asyncio
from bleak import BleakClient, BleakScanner
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
from collections import deque

# Custom theme colors
DARK_BG = "#2E2E2E"
DARKER_BG = "#252525"
HIGHLIGHT = "#3498db"
TEXT_COLOR = "#FFFFFF"
ACCENT_COLOR = "#F39C12"

# BLE UUIDs
IMU_SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214"
IMU_DATA_CHAR_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
SERVO_DATA_CHAR_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"
CONTROL_CHAR_UUID = "19B10003-E8F2-537E-4F6C-D104768A1214"

class IMUVisualizer:
    def __init__(self):
        self.client = None
        self.connected = False
        self.x_data = deque(maxlen=200)
        self.y_data = deque(maxlen=200)
        self.z_data = deque(maxlen=200)
        self.servo_positions = [90, 90, 90]
        
        # Create the main window
        self.root = tk.Tk()
        self.root.title("IMU Gimbal Control - BLE")
        self.root.geometry("1300x800")
        self.root.configure(bg=DARK_BG)
        
        # Create the figure and 3D axis
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(10, 8), facecolor=DARK_BG)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(DARKER_BG)
        
        # Create the line and dot for visualization
        self.line, = self.ax.plot([], [], [], lw=2, label='Orientation Path', color=HIGHLIGHT)
        self.dot = self.ax.plot([], [], [], marker='o', label='Current Orientation', 
                               color=ACCENT_COLOR, markersize=8)[0]
        
        # Set initial axis limits
        self.plot_range = 180
        self.ax.set_xlim(-self.plot_range, self.plot_range)
        self.ax.set_ylim(-self.plot_range, self.plot_range)
        self.ax.set_zlim(-self.plot_range, self.plot_range)
        
        # Configure the plot appearance
        self.ax.set_title("3D Orientation Trace (Yaw, Pitch, Roll)", color=TEXT_COLOR, fontsize=14)
        self.ax.set_xlabel("Yaw", color=TEXT_COLOR)
        self.ax.set_ylabel("Pitch", color=TEXT_COLOR)
        self.ax.set_zlabel("Roll", color=TEXT_COLOR)
        self.ax.tick_params(colors=TEXT_COLOR)
        self.ax.grid(True, linestyle='--', alpha=0.3)
        self.ax.legend(facecolor=DARKER_BG, edgecolor=HIGHLIGHT)
        
        # Create the canvas
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Create control panel
        self.create_control_panel()
        
        # Start the BLE connection thread
        self.ble_thread = threading.Thread(target=self.run_ble_client)
        self.ble_thread.daemon = True
        self.ble_thread.start()
        
        # Start the animation
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=True)
    
    def create_control_panel(self):
        control_frame = ttk.Frame(self.root, padding="10")
        control_frame.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Connection status
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(control_frame, textvariable=self.status_var).pack(pady=5)
        
        # Mode selection
        mode_frame = ttk.LabelFrame(control_frame, text="Mode", padding="10")
        mode_frame.pack(fill=tk.X, pady=5)
        
        self.mode_var = tk.StringVar(value="RANDOM")
        ttk.Radiobutton(mode_frame, text="Random", variable=self.mode_var, 
                       value="RANDOM").pack(anchor=tk.W)
        ttk.Radiobutton(mode_frame, text="Control", variable=self.mode_var, 
                       value="CONTROL").pack(anchor=tk.W)
        
        # Servo controls
        servo_frame = ttk.LabelFrame(control_frame, text="Servo Control", padding="10")
        servo_frame.pack(fill=tk.X, pady=5)
        
        for i in range(3):
            servo_control = ttk.Frame(servo_frame)
            servo_control.pack(fill=tk.X, pady=2)
            ttk.Label(servo_control, text=f"Servo {i+1}:").pack(side=tk.LEFT)
            ttk.Scale(servo_control, from_=0, to=180, orient=tk.HORIZONTAL,
                     command=lambda v, idx=i: self.update_servo(idx, int(float(v)))).pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        # Speed control
        speed_frame = ttk.LabelFrame(control_frame, text="Speed", padding="10")
        speed_frame.pack(fill=tk.X, pady=5)
        
        self.speed_var = tk.DoubleVar(value=2.0)
        ttk.Scale(speed_frame, from_=0.5, to=10, variable=self.speed_var,
                 orient=tk.HORIZONTAL, command=self.update_speed).pack(fill=tk.X)
    
    async def connect_to_device(self):
        devices = await BleakScanner.discover()
        for device in devices:
            if device.name == "IMU_Gimbal":
                self.client = BleakClient(device.address)
                await self.client.connect()
                self.connected = True
                self.status_var.set("Connected")
                return True
        return False
    
    async def handle_notifications(self, sender, data):
        if sender == IMU_DATA_CHAR_UUID:
            # Parse IMU data
            values = data.decode().split(',')
            if len(values) == 3:
                self.x_data.append(float(values[0]))
                self.y_data.append(float(values[1]))
                self.z_data.append(float(values[2]))
        elif sender == SERVO_DATA_CHAR_UUID:
            # Parse servo data
            values = data.decode().split(',')
            if len(values) == 3:
                self.servo_positions = [int(v) for v in values]
    
    async def send_command(self, command):
        if self.connected and self.client:
            await self.client.write_gatt_char(CONTROL_CHAR_UUID, command.encode())
    
    def update_servo(self, index, value):
        if self.connected:
            command = f"CONTROL,{self.servo_positions[0]},{self.servo_positions[1]},{self.servo_positions[2]}"
            asyncio.run(self.send_command(command))
    
    def update_speed(self, value):
        if self.connected:
            command = f"SPEED,{value}"
            asyncio.run(self.send_command(command))
    
    def update_plot(self, frame):
        if len(self.x_data) > 0:
            self.line.set_data(self.x_data, self.y_data)
            self.line.set_3d_properties(self.z_data)
            self.dot.set_data([self.x_data[-1]], [self.y_data[-1]])
            self.dot.set_3d_properties([self.z_data[-1]])
            
            # Update plot limits
            if len(self.x_data) > 1:
                x_min, x_max = min(self.x_data), max(self.x_data)
                y_min, y_max = min(self.y_data), max(self.y_data)
                z_min, z_max = min(self.z_data), max(self.z_data)
                
                x_range = max(abs(x_min), abs(x_max)) * 1.1
                y_range = max(abs(y_min), abs(y_max)) * 1.1
                z_range = max(abs(z_min), abs(z_max)) * 1.1
                
                max_range = max(x_range, y_range, z_range, 20)
                
                self.ax.set_xlim(-max_range, max_range)
                self.ax.set_ylim(-max_range, max_range)
                self.ax.set_zlim(-max_range, max_range)
        
        return self.line, self.dot
    
    async def run_ble_client_async(self):
        while True:
            if not self.connected:
                if await self.connect_to_device():
                    # Set up notifications
                    await self.client.start_notify(IMU_DATA_CHAR_UUID, self.handle_notifications)
                    await self.client.start_notify(SERVO_DATA_CHAR_UUID, self.handle_notifications)
                    
                    # Send initial mode
                    await self.send_command(self.mode_var.get())
            else:
                try:
                    if not self.client.is_connected:
                        self.connected = False
                        self.status_var.set("Disconnected")
                except Exception as e:
                    print(f"Error checking connection: {e}")
                    self.connected = False
                    self.status_var.set("Disconnected")
            
            await asyncio.sleep(1)
    
    def run_ble_client(self):
        asyncio.run(self.run_ble_client_async())
    
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    visualizer = IMUVisualizer()
    visualizer.run() 
