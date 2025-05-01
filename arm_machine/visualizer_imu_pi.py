import sys
import time
from typing import Optional, Tuple

# Import visualization components
from visualizer_imu import (
    tk, ttk, plt, FuncAnimation, Axes3D, FigureCanvasTkAgg,
    threading, np, Image, ImageTk, ImageDraw, math, colorsys,
    AngleUnwrapper, KalmanFilter3D, DynamixelControlApp
)

# Import BNO055 interface
from bno055_imu import BNO055_IMU

class IMUVisualizer:
    def __init__(self, root: tk.Tk):
        """Initialize the IMU visualizer with direct BNO055 connection"""
        # Initialize IMU
        try:
            self.imu = BNO055_IMU()
        except Exception as e:
            print(f"Failed to initialize BNO055: {e}")
            sys.exit(1)

        # Initialize visualization (reuse existing DynamixelControlApp)
        self.app = DynamixelControlApp(root)
        
        # Override the update_plot function to use our IMU
        def update_plot():
            data_updated = False
            
            # Read from IMU
            euler = self.imu.read_euler()
            if euler:
                yaw, pitch, roll = euler
                
                # Use the existing visualization logic
                if self.app.continuous_yaw_var.get():
                    yaw = self.app.yaw_unwrapper.unwrap(yaw)
                
                # Apply Kalman filter
                measurement = np.array([yaw, pitch, roll])
                self.app.kalman_filter.predict()
                filtered = self.app.kalman_filter.update(measurement)
                
                # Update data arrays
                self.app.x_data.append(yaw)
                self.app.y_data.append(pitch)
                self.app.z_data.append(roll)
                
                self.app.x_filtered.append(filtered[0])
                self.app.y_filtered.append(filtered[1])
                self.app.z_filtered.append(filtered[2])
                
                # Limit history
                if len(self.app.x_data) > self.app.DATA_HISTORY_LENGTH:
                    self.app.x_data = self.app.x_data[-self.app.DATA_HISTORY_LENGTH:]
                    self.app.y_data = self.app.y_data[-self.app.DATA_HISTORY_LENGTH:]
                    self.app.z_data = self.app.z_data[-self.app.DATA_HISTORY_LENGTH:]
                    self.app.x_filtered = self.app.x_filtered[-self.app.DATA_HISTORY_LENGTH:]
                    self.app.y_filtered = self.app.y_filtered[-self.app.DATA_HISTORY_LENGTH:]
                    self.app.z_filtered = self.app.z_filtered[-self.app.DATA_HISTORY_LENGTH:]
                
                data_updated = True
                
                # Update displays
                display_yaw = filtered[0]
                if not self.app.continuous_yaw_var.get():
                    display_yaw = display_yaw % 360
                self.app.update_angle_display(display_yaw, filtered[1], filtered[2])

                # Update calibration status if available
                cal_status = self.imu.get_calibration_status()
                if cal_status:
                    sys, gyro, accel, mag = cal_status
                    print(f"Calibration - Sys: {sys}/3, Gyro: {gyro}/3, Accel: {accel}/3, Mag: {mag}/3")
            
            # Update visualization if data changed
            if data_updated and len(self.app.x_data) > 0:
                self.app.update_visualization()
            
            # Schedule next update
            root.after(10, update_plot)
        
        # Override the zero_imu function
        def zero_imu_override():
            if self.imu.zero_imu():
                # Reset Kalman filter and unwrapper
                self.app.kalman_filter = KalmanFilter3D(process_noise=0.1, measurement_noise=1.0)
                self.app.yaw_unwrapper.reset()
        
        # Replace the original functions
        self.app.update_plot = update_plot
        self.app.zero_imu = zero_imu_override
        
        # Start the update process
        root.after(10, update_plot)

    def cleanup(self):
        """Clean up IMU resources"""
        if hasattr(self, 'imu'):
            self.imu.close()
            print("IMU connection closed.")

def main():
    """Main function to run the visualizer"""
    root = tk.Tk()
    visualizer = IMUVisualizer(root)
    
    try:
        root.mainloop()
    finally:
        visualizer.cleanup()

if __name__ == "__main__":
    main() 
