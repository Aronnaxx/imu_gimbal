import tkinter as tk
from adafruit_servokit import ServoKit

# Set up the PCA9685 board
kit = ServoKit(channels=16)

# Servo config
servo_channels = [0, 1, 2]  # Adjust as needed
servo_home_angles = {
    0: 90,  # Yaw
    1: 90,  # Pitch
    2: 90   # Roll or whatever
}

# Create GUI
root = tk.Tk()
root.title("Servo Zeroing GUI")

sliders = {}

def set_angle(ch, angle):
    kit.servo[ch].angle = angle

def home_servos():
    for ch in servo_channels:
        angle = servo_home_angles.get(ch, 90)
        sliders[ch].set(angle)
        set_angle(ch, angle)

for ch in servo_channels:
    label = tk.Label(root, text=f"Servo {ch}")
    label.pack()
    
    slider = tk.Scale(
        root, from_=0, to=180, orient=tk.HORIZONTAL,
        command=lambda val, c=ch: set_angle(c, float(val))
    )
    slider.set(servo_home_angles.get(ch, 90))
    slider.pack()
    
    sliders[ch] = slider

home_button = tk.Button(root, text="Home All Servos", command=home_servos)
home_button.pack(pady=10)

root.mainloop()
