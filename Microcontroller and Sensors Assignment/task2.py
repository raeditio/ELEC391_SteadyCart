import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Serial port configuration
SERIAL_PORT = "COM3"
BAUD_RATE = 9600

# Data storage
MAX_POINTS = 100
angle_data = deque([0] * MAX_POINTS, maxlen=MAX_POINTS)  # Store angle values

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Could not open port {SERIAL_PORT}: {e}")
    exit()

# Set up the plot
fig, ax = plt.subplots()
line, = ax.plot(angle_data, label="Angle (degrees)")
ax.set_ylim(0, 180)  # Angle range from 0 to 180 degrees
ax.set_title("Real-Time Accelerometer Angle Data")
ax.set_xlabel("Samples")
ax.set_ylabel("Angle (degrees)")
ax.legend()

# Update function for animation
def update(frame):
    if ser.in_waiting > 0:  # Check if data is available
        try:
            line_str = ser.readline().decode("utf-8").strip()  # Read a line from serial
            parts = line_str.split('\t')
            if len(parts) == 4:  # Ensure the line contains x, y, z, and angle
                angle = float(parts[3])  # Extract the angle
                angle_data.append(angle)  # Add the angle to the deque
                line.set_ydata(angle_data)  # Update the line plot
        except ValueError:
            pass

    return line,

# Create the animation
ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.show()

# Close the serial port on exit
ser.close()