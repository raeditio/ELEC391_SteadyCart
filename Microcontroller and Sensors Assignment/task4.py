import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# Initialize serial communication
serial_port = 'COM3'  # Replace with your port
baud_rate = 9600
ser = serial.Serial(serial_port, baud_rate)

window_size = 100
accel_data = deque([0] * window_size, maxlen=window_size)
gyro_data = deque([0] * window_size, maxlen=window_size)
comp_data = deque([0] * window_size, maxlen=window_size)

def update(frame):
    line = ser.readline().decode().strip()
    try:
        accel, gyro, comp = map(float, line.split(","))
        accel_data.append(accel)
        gyro_data.append(gyro)
        comp_data.append(comp)

        ax.clear()
        ax.plot(accel_data, label="Accelerometer Angle")
        ax.plot(gyro_data, label="Gyroscope Angle")
        ax.plot(comp_data, label="Complementary Filter Angle")
        ax.legend()
        ax.set_ylim(-90, 90)
        ax.set_title("Real-Time Angle Plotting")
    except ValueError:
        pass

# Set up the plot
fig, ax = plt.subplots()
ani = animation.FuncAnimation(fig, update, interval=50)
plt.show()
