import serial
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# ============================================================================
#  User Parameters
# ============================================================================
SERIAL_PORT = "COM3"      # Update to your Arduino port, e.g. "COM3" on Windows
BAUD_RATE = 2000000        # Ensure it matches your Arduino Serial.begin() rate
MAX_POINTS = 100          # Number of samples to keep in the plot (history length)
REFRESH_MS = 50           # Plot refresh rate in milliseconds

Y_MIN, Y_MAX = -200, 200      # Since all values remain between -1 and 1

# ============================================================================
#  Open Serial Port
# ============================================================================
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    time.sleep(2)  # A short delay to let the Arduino reset
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
except serial.SerialException as e:
    print(f"Could not open port {SERIAL_PORT}: {e}")
    ser = None

# ============================================================================
#  Data Buffers: We'll store the last MAX_POINTS for each axis
# ============================================================================
x_vals = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)  # History for X sensor
y_vals = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)  # History for Y sensor
z_vals = deque([0.0]*MAX_POINTS, maxlen=MAX_POINTS)  # History for Z sensor
index_vals = deque(range(MAX_POINTS), maxlen=MAX_POINTS)  # x-axis indices 0..MAX_POINTS-1

# ============================================================================
#  Setup Matplotlib Figure
# ============================================================================
fig, ax = plt.subplots()

# Create line objects for each axis
line_x, = ax.plot(index_vals, x_vals, label="X", color='red')
line_y, = ax.plot(index_vals, y_vals, label="Y", color='green')
line_z, = ax.plot(index_vals, z_vals, label="Z", color='blue')

# Format the axes
ax.set_xlim(0, MAX_POINTS - 1)  # x-axis from 0 to MAX_POINTS-1
ax.set_ylim(Y_MIN, Y_MAX)       # y-axis from -1 to +1
ax.set_xlabel("Samples")
ax.set_ylabel("Value")
ax.set_title("Real-Time X, Y, Z Sensor Data")
ax.legend(loc="upper right")

# ============================================================================
#  Animation Update Function
# ============================================================================
def update(frame):
    while ser.in_waiting > 0:
        line_bytes = ser.readline().strip()
        if line_bytes:
            try:
                # Decode the incoming string, e.g. "0.12 -0.50 0.77"
                line_str = line_bytes.decode("utf-8")
                parts = line_str.split()
                print(parts)
                
                # Expecting three floats: x_val, y_val, z_val
                if len(parts) >= 3:
                    x_val = float(parts[0])
                    y_val = float(parts[1])
                    z_val = float(parts[2])
                    
                    # Append these new values to our deques
                    x_vals.append(x_val)
                    y_vals.append(y_val)
                    z_vals.append(z_val)
                    
                    # We also want our x-axis to keep incrementing, so let's do:
                    # take the last index_val + 1, or if empty, start at 0
                    if len(index_vals) == 0:
                        next_index = 0
                    else:
                        next_index = index_vals[-1] 
                    index_vals.append(next_index)
                    
                    # Now update the line data
                    line_x.set_data(range(len(x_vals)), x_vals)
                    line_y.set_data(range(len(y_vals)), y_vals)
                    line_z.set_data(range(len(z_vals)), z_vals)
                    
                    # Adjust the x-axis to show the current range of indices
                    # ax.set_xlim(index_vals[0], index_vals[-1])
                    
            except ValueError:
                # If float conversion fails, ignore the line
                pass
    
    return line_x, line_y, line_z

# ============================================================================
#  Create the Animation
# ============================================================================
ani = animation.FuncAnimation(
    fig=fig,
    func=update,
    interval=REFRESH_MS,
    blit=False,
    cache_frame_data=False
)

plt.show()

# ============================================================================
#  Cleanup: Close the serial port on exit
# ============================================================================
if ser is not None:
    ser.close()