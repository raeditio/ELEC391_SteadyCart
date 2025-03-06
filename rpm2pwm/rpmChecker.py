import cv2
import numpy as np

# Load video
cap = cv2.VideoCapture("rpm.mp4")
fps = cap.get(cv2.CAP_PROP_FPS)  # Get frame rate

# Define color ranges (tune based on lighting)
lower_black = np.array([0, 0, 0])  # Black marker
upper_black = np.array([180, 255, 40])

frame_count = 0
marker_positions = []
rotations = 0
last_marker_y = None

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Convert frame to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Mask for black color (tape marker)
    mask = cv2.inRange(hsv, lower_black, upper_black)
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        # Get largest detected marker
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])  # X position
            cy = int(M["m01"] / M["m00"])  # Y position

            # Track marker movement
            marker_positions.append(cy)

            # Detect full rotations based on vertical position changes
            if last_marker_y is not None and abs(cy - last_marker_y) > 50:
                rotations += 1  # Count full rotations when marker moves significantly

            last_marker_y = cy  # Update last position

# Calculate RPM
video_duration = frame_count / fps
rpm = (rotations / video_duration) * 60

print(f"Estimated RPM: {rpm:.2f}")

cap.release()
cv2.destroyAllWindows()