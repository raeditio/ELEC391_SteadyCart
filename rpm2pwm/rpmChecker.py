import cv2
import numpy as np

# Load the video
video_path = "videos/pwm255.mp4"
cap = cv2.VideoCapture(video_path)

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
duration = frame_count / fps

# Extract frames and detect tape marker
frames_with_marker = []
frame_idx = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale for better contrast
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply thresholding to highlight the black tape marker
    _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)

    # Find contours
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If a significant contour is detected, assume it's the tape
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # Adjust threshold as needed
            frames_with_marker.append(frame_idx)
            break

    frame_idx += 1

cap.release()

# Calculate RPM (Revolutions Per Minute)
if len(frames_with_marker) > 1:
    # Calculate the time between successive detections of the marker
    time_intervals = np.diff(np.array(frames_with_marker)) / fps

    # Average time per revolution
    avg_time_per_rev = np.mean(time_intervals) if len(time_intervals) > 0 else None

    # RPM calculation
    rpm = (60 / avg_time_per_rev) if avg_time_per_rev else None
else:
    rpm = None

# Print the RPM
print(f"RPM: {rpm}")
