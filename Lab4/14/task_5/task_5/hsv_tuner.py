import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

# Function that does nothing, needed for trackbar creation
def nothing(x):
    pass

# Get the path to the video file
# Note: Adjusted path to be more robust from the workspace root
pkg_share_path = get_package_share_directory('task_5')
video_path = os.path.join(pkg_share_path, 'resource', 'lab3_video.avi')
cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print(f"Error: Could not open video file at {video_path}")
    exit()

# Create a window to display the controls
cv2.namedWindow("Trackbars")

# Create trackbars for H, S, V lower and upper bounds
cv2.createTrackbar("L - H", "Trackbars", 0, 179, nothing) # Hue is 0-179 in OpenCV
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 179, 179, nothing)
cv2.createTrackbar("U - S", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)

# Set initial values from your detector script
cv2.setTrackbarPos("L - H", "Trackbars", 20)
cv2.setTrackbarPos("L - S", "Trackbars", 100)
cv2.setTrackbarPos("L - V", "Trackbars", 100)
cv2.setTrackbarPos("U - H", "Trackbars", 40)

# --- NEW: Add a state variable for pausing ---
is_paused = False
current_frame = None

while True:
    # --- NEW: Only read a new frame if not paused ---
    if not is_paused:
        ret, frame = cap.read()
        if not ret:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0) # Loop video
            continue
        # Store the last good frame
        current_frame = frame.copy()
    
    # Ensure there's a frame to process
    if current_frame is None:
        continue

    # All processing is done on the current_frame (which is frozen when paused)
    hsv = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)

    # Get current positions of the trackbars
    l_h = cv2.getTrackbarPos("L - H", "Trackbars")
    l_s = cv2.getTrackbarPos("L - S", "Trackbars")
    l_v = cv2.getTrackbarPos("L - V", "Trackbars")
    u_h = cv2.getTrackbarPos("U - H", "Trackbars")
    u_s = cv2.getTrackbarPos("U - S", "Trackbars")
    u_v = cv2.getTrackbarPos("U - V", "Trackbars")

    lower_bound = np.array([l_h, l_s, l_v])
    upper_bound = np.array([u_h, u_s, u_v])
    
    mask = cv2.inRange(hsv, lower_bound, upper_bound)
    
    # --- NEW: Add a visual indicator for the pause state ---
    display_frame = current_frame.copy()
    if is_paused:
        cv2.putText(display_frame, "PAUSED", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    # Show the original frame and the mask
    cv2.imshow("Original Frame", display_frame)
    cv2.imshow("Mask", mask)

    # --- NEW: Update keypress handling ---
    key = cv2.waitKey(30) & 0xFF # Wait 30ms for a keypress
    if key == ord('q'): # Press 'q' to quit
        break
    if key == ord(' '): # Press spacebar to toggle pause
        is_paused = not is_paused

cap.release()
cv2.destroyAllWindows()

print("\n--- Final HSV Values ---")
print(f"lower_bound = np.array([{l_h}, {l_s}, {l_v}])")
print(f"upper_bound = np.array([{u_h}, {u_s}, {u_v}])")