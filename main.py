# main.py
import cv2
import numpy as np
import robot_driver # Your robot control functions
import hand_tracker # Your hand tracking functions

# Import the functions you created in calibration.py
from calibration import load_transform, transform_cam_to_robot

# --- Constants ---
Z_DRAW_AIR = 10 # Your chosen air-drawing height
# Get frame dimensions from your hand tracker setup
FRAME_WIDTH, FRAME_HEIGHT = 640, 480

# Load the transformation matrix calculated by calibration.py
try:
    transformation_matrix = load_transform()
    print("Transformation matrix loaded successfully.")
except FileNotFoundError:
    print("ERROR: Calibration matrix file (e.g., affine_matrix.npy) not found!")
    print("Please run the calibration script first.")
    exit() # Stop if calibration hasn't been done

# --- Initialize Robot, Camera, etc. ---
#robot_driver.go_home()

# main.py (continued)

while True:
    # 1. Get Hand Landmark Coordinates (Normalized Camera Space 0.0-1.0)
    success, cam_x_norm, cam_y_norm = hand_tracker.get_fingertip_coords() # Adapt your tracker

    if success:
        # 2. Transform to Robot Coordinates using the loaded matrix
        robot_x, robot_y = transform_cam_to_robot(
            cam_x_norm,
            cam_y_norm,
            transformation_matrix, # Use the loaded matrix
            FRAME_WIDTH,
            FRAME_HEIGHT
        )

        # --- Apply Smoothing Filter if needed ---
        # smoothed_robot_x, smoothed_robot_y = apply_smoothing(robot_x, robot_y)

        # 3. Command the Robot
        robot_driver.move_to_point(robot_x, robot_y, Z_DRAW_AIR)
        # Or use smoothed coordinates:
        # robot_driver.move_to_point(smoothed_robot_x, smoothed_robot_y, Z_DRAW_AIR)

    # --- OpenCV display code, handle exit key ---
    # key = cv2.waitKey(1) & 0xFF
    # if key == ord('q'):
    #    break

# --- Cleanup ---
# robot_driver.go_home()
# ... close camera ...