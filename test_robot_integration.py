import robot_driver
from calibration import load_transform, transform_cam_to_robot
from hand_tracker import HandTracker
import numpy as np

# Test robot_driver.move_to_point
print("Testing robot_driver.move_to_point...")
robot_driver.move_to_point(0.10,0.10, 0.10)  # Example coordinates (adjust as needed)

# Test robot_driver.calculate_inverse_kinematics
print("Testing robot_driver.calculate_inverse_kinematics...")
angles = robot_driver.calculate_inverse_kinematics(0.10, 0.10, 0.10)
print(f"Calculated joint angles: {angles}")

# Test calibration transform
print("Testing calibration transform...")
try:
    matrix = load_transform()
    rx, ry = transform_cam_to_robot(0.5, 0.5, matrix, 1920, 1080)
    print(f"Transformed (0.5, 0.5) to robot coords: ({rx}, {ry})")
except Exception as e:
    print(f"Calibration test failed: {e}")

# Test hand tracker instantiation (camera not started)
print("Testing HandTracker instantiation...")
try:
    tracker = HandTracker()
    print("HandTracker created successfully.")
except Exception as e:
    print(f"HandTracker creation failed: {e}")
