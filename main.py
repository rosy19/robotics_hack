# main.py
import cv2
import numpy as np
import robot_driver  # Your robot control functions
from hand_tracker import HandTracker
from calibration import load_transform, transform_cam_to_robot

# --- Constants ---
Z_DRAW_AIR = 0.03 # Your chosen air-drawing height
# Get frame dimensions from your hand tracker setup
FRAME_WIDTH, FRAME_HEIGHT = 1920, 1080

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

# Initialize hand tracker
tracker = HandTracker()
if not tracker.start_camera():
    print("Error: Could not initialize camera!")
    exit()

try:
    while True:
        # Get hand position and visualization
        cam_x_norm, cam_y_norm, is_pinching, image = tracker.get_hand_position()
        
        if cam_x_norm is not None and is_pinching:
            # Transform to Robot Coordinates using the loaded matrix
            robot_x, robot_y = transform_cam_to_robot(
                cam_x_norm,
                cam_y_norm,
                transformation_matrix,
                FRAME_WIDTH,
                FRAME_HEIGHT
            )

            # Command the Robot
            #robot_driver.move_to_point(robot_x, robot_y, Z_DRAW_AIR)
            #print(f"Moving to Robot Coords: X={robot_x:.3f} m, Y={robot_y:.3f} m, Z={Z_DRAW_AIR:.3f} m")
# --- TEST SECTION ---
            # Print the target coordinates
            print(f"PINCH -> Target Robot Coords: X={robot_x:.4f} m, Y={robot_y:.4f} m, Z={Z_DRAW_AIR:.4f} m")

            # 2. Calculate Joint Angles using IK function
            joint_angles = robot_driver.calculate_inverse_kinematics(robot_x, robot_y, Z_DRAW_AIR)

            if joint_angles is not None:
                # Format angles for printing
                angle_str = ", ".join([f"{angle:.2f}" for angle in joint_angles])
                print(f"       -> Calculated Angles (deg): [{angle_str}]")
            else:
                print("       -> IK solution not found for this point (likely unreachable).")


            # !!! KEEP ACTUAL ROBOT MOVEMENT COMMENTED OUT !!!
            # robot_driver.move_to_point(robot_x_m, robot_y_m, Z_DRAW_AIR)
            # --- END TEST SECTION ---

        # Display camera feed
        if image is not None:
            display_text = "Not Tracking"
            if cam_x_norm is not None:
                display_text = "Hand Detected"
                if is_pinching:
                     display_text = f"PINCHING -> Target: X={robot_x:.2f} Y={robot_y:.2f}"
                     if joint_angles is None:
                         display_text += " (UNREACHABLE)"

            cv2.putText(image, display_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.imshow("Robot Hand Control - ANGLE TEST MODE", image)

        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

finally:
    # --- Cleanup ---
    tracker.stop()
    cv2.destroyAllWindows()
    robot_driver.go_home()