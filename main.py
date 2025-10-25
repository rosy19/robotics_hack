# main.py
import cv2
import numpy as np
import robot_driver  # Your robot control functions
from hand_tracker import HandTracker
from calibration import load_transform, transform_cam_to_robot
import paho.mqtt.client as mqtt

# --- MQTT CONFIG ---
MQTT_BROKER = "10.148.40.253"  
MQTT_PORT = 1883
MQTT_BASE_TOPIC = "robot"

# --- Connect MQTT ---
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)

# --- Constants ---
Z_DRAW_AIR = 0.03 

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
    r_x_list = []
    r_y_list = []
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

            r_x_list.append(robot_x)
            r_y_list.append(robot_y)

            if len(r_x_list) == 5:
                robot_x = np.mean(r_x_list)
                robot_y = np.mean(r_y_list)
                r_x_list = []
                r_y_list = []
            else:
                continue


            # Print the target coordinates
            #print(f"PINCH -> Target Robot Coords: X={robot_x:.4f} m, Y={robot_y:.4f} m, Z={Z_DRAW_AIR:.4f} m")
            # Publish robot_x and robot_y to MQTT
            mqtt_client.publish(f"robot/pre/x", float(robot_x))
            mqtt_client.publish(f"robot/pre/y", float(robot_y))
            #print(f"Published to MQTT: {MQTT_BASE_TOPIC}/x = {robot_x:.4f}, {MQTT_BASE_TOPIC}/y = {robot_y:.4f}")

            print("robot_x", robot_x)
            print("robot_y", robot_y)
            print("cam_x", cam_x_norm)
            print("cam_y", cam_y_norm)

            joint_angles = robot_driver.calculate_inverse_kinematics(robot_x, robot_y, Z_DRAW_AIR)

            robot_driver.move_to_point(robot_x, robot_y, Z_DRAW_AIR)
            

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