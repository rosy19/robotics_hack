# Import necessary classes
from lerobot.robots.so101_follower import SO101FollowerEndEffectorConfig, SO101FollowerEndEffector
import numpy as np
import time


# --- 1. Define Target Poses (The "Instructions") ---

# A Pose is a dictionary containing position and orientation (quaternion).
# For the SO-101, a safe default orientation (gripper facing down) is common.
# Example: [x, y, z] in meters, and [qx, qy, qz, qw] for orientation.
# NOTE: Position is relative to the arm's base. You may need to adjust these values
# based on your arm's workspace.

DEFAULT_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0]) # Example: Gripper pointing straight down

target_poses = [
    # Waypoint 1: Neutral Position (e.g., hovering over the base)
    {
        "motor_position": np.array([0.15, 0.0, 0.15]), 
        "motor_orientation": DEFAULT_ORIENTATION,
        "gripper_action": 1.0 # Closed gripper
    },
    # Waypoint 2: Right side (e.g., pick-up point)
    {
        "motor_position": np.array([0.25, -0.10, 0.05]), 
        "motor_orientation": DEFAULT_ORIENTATION,
        "gripper_action": 0.0 # Open gripper
    },
    # Waypoint 3: Center (e.g., drop-off point)
    {
        "motor_position": np.array([0.25, 0.10, 0.15]), 
        "motor_orientation": DEFAULT_ORIENTATION,
        "gripper_action": 1.0 # Closed gripper
    }
]

# --- 2. Configuration and Initialization ---

# Use the IK-enabled configuration for the Follower Arm
follower_config = SO101FollowerEndEffectorConfig(
    port="/dev/ttyACM0",
    id="my_robot_arm",
)

try:
    robot = SO101FollowerEndEffector(follower_config) 
    robot.connect()
    print("Follower Arm connected. Starting pre-programmed IK movement.")

    # --- 3. Execute the Waypoints ---
    for i, pose in enumerate(target_poses):
        print(f"\nMoving to Waypoint {i+1}...")
        
        # The 'action' must be a dictionary with specific keys for the IK solver
        action = {
            "position": pose["motor_position"],       # Target 3D position
            "orientation": pose["motor_orientation"], # Target orientation
            "gripper_action": pose["gripper_action"], # Target gripper state (0.0=Open, 1.0=Closed)
        }
        
        # Send the action to the robot. The IK solver calculates the joint angles.
        robot.send_action(action)
        
        # Pause to let the robot reach the target and execute the gripper action
        time.sleep(3.0) 
        
    print("\nSequence complete.")

except Exception as e:
    print(f"An error occurred: {e}")
    print("Ensure the arm is powered, connected to the correct port, and calibrated.")