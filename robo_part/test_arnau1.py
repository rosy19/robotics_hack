# --- 1. CORE IMPORTS ---
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower
from lerobot.model.kinematics import RobotKinematics 
import numpy as np
import time
import os
# We need to manually define the 4x4 transform for the IK call
from scipy.spatial.transform import Rotation

# --- 2. CONFIGURATION DATA ---
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
URDF_PATH = "/home/sofia/SO101/so101_new_calib.urdf" 

# Target Pose (A safe, reachable position)
TARGET_POSITION = np.array([0.20, 0.15, 0.25]) # X=0.20m, Y=0.15m, Z=0.25m
DEFAULT_ORIENTATION_TWIST = np.array([0.0, 0.0, 0.0]) # RPY (Roll, Pitch, Yaw in radians)
TARGET_GRIPPER_STATE = 100.0 # Gripper closed

# --- 3. HELPER FUNCTION (To convert RPY to 4x4 matrix) ---

def pose_to_matrix(position: np.ndarray, rpy_rad: np.ndarray) -> np.ndarray:
    """Converts a position and RPY (radians) to a 4x4 transformation matrix."""
    # Create rotation matrix from RPY (Roll, Pitch, Yaw)
    R = Rotation.from_euler('xyz', rpy_rad, degrees=False).as_matrix()
    
    # Create 4x4 matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = position
    return T


# --- 4. INITIALIZE KINEMATICS & ROBOT ---

# 4.1 Initialize the Kinematics Model (This is where placo is imported)
kinematics_model = RobotKinematics(
    urdf_path=URDF_PATH, 
    joint_names=MOTOR_NAMES,
    target_frame_name="gripper_frame_link" 
)

# 4.2 Initialize the standard SO101FollowerConfig
follower_config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    id="my_robot_arm",
    use_degrees=True, # Kinematics output is in degrees
)

try:
    print(f"Bypassing processor. Calling kinematics_model.inverse_kinematics directly.")
    print("Initializing SO-101 Follower Arm...")
    
    # 4.3 Instantiate and Connect
    robot = SO101Follower(follower_config) 
    
    # CRITICAL: NO ACTION PROCESSORS ARE USED
    robot.action_processors = [] 
    
    robot.connect()
    print("Arm connected. Executing move command.")

    # --- 5. COMMAND (Execute IK and Send Joint Angles) ---
    
    print(f"Commanding arm to position: {TARGET_POSITION} meters.")

    # Step 1: Create the 4x4 target pose matrix
    # The twist is RPY in radians (0, 0, 0) for the default orientation
    target_pose_matrix = pose_to_matrix(
        TARGET_POSITION,
        DEFAULT_ORIENTATION_TWIST 
    )
    
    # Step 2: Define a neutral/initial guess position (required by placo IK)
    # 0 for all 6 motors (including gripper which the IK function preserves)
    current_joint_pos_deg = np.array([0.0] * len(MOTOR_NAMES))
    
    # Step 3: Calculate the required joint angles using the built-in method
    calculated_joint_angles_deg = kinematics_model.inverse_kinematics(
        current_joint_pos=current_joint_pos_deg,
        desired_ee_pose=target_pose_matrix,
        position_weight=1.0,
        orientation_weight=0.0 # Only constrain position
    )
    
    # Step 4: Convert the array output to the required dictionary action format
    # The IK function returns an array where:
    # First N joints are the calculated angles
    # Last joint (gripper) is the initial guess (TARGET_GRIPPER_STATE)
    action = {}
    for i, name in enumerate(MOTOR_NAMES):
        action[name] = calculated_joint_angles_deg[i]
        
    # Manually ensure the gripper position is the target state, not the initial guess
    action["gripper"] = float(TARGET_GRIPPER_STATE)
    
    # Step 5: Send the calculated joint angles
    robot.send_action(action)
    
    time.sleep(4.0) 
        
    print("Movement complete.")

except Exception as e:
    # If this fails, it means placo failed to initialize entirely.
    print(f"\nAn error occurred: {e}")
    print("Final check on placo failure. If this persists, the placo C++ dependency is permanently broken.")

finally:
    print("Program terminated.")