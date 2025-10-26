from lerobot.processor.robot.inverse_kinematics import InverseKinematicsEEToJoints
from lerobot.processor import RobotProcessorPipeline
from lerobot.processor.converters import robot_action_to_transition, transition_to_robot_action
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

# 1. Robot Config
follower_config = SO101FollowerConfig(
    port="/dev/ttyACM0",
    id="my_robot_arm",
    max_relative_target=5.0
) 

# 1. Create your kinematics solver from the URDF
kinematics_solver = {
    "urdf_path": "so101_new_calib.urdf",
    "target_frame_name": "gripper_frame_link",
}
robot = SO101Follower(follower_config)
robot.connect()
# Get current joint positions in degrees
current_joints = [robot.get_joint(j) for j in robot.joint_names]

# Convert to radians if your FK solver expects radians
import numpy as np
current_joints_rad = np.deg2rad(current_joints)

# Use the kinematics solver to compute EE pose
from lerobot.processor.robot.inverse_kinematics import InverseKinematicsEEToJoints

ik_solver = InverseKinematicsEEToJoints(kinematics=kinematics_solver, motor_names=robot.joint_names)

# Many IK classes also have a forward kinematics method; if not:
ee_pose = ik_solver.kinematics.forward(current_joints_rad)

print("EE Position (XYZ):", ee_pose[:3, 3])  # extract translation from 4x4 pose matrix

# 2. Build the processor pipeline
robot_ee_to_joints_processor = RobotProcessorPipeline(
    steps=[
        InverseKinematicsEEToJoints(
            kinematics=kinematics_solver,
            motor_names=list(robot.bus.motors.keys()),
            initial_guess_current_joints=True,
        ),
    ],
    to_transition=robot_action_to_transition,
    to_output=transition_to_robot_action,
)

for joint, val in joint_action.items():
    if "joint" in joint:
        joint_action[joint] = max(min(val, robot.joint_limits[joint][1]), robot.joint_limits[joint][0])

ee_target = {"ee.pos": [0.20, 0.0, 0.10]}  # target end-effector XYZ
joint_action = robot_ee_to_joints_processor(ee_target)
# Add gripper control
current_gripper_target = 2.0 
joint_action["gripper.pos"] = current_gripper_target

robot.send_action(joint_action)