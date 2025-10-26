import os
import time
import threading
import numpy as np
import paho.mqtt.client as mqtt
from lerobot.robots.so101_follower import SO101Follower, SO101FollowerConfig
from lerobot.model.kinematics import RobotKinematics
from lerobot.motors import MotorsBus


# ==============================
# --- Global Configuration ---
# ==============================

URDF_PATH = os.path.expanduser("~/SO101/so101_new_calib.urdf")
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MOVE_STEPS = 2
MOVE_DELAY = 0.01  # seconds between steps
Z_MIN = 0.04  # minimal Z to avoid collisions


# ==============================
# --- Globals ---
# ==============================
robot = None
kin_solver = None
robot_joint_names = None
_client = None
_stop_flag = False
_action_x = 0.0
_action_y = 0.0


# ==============================
# --- Robot Setup ---
# ==============================

def init_robot():
    """Initialize and calibrate the SO101 robot, and load kinematics solver."""
    global robot, kin_solver, robot_joint_names

    follower_config = SO101FollowerConfig(
        port="/dev/ttyACM0",
        id="my_robot_arm",
    )
    robot = SO101Follower(follower_config)
    robot.connect(calibrate=True)
    #robot.calibrate()

    robot_joint_names = list(robot.bus.motors.keys())
    print(f"[INIT] Robot joints: {robot_joint_names}")

    #robot.bus.disable_torque()



    kin_solver = RobotKinematics(
        urdf_path=URDF_PATH,
        target_frame_name="gripper_frame_link",
        joint_names=robot_joint_names
    )

    print("[INIT] Robot and kinematics initialized.")


# ==============================
# --- Motion Control ---
# ==============================
# def robot_units_to_degrees(norm_val, lower_rad, upper_rad):
#     """Convert normalized robot unit (-100..100) to joint angle in degrees."""
#     # Map -100..100 → lower..upper (in radians)
#     rad = ((norm_val + 100) / 200) * (upper_rad - lower_rad) + lower_rad
#     return np.degrees(rad)

def robot_units_to_degrees(norm_val, lower_rad, upper_rad, norm_mode="M100_100"):
    """Convert normalized robot unit to joint angle in degrees."""
    if norm_mode == "M100_100":
        rad = ((norm_val + 100) / 200) * (upper_rad - lower_rad) + lower_rad
    elif norm_mode == "M0_100":
        rad = (norm_val / 100) * (upper_rad - lower_rad) + lower_rad
    else:
        raise ValueError(f"Unknown norm_mode: {norm_mode}")
    return np.degrees(rad)

def degrees_to_robot_units(deg_val, lower_rad, upper_rad, norm_mode="M100_100"):
    """
    Convert joint angle in degrees to normalized robot units.

    norm_mode: 
        "M100_100" -> -100..100
        "M0_100"   -> 0..100 (for gripper)
    """
    rad_val = np.radians(deg_val)
    # Map rad_val from [lower_rad, upper_rad] → [-100,100] or [0,100]
    if norm_mode == "M100_100":
        norm = ((rad_val - lower_rad) / (upper_rad - lower_rad)) * 200 - 100
        return norm
    elif norm_mode == "M0_100":
        norm = ((rad_val - lower_rad) / (upper_rad - lower_rad)) * 100
        return norm
    else:
        raise ValueError(f"Unknown norm_mode: {norm_mode}")

def convert_joints_to_robot_units(joints_deg):
    """Convert an array of joint angles (deg) into robot units for all joints."""
    joint_ranges = [
        (-1.91986, 1.91986),   # shoulder_pan
        (-1.74533, 1.74533),   # shoulder_lift
        (-1.69,    1.69),      # elbow_flex
        (-1.65806, 1.65806),   # wrist_flex
        (-2.74385, 2.84121),   # wrist_roll
        (-0.174533, 1.74533)   # gripper
    ]

    norm_modes = [
        "M100_100",
        "M100_100",
        "M100_100",
        "M100_100",
        "M100_100",
        "M0_100"   # gripper
    ]

    return np.array([
        degrees_to_robot_units(joints_deg[i], *joint_ranges[i], norm_mode=norm_modes[i])
        for i in range(len(joints_deg))
    ])

def convert_current_joints(joints_arr):
    """
    Convert an array of normalized joint values (-100..100) into degrees,
    using each joint's specific motion limits.
    """
    norm_modes = ["M100_100"]*5 + ["M0_100"]

    joint_ranges = [
        (-1.91986, 1.91986),   # shoulder_pan
        (-1.74533, 1.74533),   # shoulder_lift
        (-1.69,    1.69),      # elbow_flex
        (-1.65806, 1.65806),   # wrist_flex
        (-2.74385, 2.84121),   # wrist_roll
        (-0.174533, 1.74533)   # gripper
    ]

    current_deg_joints = np.array([
        robot_units_to_degrees(val, *joint_ranges[i],norm_mode=norm_modes[i])
        for i, val in enumerate(joints_arr)
    ])

    return current_deg_joints


def move_robot_to_target(x: float, y: float):
    """Move robot linearly in Cartesian space toward a target (x, y)."""
    global robot, kin_solver, robot_joint_names

    nc_current_joints = np.array([
        robot.get_observation()[f"{name}.pos"] for name in robot_joint_names
    ])

    current_joints = convert_current_joints(nc_current_joints)
    current_pose = kin_solver.forward_kinematics(current_joints)
    current_pos = current_pose[:3, 3]

    #print(deg_current_joints)
    # print(current_joints)
    # print(current_pose)
    #print(current_pos)
    # return

    print(f"[MOVE] Current EE position: {np.round(current_pos, 3)}")

    # Define target end-effector position
    target_pos = np.array([x, y, Z_MIN])
    target_pose = current_pose.copy()
    target_pose[:3, 3] = target_pos

    # Linear interpolation
    for step in range(1, MOVE_STEPS + 1):
        # tolerance = 0.002  # 2 mm
        # if np.linalg.norm(current_pos - target_pos) < tolerance:
        #     print("Target reached, no movement needed")
        #     break 

        #alpha = step / MOVE_STEPS
        #interp_pos = (1 - alpha) * current_pos + alpha * target_pos
        interp_pose = current_pose.copy()
        #interp_pose[:3, 3] = interp_pos

        joint_angles = kin_solver.inverse_kinematics(current_joints, interp_pose)
        
        robot_units = convert_joints_to_robot_units(joint_angles)
        joint_dict = {f"{name}.pos": val for name, val in zip(robot_joint_names, robot_units)}
        #joint_dict["gripper.pos"] = 1.0

        robot.send_action(joint_dict)
        current_joints = joint_angles
        #time.sleep(MOVE_DELAY)

    print(f"[MOVE] Target reached: {np.round(target_pos, 3)}")
    print("--------------------------------")


# ==============================
# --- MQTT Handlers ---
# ==============================

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected successfully.")
        client.subscribe("robot/pre/#")
        print("[MQTT] Subscribed to topic pattern: robot/#")
    else:
        print(f"[MQTT] Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Handle incoming MQTT messages to move robot."""
    global _action_x, _action_y

    topic, payload = msg.topic, msg.payload.decode("utf-8").strip()

    try:
        value = float(payload)
    except ValueError:
        print(f"[WARN] Invalid payload '{payload}' on topic '{topic}'")
        return

    if topic == "robot/pre/x":
        _action_x = value
    elif topic == "robot/pre/y":
        _action_y = value
    else:
        print(f"[WARN] Unknown topic '{topic}', ignoring.")
        return

    print(f"[MQTT] Received: x={_action_x:.2f}, y={_action_y:.2f}")
    move_robot_to_target(_action_x, _action_y)


# ==============================
# --- MQTT Client Loop ---
# ==============================

def _mqtt_loop():
    global _client, _stop_flag
    while not _stop_flag:
        _client.loop(timeout=1.0)
        time.sleep(0.1)
    print("[MQTT] Loop stopped.")


def start_mqtt_listener(broker=MQTT_BROKER, port=MQTT_PORT, username=None, password=None):
    """Start MQTT listener in a background thread."""
    global _client, _stop_flag

    if _client is not None:
        print("[INFO] MQTT client already running.")
        return

    _stop_flag = False
    _client = mqtt.Client()

    if username and password:
        _client.username_pw_set(username, password)

    _client.on_connect = on_connect
    _client.on_message = on_message

    print(f"[MQTT] Connecting to {broker}:{port} ...")
    _client.connect(broker, port, keepalive=61)

    thread = threading.Thread(target=_mqtt_loop, daemon=True)
    thread.start()
    print("[MQTT] Listener started in background thread.")


# ==============================
# --- Main Execution ---
# ==============================

if __name__ == "__main__":
    init_robot()
    #move_robot_to_target(0.2,0.2)
    start_mqtt_listener()
    print("[SYSTEM] Ready and listening for MQTT commands...")
    try:
        while True:
            # Keep program alive so MQTT listener thread runs
            pass
    except KeyboardInterrupt:
        print("\n[MAIN] Stopping...")
        stop_mqtt_listener()
        disconnect_robot()
        print("[MAIN] Shutdown complete.")
