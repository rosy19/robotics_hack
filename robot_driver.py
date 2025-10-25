import ikpy.chain
import numpy as np

import paho.mqtt.client as mqtt

# Define joint limits in degrees for normalization
JOINT_LIMITS = {
    'shoulder_pan':   (-100, 100),   # example values, replace with your real limits
    'shoulder_lift':  (-100, 100),
    'elbow_flex':     (-100, 100),
    'wrist_flex':     (-100, 100),
    'wrist_roll':     (-100, 100)
}

# --- MQTT CONFIG ---
MQTT_BROKER = "10.148.40.253"   # ← Replace with your Raspberry Pi IP (or 'localhost' if broker runs here)
MQTT_PORT = 1883
MQTT_BASE_TOPIC = "robot"

# --- Connect MQTT ---
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)


# --- Define Your Robot's Kinematic Chain ---
# Load the robot structure directly from the URDF file.
# Make sure the URDF file path is correct.
try:
    # IMPORTANT: Ensure this path is correct for your system
    urdf_file_path = "/Users/dambast/Desktop/robotics_hack/so101_new_calib.urdf" 
    my_chain = ikpy.chain.Chain.from_urdf_file(urdf_file_path)
    print("URDF file loaded successfully.")
    
    # Identify active links (those controlled by IK, excluding fixed joints)
    # The list 'active_links_mask' determines which links ikpy considers controllable.
    # We often want to exclude the base and any fixed end-effector mounts.
    # Let's rebuild the chain with an explicit mask if needed, focusing on the arm joints.
    
    # Get names of all links defined in URDF
    all_link_names = [link.name for link in my_chain.links]
    print("All links found:", all_link_names)

    # Derive non-fixed (actuated) joint/link names from the chain
    non_fixed_joint_names = [link.name for link in my_chain.links if link.joint_type != 'fixed']

    # Define the pose joints we want IK to control (exclude gripper/other non-pose DOFs)
    # Keep order stable for downstream mapping and publishing
    pose_joint_names = [
        name for name in ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
        if name in non_fixed_joint_names
    ]

    # Create the active_links_mask based on pose joints (True only for pose joints)
    active_links_mask = [link.name in pose_joint_names for link in my_chain.links]

    # Counts for logging/validation
    num_non_fixed_joints = len(non_fixed_joint_names)
    num_pose_joints = len(pose_joint_names)
    print(f"Total non-fixed joints found by ikpy (incl. gripper): {num_non_fixed_joints}")
    print(f"Active pose joints (masked): {pose_joint_names}")
    print(f"Expecting {num_pose_joints} active pose joint angles.")

except FileNotFoundError:
    print(f"ERROR: URDF file not found at the specified path: {urdf_file_path}")
    exit()
except Exception as e:
    print(f"An error occurred while loading the URDF file: {e}")
    exit()

def calculate_inverse_kinematics(target_x_m, target_y_m, target_z_m):
    """
    Calculates the joint angles needed to reach a target XYZ position using IK.

    Args:
        target_x_m (float): Target X coordinate in METERS.
        target_y_m (float): Target Y coordinate in METERS.
        target_z_m (float): Target Z coordinate in METERS.

    Returns:
        numpy.ndarray or None: An array of joint angles in degrees for the ACTIVE POSE JOINTS
                               (e.g., shoulder_pan to wrist_roll),
                               or None if the calculation fails or returns an unexpected number of angles.
    """
    target_position = [target_x_m, target_y_m, target_z_m]

    # Get initial joint angles (using zeros is a common starting point)
    # ikpy includes a dummy link at the start, length is num_links + 1
    # We need angles in radians for the calculation.
    initial_joint_angles_rad = [0.0] * len(my_chain.links) 

    try:
        # --- Calculate Inverse Kinematics ---
        # Prefer a stable orientation so the tool doesn't flip while tracing.
        # Constrain the end-effector Z axis; fallback to unconstrained if IK fails.
        target_orientation_vec = [0.0, 0.0, -1.0]  # Assumes tool Z should point "down"; adjust if needed
        try:
            calculated_joint_angles_rad = my_chain.inverse_kinematics(
                target_position=target_position,
                initial_position=initial_joint_angles_rad,
                target_orientation=target_orientation_vec,
                orientation_mode="Z",
                active_links_mask=active_links_mask
            )
        except Exception as e_orient:
            # Fallback: run without orientation constraint but still respect active mask
            calculated_joint_angles_rad = my_chain.inverse_kinematics(
                target_position=target_position,
                initial_position=initial_joint_angles_rad,
                active_links_mask=active_links_mask
            )

        # Convert radians to degrees
        calculated_joint_angles_deg = np.degrees(calculated_joint_angles_rad)

        # --- Verification (Forward Kinematics) --- Optional but useful
        # Use the full angle list (in radians) returned by IK for FK check
        fk_matrix = my_chain.forward_kinematics(calculated_joint_angles_rad) 
        fk_position = fk_matrix[:3, 3] # Extract X, Y, Z position
        position_error = np.linalg.norm(np.array(fk_position) - np.array(target_position))
        #print(f"  IK Target(m): {target_position}, FK Result(m): [{fk_position[0]:.4f}, {fk_position[1]:.4f}, {fk_position[2]:.4f}], Pos Error: {position_error * 100:.3f} cm")

        # --- ROBUST JOINT MAPPING BY NAME ---
        # Map returned angles to link names, then pick only pose joints in the desired order
        angles_by_name_deg = {}
        for idx, link in enumerate(my_chain.links):
            # IKPy returns one angle per link; for fixed links this is typically 0
            if hasattr(link, 'joint_type') and link.joint_type != 'fixed':
                angles_by_name_deg[link.name] = calculated_joint_angles_deg[idx]

        active_pose_joint_angles_deg = [angles_by_name_deg[name] for name in pose_joint_names if name in angles_by_name_deg]

        if len(active_pose_joint_angles_deg) != num_pose_joints:
            print(f"Warning: Could not map all pose joints. Expected {num_pose_joints}, got {len(active_pose_joint_angles_deg)}. Mapped: {list(angles_by_name_deg.keys())}")
            return None

        return active_pose_joint_angles_deg

    except ValueError as e:
        # This often happens if the target is physically unreachable
        print(f"IK calculation failed for target {target_position}: {e}")
        return None

def move_to_point(target_x_m, target_y_m, target_z_m):
    """
    Calculates IK and sends joint angles to the robot hardware (placeholder).
    
    Args:
        target_x_m (float): Target X coordinate in METERS.
        target_y_m (float): Target Y coordinate in METERS.
        target_z_m (float): Target Z coordinate in METERS.
    """
    #print(f"Attempting to move to target (m): [{target_x_m:.4f}, {target_y_m:.4f}, {target_z_m:.4f}]")
    
    # --- FIX FOR UNITS ---
    # Ensure coordinates passed to calculate_inverse_kinematics are in METERS.
    joint_angles_deg = calculate_inverse_kinematics(target_x_m, target_y_m, target_z_m)

    if joint_angles_deg is not None:
        # Ensure we have the correct number of angles before proceeding
        if len(joint_angles_deg) == num_pose_joints:
            # Stable mapping: publish using pose_joint_names order
            for name, angle in zip(pose_joint_names, joint_angles_deg):
                if name not in JOINT_LIMITS:
                    print(f"  Warning: JOINT_LIMITS missing for {name}. Skipping publish for this joint.")
                    continue

                # Normalize angle to [-100, 100]
                min_angle, max_angle = JOINT_LIMITS[name]
                normalized = 200 * (angle - min_angle) / (max_angle - min_angle) - 100
                normalized = max(-100, min(100, normalized))  # Clamp to [-100, 100]

                topic = f"{MQTT_BASE_TOPIC}/post/{name}"
                mqtt_client.publish(topic, normalized)
                #print(f"    Published: {topic} = {normalized:.2f}")

            print("  ✅ All joint angles published to MQTT.") 
        else:
            print(f"  Error: IK calculation returned an unexpected number of angles ({len(joint_angles_deg)}). Cannot move robot.")
            
    else:
        print(f"  Failed to move robot - IK solution not found.")

def go_home(x_home_m=0.15, y_home_m=0.0, z_home_m=0.20):
    """Moves the robot to a predefined home position."""
    print(f"\nMoving to Home Position (m): [{x_home_m}, {y_home_m}, {z_home_m}]")
    move_to_point(x_home_m, y_home_m, z_home_m)