import ikpy.chain
import numpy as np

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
    
    # Define the names of the joints/links you actually want to control with IK
    # These typically correspond to the revolute joints listed in your URDF's <joint> tags
    # Example: ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll']
    # Adjust this list based on the URDF file structure and your needs
    active_link_names = ['shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll'] # Example for 5 DOF arm pose
    
    # Create the active_links_mask based on the names
    active_links_mask = [link.name in active_link_names for link in my_chain.links]
    
    # Recreate the chain with the specific mask if you want fine control
    # Or, rely on ikpy's default behavior and adjust slicing later.
    # For simplicity, let's stick to slicing for now, assuming the default includes the base + active joints + gripper.

    # Expected number of controllable joints based on URDF (excluding fixed)
    # This count should match the number of non-fixed joints ikpy will try to solve for.
    num_active_joints = sum(1 for link in my_chain.links if link.joint_type != 'fixed') 
    # Usually, we care about the arm pose joints (e.g., 5 for SO101's arm)
    num_pose_joints = len(active_link_names) # Should be 5 based on URDF parsing
    print(f"Total non-fixed joints found by ikpy (incl. gripper): {num_active_joints}")
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
        calculated_joint_angles_rad = my_chain.inverse_kinematics(
            target_position=target_position,
            initial_position=initial_joint_angles_rad,
            # You might need to add target_orientation and orientation_mode here
            # depending on whether you need to control the pen's angle precisely.
            # Example for pointing down (requires experimentation):
            # target_orientation=[0, -1, 0], # Pointing Y-down in the tool frame
            # orientation_mode="Z" # Match the Z-axis orientation
        )

        # Convert radians to degrees
        calculated_joint_angles_deg = np.degrees(calculated_joint_angles_rad)

        # --- Verification (Forward Kinematics) --- Optional but useful
        # Use the full angle list (in radians) returned by IK for FK check
        fk_matrix = my_chain.forward_kinematics(calculated_joint_angles_rad) 
        fk_position = fk_matrix[:3, 3] # Extract X, Y, Z position
        position_error = np.linalg.norm(np.array(fk_position) - np.array(target_position))
        print(f"  IK Target(m): {target_position}, FK Result(m): [{fk_position[0]:.4f}, {fk_position[1]:.4f}, {fk_position[2]:.4f}], Pos Error: {position_error * 100:.3f} cm")

        # --- FIX FOR ANGLE SLICING ---
        # Based on URDF and previous output, ikpy likely returns angles for:
        # [dummy_base, shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper, ...] 
        # We want the 5 angles from shoulder_pan to wrist_roll.
        # This corresponds to indices 1 through 5 (inclusive) in the full list.
        if len(calculated_joint_angles_deg) >= (num_pose_joints + 1): # Check if we have enough angles
             active_pose_joint_angles_deg = calculated_joint_angles_deg[1 : num_pose_joints + 1] # Slice indices 1, 2, 3, 4, 5
        else:
             print(f"Error: IK returned only {len(calculated_joint_angles_deg)} angles, expected at least {num_pose_joints + 1}.")
             return None
             
        # Optional: Further check if the number matches exactly what you need
        if len(active_pose_joint_angles_deg) != num_pose_joints:
             print(f"Warning: Sliced angles count ({len(active_pose_joint_angles_deg)}) doesn't match expected pose joints ({num_pose_joints}). Full output (deg): {calculated_joint_angles_deg}")
             # Depending on robustness needed, you might return None here too.

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
    print(f"Attempting to move to target (m): [{target_x_m:.4f}, {target_y_m:.4f}, {target_z_m:.4f}]")
    
    # --- FIX FOR UNITS ---
    # Ensure coordinates passed to calculate_inverse_kinematics are in METERS.
    joint_angles_deg = calculate_inverse_kinematics(target_x_m, target_y_m, target_z_m)

    if joint_angles_deg is not None:
        # Ensure we have the correct number of angles before proceeding
        if len(joint_angles_deg) == num_pose_joints:
            print(f"  IK Solution Found. Sending joint angles (degrees):")
            active_links_names_for_print = [link.name for link in my_chain.links if link.joint_type != 'fixed'][0:num_pose_joints] # Get the names corresponding to the angles
            for name, angle in zip(active_links_names_for_print, joint_angles_deg):
                 print(f"    {name}: {angle:.4f}")
                 
            # ================================================================
            # TODO: Add your code here to send the `joint_angles_deg` array 
            #       to your actual robot hardware interface/controller script.
            # Example (pseudo-code):
            # robot_hardware_interface.set_joint_positions(joint_angles_deg) 
            # ================================================================
            print("  (Placeholder: Sent angles to robot hardware)") 
        else:
            print(f"  Error: IK calculation returned an unexpected number of angles ({len(joint_angles_deg)}). Cannot move robot.")
            
    else:
        print(f"  Failed to move robot - IK solution not found.")

def go_home(x_home_m=0.15, y_home_m=0.0, z_home_m=0.20):
    """Moves the robot to a predefined home position."""
    print(f"\nMoving to Home Position (m): [{x_home_m}, {y_home_m}, {z_home_m}]")
    move_to_point(x_home_m, y_home_m, z_home_m)