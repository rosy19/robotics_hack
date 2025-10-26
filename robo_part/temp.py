import csv
import time
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

# --- Robot Configuration ---
follower_config = SO101FollowerConfig(
    port="/dev/ttyACM1",
    id="my_robot_arm",
)
device = SO101Follower(follower_config)
device.connect()

# --- CSV Configuration ---
CSV_PATH = "positions_actions.csv"  # TODO: set your CSV path here
COLUMN_INDICES = [2, 3, 4]  # Python is 0-indexed → columns 3–6 in human terms

# --- Timing parameters ---
DELAY_BETWEEN_ACTIONS = 0.05  # seconds between each row command

# --- Fixed joints ---
FIXED_WRIST_ROLL = 0.0
FIXED_GRIPPER = 2.0
FIXED_WRIST_FLEX = 0.0

# --- Load and send data ---
with open(CSV_PATH, newline='') as csvfile:
    reader = csv.reader(csvfile)
    next(reader)  # Skip header if exists (remove this line if no header)

    for i, row in enumerate(reader, start=1):
        try:
            # Extract values from the desired columns
            values = [float(row[idx]) for idx in COLUMN_INDICES]

            # Map them to the robot action
            action = {
                'shoulder_pan.pos': values[0],
                'shoulder_lift.pos': values[1],
                'elbow_flex.pos': values[2],
                'wrist_flex.pos': FIXED_WRIST_FLEX,
                'wrist_roll.pos': FIXED_WRIST_ROLL,
                'gripper.pos': FIXED_GRIPPER
            }

            # Send to robot
            device.send_action(action)
            print(f"[{i}] Sent action: {action}")

            time.sleep(DELAY_BETWEEN_ACTIONS)

        except (ValueError, IndexError) as e:
            print(f"[{i}] Skipping row due to error: {e}")

print("✅ Finished sending all actions.")
