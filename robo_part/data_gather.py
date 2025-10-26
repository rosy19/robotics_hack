from lerobot.teleoperators.so101_leader import SO101LeaderConfig, SO101Leader
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower

_pos_x = 0.0
_pos_y = 0.0

old_pos_x = 0.0
old_pos_y = 0.0

_action = {'shoulder_pan.pos': -1.8003273322422189, 'shoulder_lift.pos': -21.953221173574065, 'elbow_flex.pos': 70.04484304932734, 'wrist_flex.pos': 0.0, 'wrist_roll.pos': 0.0, 'gripper.pos': 2.0}

_client = None

device = None

# - Robot Type: Follower / Leader
# - True: Leader -> Gathers data from Leader robot
# - False: Follower -> Moves the robot acording to data
type = False # False - Follower, True - Leader

# --- ROBOT SETUP ---
def initialize_robot():
    """Initialize and connect to the robot."""
    global device, type
    if type:
        leader_config = SO101LeaderConfig(
            port="/dev/ttyACM1",
            id="my_controller",
        )
        device = SO101Leader(leader_config)
        device.connect()
    else:
        follower_config = SO101FollowerConfig(
            port="/dev/ttyACM1",
            id="my_robot_arm",
        )   
        device = SO101Follower(follower_config)
        device.connect()
        time.sleep(1)
        device.send_action(_action)

    return device

# --- MQTT CALLBACKS ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected successfully.")
        client.subscribe("robot/pre/#")
        print("[MQTT] Subscribed to topic: robot/pre/#")
    else:
        print(f"[MQTT] Connection failed with code {rc}")

def on_message(client, userdata, msg):
    """Handle incoming MQTT messages for multiple topics."""
    global _action, _pos_x, _pos_y, old_pos_x, old_pos_y

    topic = msg.topic
    payload = msg.payload.decode("utf-8").strip()

    if topic == "robot/pre/x":
        if _pos_x is not None:
            old_pos_x = _pos_x
        _pos_x = payload
    elif topic == "robot/pre/y":
        if _pos_y is not None:
            old_pos_y = _pos_y
        _pos_y = payload
    try:
        if old_pos_x != _pos_x or old_pos_y != _pos_y:
            if type:
                update_position(_pos_x, _pos_y)
            else:
                move_robot(_pos_x, _pos_y)
    except ValueError:
        print(f"[ERROR] Invalid float value received on topic '{topic}': {payload}")

# --- MQTT CONTROLLER ---
def start_mqtt_listener(broker="localhost", port=1883):
    """Start MQTT client in a background thread."""
    global _client

    if _client is not None:
        print("[INFO] MQTT client already running.")
        return

    _client = mqtt.Client()

    _client.on_connect = on_connect
    _client.on_message = on_message

    print(f"[MQTT] Connecting to {broker}:{port} ...")
    _client.connect(broker, port, 60)

    thread = threading.Thread(target=_mqtt_loop, daemon=True)
    thread.start()
    print("[MQTT] Listener started in background thread.")

def _mqtt_loop():
    """Internal thread loop for MQTT client."""
    global _client
    while True:
        _client.loop(timeout=1.0)
        time.sleep(0.1)
    print("[MQTT] Loop stopped.")

def update_position(x, y):
    """Saves data in a csv file when robot is a leader"""
    # Read back current action from the robot and append to CSV:
    
    csv_path = "positions_actions.csv"
    action = device.get_action()

    x_val = float(x)
    y_val = float(y)

    # collect action values in a deterministic order (insertion order)
    # exclude the last action entry
    action_keys = list(action.keys())[:-3]
    values = []
    for k in action_keys:
        values.append(float(action[k]))

    write_header = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["pos_x", "pos_y"] + action_keys)
        writer.writerow([x_val, y_val] + values)


def move_robot(x, y):
    """Use the loaded Keras model to map (x,y) -> first 4 action values and send to the robot."""
    global _model, _action, device
    x_val = float(x)
    y_val = float(y)
    
    if _model is None:
        print("[ERROR] move_robot: model not loaded.")
        return

    inp = np.array([[x_val, y_val]], dtype=float)
    
    preds = _model.predict(inp)
    preds = np.asarray(preds).flatten()

    if preds.size < 3:
        print(f"[ERROR] move_robot: model output must contain at least 3 values, got {preds.size}")
        return

    first3 = [float(preds[i]) for i in range(3)]
    # Update the first three action entries in a deterministic order

    _action['shoulder_pan.pos'] = first3[0]
    _action['shoulder_lift.pos'] = first3[1]
    _action['elbow_flex.pos'] = first3[2]

    print(f"X: {x_val}, Y: {y_val}, shoulder_pan: {_action['shoulder_pan.pos']}, shoulder_lift: {_action['shoulder_lift.pos']}, elbow_flex: {_action['elbow_flex.pos']}")
    device.send_action(_action)


def init_model(model_path):
    """Load a Keras model from disk into the module-level _model variable."""
    global _model
    try:
        _model = keras.models.load_model(model_path)
        print(f"[INFO] Keras model loaded from {model_path}.")
    except Exception as e:
        print(f"[ERROR] Failed to load model from {model_path}: {e}")
        _model = None


if __name__ == "__main__":
    import threading
    import time
    import os
    import csv
    import numpy as np
    from tensorflow import keras
    import paho.mqtt.client as mqtt

    # --- MQTT SETTINGS ---
    BROKER = "localhost"
    PORT = 1883

        # --- INITIALIZE ---
    initialize_robot()   # Connect robot
    if not type:
        init_model("arm_model.keras")

    start_mqtt_listener(broker=BROKER, port=PORT)  # Start listening for commands

    try:
        while True:
            time.sleep(0.001)
    except KeyboardInterrupt:
        print("\n[INFO] Shutting down...")