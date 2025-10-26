# robot_mqtt_controller.py

import json
import time
import threading
import paho.mqtt.client as mqtt
from lerobot.robots.so101_follower import SO101FollowerConfig, SO101Follower


# --- GLOBAL VARIABLES ---
_device = None
_action = {
    'shoulder_pan.pos': 0.0,
    'shoulder_lift.pos': 0.0,
    'elbow_flex.pos': 0.0,
    'wrist_flex.pos': 0.0,
    'wrist_roll.pos': 0.0,
    'gripper.pos': 100.0,
}

TOPIC_TO_ACTION_KEY = {
    "robot/post/shoulder_pan": "shoulder_pan.pos",
    "robot/post/shoulder_lift": "shoulder_lift.pos",
    "robot/post/elbow_flex": "elbow_flex.pos",
    "robot/post/wrist_flex": "wrist_flex.pos",
    "robot/post/gripper": "gripper.pos"
}


_client = None
_stop_flag = False


# --- ROBOT SETUP ---
def initialize_robot(port="/dev/ttyACM1", robot_id="my_robot_arm_b"):
    """Initialize and connect to the robot."""
    global _device
    if _device is not None:
        print("[INFO] Robot already initialized.")
        return _device

    cfg = SO101FollowerConfig(port=port, id=robot_id)
    _device = SO101Follower(cfg)
    print("[INFO] Connecting to robot...")
    _device.connect()
    print("[INFO] Robot connected.")
    return _device


def send_action():
    """Send the current action dictionary to the robot."""
    global _device, _action
    if _device is None:
        raise RuntimeError("Robot not initialized. Call initialize_robot() first.")
    _device.send_action(_action)
    print(f"[INFO] Sent action: {_action}")


# --- MQTT CALLBACKS ---
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected successfully.")
        client.subscribe("robot/post/#")  # Example topic pattern
        print("[MQTT] Subscribed to topic: robot/#")
    else:
        print(f"[MQTT] Connection failed with code {rc}")


def on_message(client, userdata, msg):
    """Handle incoming MQTT messages for multiple topics."""
    global _action

    topic = msg.topic
    payload = msg.payload.decode("utf-8").strip()

    if topic not in TOPIC_TO_ACTION_KEY:
        print(f"[WARN] Unknown topic '{topic}', ignoring.")
        return

    try:
        value = float(payload)  # Convert payload to float
        action_key = TOPIC_TO_ACTION_KEY[topic]
        _action[action_key] = value
        send_action()
    except ValueError:
        print(f"[ERROR] Invalid float value received on topic '{topic}': {payload}")


# --- MQTT CONTROLLER ---
def start_mqtt_listener(broker="localhost", port=1883, username=None, password=None):
    """Start MQTT client in a background thread."""
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
    _client.connect(broker, port, 60)

    thread = threading.Thread(target=_mqtt_loop, daemon=True)
    thread.start()
    print("[MQTT] Listener started in background thread.")


def _mqtt_loop():
    """Internal thread loop for MQTT client."""
    global _client, _stop_flag
    while not _stop_flag:
        _client.loop(timeout=1.0)
        time.sleep(0.1)
    print("[MQTT] Loop stopped.")


def stop_mqtt_listener():
    """Stop MQTT listener and disconnect."""
    global _client, _stop_flag
    _stop_flag = True
    if _client:
        _client.disconnect()
        _client = None
        print("[MQTT] Disconnected.")


def disconnect_robot():
    """Safely disconnect the robot."""
    global _device
    if _device:
        print("[INFO] Disconnecting robot...")
        _device.disconnect()
        _device = None
        print("[INFO] Robot disconnected.")
