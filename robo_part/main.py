from robot_mqtt_controller import (
    initialize_robot,
    start_mqtt_listener,
    stop_mqtt_listener,
    disconnect_robot
)

# --- CONFIG ---
BROKER = "10.148.40.253"  # Change to your MQTT broker IP if needed
PORT = 1883

# --- INITIALIZE ---
initialize_robot("/dev/ttyACM1")   # Connect robot
start_mqtt_listener(broker=BROKER, port=PORT)  # Start listening for commands

print("[MAIN] Robot MQTT controller is running. Press Ctrl+C to stop.")

try:
    while True:
        # Keep program alive so MQTT listener thread runs
        pass
except KeyboardInterrupt:
    print("\n[MAIN] Stopping...")
    stop_mqtt_listener()
    disconnect_robot()
    print("[MAIN] Shutdown complete.")
