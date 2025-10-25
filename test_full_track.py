import mediapipe as mp
import cv2
import time
import math

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

# Load the pre-trained MediaPipe model
MODEL_PATH = "models/hand_landmarker.task"

def print_result(result, output_image, timestamp_ms):
    if len(result.hand_landmarks) > 0:
        hand = result.hand_landmarks[0]
        thumb_tip = hand[4]
        index_tip = hand[8]

        # Compute midpoint between thumb and index finger
        cx = (thumb_tip.x + index_tip.x) / 2
        cy = (thumb_tip.y + index_tip.y) / 2

        # Compute distance between the tips (for pinch detection)
        dist = math.hypot(index_tip.x - thumb_tip.x, index_tip.y - thumb_tip.y)

        print(f"Pinch center: ({cx:.3f}, {cy:.3f}), distance: {dist:.4f}")

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path=MODEL_PATH),
    running_mode=VisionRunningMode.LIVE_STREAM,
    result_callback=print_result
)

cap = cv2.VideoCapture(0)
timestamp = 0

with HandLandmarker.create_from_options(options) as landmarker:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            continue

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
        landmarker.detect_async(mp_image, timestamp)
        timestamp += 33  # ~30 fps

        cv2.imshow("Camera", frame)
        if cv2.waitKey(5) & 0xFF == 27:
            break

cap.release()
cv2.destroyAllWindows()
