import mediapipe as mp
import cv2
import math

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

class HandTracker:
    def __init__(self, model_path="models/hand_landmarker.task"):
        self.cap = None
        self.latest_result = None
        self.timestamp = 0
        
        # Initialize the HandLandmarker
        options = HandLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self._result_callback
        )
        self.landmarker = HandLandmarker.create_from_options(options)

    def _result_callback(self, result, output_image, timestamp_ms):
        """Callback for the HandLandmarker"""
        self.latest_result = result

    def start_camera(self, camera_id=0):
        """Initialize the camera"""
        self.cap = cv2.VideoCapture(camera_id)
        return self.cap.isOpened()

    def get_hand_position(self):
        """Get the current hand position and pinch status"""
        if not self.cap or not self.cap.isOpened():
            return None, None, None, None

        success, image = self.cap.read()
        if not success:
            return None, None, None, None

        # Convert BGR to RGB and create MediaPipe Image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image_rgb)
        
        # Process the frame
        self.landmarker.detect_async(mp_image, self.timestamp)
        self.timestamp += 1  # Update timestamp for next frame
        
        if self.latest_result and self.latest_result.hand_landmarks:
            hand = self.latest_result.hand_landmarks[0]  # Get first hand
            
            # Get thumb and index finger coordinates
            thumb_tip = hand[4]
            index_tip = hand[8]
            
            # Calculate pinch center point
            cx = (thumb_tip.x + index_tip.x) / 2
            cy = (thumb_tip.y + index_tip.y) / 2
            
            # Calculate pinch distance
            pinch_distance = math.hypot(index_tip.x - thumb_tip.x, index_tip.y - thumb_tip.y)
            
            # Draw visualization
            h, w, _ = image.shape
            
            # Draw thumb and index tips
            x_thumb = int(thumb_tip.x * w)
            y_thumb = int(thumb_tip.y * h)
            x_index = int(index_tip.x * w)
            y_index = int(index_tip.y * h)
            cv2.circle(image, (x_thumb, y_thumb), 10, (0, 255, 0), -1)
            cv2.circle(image, (x_index, y_index), 10, (0, 255, 0), -1)

            # Draw pinch center if pinching
            if pinch_distance < 0.1:
                cx_pix = int(cx * w)
                cy_pix = int(cy * h)
                cv2.circle(image, (cx_pix, cy_pix), 8, (0, 0, 255), -1)

            return cx, cy, pinch_distance < 0.1, image

        return None, None, None, image

    def stop(self):
        """Release resources"""
        if self.cap:
            self.cap.release()
        self.landmarker.close()