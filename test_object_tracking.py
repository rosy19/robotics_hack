import cv2
import mediapipe as mp
import math 

# Initialize Mediapipe
mp_drawing = mp.solutions.drawing_utils
mp_hands = mp.solutions.hands

# OpenCV video capture
cap = cv2.VideoCapture(1)

with mp_hands.Hands(
    model_complexity=0,
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
) as hands:
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            continue
        
        # Convert BGR to RGB for Mediapipe
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Process the frame
        results = hands.process(image_rgb)
        # Draw landmarks on the original frame
        image.flags.writeable = True
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # get index finger tip (landmark #8)
                h, w, _ = image.shape
                x_idx = int(hand_landmarks.landmark[8].x * w)
                y_idx = int(hand_landmarks.landmark[8].y * h)
                cv2.circle(image, (x_idx, y_idx), 10, (0, 255, 0), -1)
                print(f"Index finger tip: ({hand_landmarks.landmark[8].x}, {hand_landmarks.landmark[8].y})")
                
                # get thumb finger tip (landmark #4)
                x_tmb = int(hand_landmarks.landmark[4].x * w)
                y_tmb = int(hand_landmarks.landmark[4].y * h)
                cv2.circle(image, (x_tmb, y_tmb), 10, (0, 255, 0), -1)
                print(f"Thumb finger tip: ({hand_landmarks.landmark[4].x}, {hand_landmarks.landmark[4].y})")
                
                # Compute midpoint between thumb and index finger
                cx = (hand_landmarks.landmark[8].x+ hand_landmarks.landmark[4].x) / 2
                cy = (hand_landmarks.landmark[8].y + hand_landmarks.landmark[4].y) / 2

                # Compute distance between the tips (for pinch detection)
                dist = math.hypot(hand_landmarks.landmark[8].x - hand_landmarks.landmark[4].x, hand_landmarks.landmark[8].y - hand_landmarks.landmark[4].y)
                if dist < 0.1:
                    # Visualizations
                    cx_pix = int(cx * w)
                    cy_pix = int(cy * h)
                    cv2.circle(image, (cx_pix, cy_pix), 8, (0, 255, 0), -1)
                print(f"Pinch center: ({cx:.3f}, {cy:.3f}), Distance: ({dist:.3f})")

        # Display
        cv2.imshow("MediaPipe Hands", cv2.flip(image, 1))
        if cv2.waitKey(1) & 0xFF == 27:  # ESC to quit
            break

cap.release()
cv2.destroyAllWindows()
