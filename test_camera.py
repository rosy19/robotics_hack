import cv2

def main():
    cap = cv2.VideoCapture(0)  # Use 0 for default camera, change if needed
    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("Press ESC to exit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        cv2.imshow("Camera Test", frame)
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
