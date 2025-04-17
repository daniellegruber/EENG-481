import cv2
import mediapipe as mp
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(static_image_mode=False,
                       max_num_hands=2,
                       min_detection_confidence=0.5,
                       min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Hand landmark names based on MediaPipe's 21 key points
landmark_names = [
    "Wrist", "Thumb_CMC", "Thumb_MCP", "Thumb_SIP", "Thumb_Tip",
    "Index_MCP", "Index_PIP", "Index_DIP", "Index_Tip",
    "Middle_MCP", "Middle_PIP", "Middle_DIP", "Middle_Tip",
    "Ring_MCP", "Ring_PIP", "Ring_DIP", "Ring_Tip",
    "Pinky_MCP", "Pinky_PIP", "Pinky_DIP", "Pinky_Tip"
]

# Open USB camera (default ID 0, change if necessary based on where camera is connected)
camera = cv2.VideoCapture(0)

if not camera.isOpened():
    print("Error: Camera could not be opened.")
    exit()

last_print_time = time.time()

while True:
    ret, frame = camera.read()
    if not ret:
        print("Error: Frame not read from camera.")
        break

    # Flip frame horizontally for a mirror view
    frame = cv2.flip(frame, 1)

    # Convert frame to RGB
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Process frame using MediaPipe Hands
    results = hands.process(rgb_frame)

    current_time = time.time()
    
    if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
            # Extract bounding box coordinates
            h, w, _ = frame.shape
            x_min = int(min([landmark.x for landmark in hand_landmarks.landmark]) * w)
            x_max = int(max([landmark.x for landmark in hand_landmarks.landmark]) * w)
            y_min = int(min([landmark.y for landmark in hand_landmarks.landmark]) * h)
            y_max = int(max([landmark.y for landmark in hand_landmarks.landmark]) * h)

            # Draw bounding box
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Optionally draw hand landmarks
            mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
            
            # Print landmark data with names every 2 seconds
            # if current_time - last_print_time >= 2:
            #     for idx, landmark in enumerate(hand_landmarks.landmark):
            #         print(f"{landmark_names[idx]}: (x={landmark.x}, y={landmark.y}, z={landmark.z})")
            #     last_print_time = current_time

    # Display the frame
    cv2.imshow("Hand Detection", frame)

    # Exit loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
camera.release()
cv2.destroyAllWindows()
