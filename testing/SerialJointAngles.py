import cv2
import mediapipe as mp
import time
import math
import serial  # Import PySerial

# Initialize serial communication with Arduino.
# Change the port string below to match your system (e.g. 'COM3' on Windows or '/dev/tty.usbmodem...' on macOS).
arduino = serial.Serial('/dev/cu.usbmodem21201', 9600, timeout=1)
time.sleep(2)  # Allow time for Arduino to reset

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_drawing = mp.solutions.drawing_utils

# For naming landmarks (not used in mapping but useful for debugging)
landmark_names = [
    "Wrist", "Thumb_CMC", "Thumb_MCP", "Thumb_SIP", "Thumb_Tip",
    "Index_MCP", "Index_PIP", "Index_DIP", "Index_Tip",
    "Middle_MCP", "Middle_PIP", "Middle_DIP", "Middle_Tip",
    "Ring_MCP", "Ring_PIP", "Ring_DIP", "Ring_Tip",
    "Pinky_MCP", "Pinky_PIP", "Pinky_DIP", "Pinky_Tip"
]

# Function to calculate the angle (in degrees) at point b given points a, b, c.
def calculate_angle(a, b, c):
    ba = [a[i] - b[i] for i in range(3)]
    bc = [c[i] - b[i] for i in range(3)]
    dot_product = sum(ba[i] * bc[i] for i in range(3))
    norm_ba = math.sqrt(sum(ba[i] ** 2 for i in range(3)))
    norm_bc = math.sqrt(sum(bc[i] ** 2 for i in range(3)))
    if norm_ba * norm_bc == 0:
        return 0.0
    cosine_angle = dot_product / (norm_ba * norm_bc)
    cosine_angle = max(min(cosine_angle, 1.0), -1.0)  # Clamp
    angle = math.acos(cosine_angle)
    return math.degrees(angle)

# Helper to convert normalized landmark coordinates to pixel coordinates.
def to_pixel_coords(landmark, width, height):
    return int(landmark.x * width), int(landmark.y * height)

# Open the camera.
camera = cv2.VideoCapture(0)
if not camera.isOpened():
    print("Error: Camera could not be opened.")
    exit()

# This variable holds the center (in pixel coordinates) of the originally detected hand.
tracked_hand_center = None

while True:
    ret, frame = camera.read()
    if not ret:
        print("Error: Frame not read from camera.")
        break

    # Flip for a mirror view.
    frame = cv2.flip(frame, 1)
    h, w, _ = frame.shape

    # Process the frame with MediaPipe.
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb_frame)

    if results.multi_hand_landmarks:
        # Compute centers of detected hands.
        hand_centers = []
        for hand_landmarks in results.multi_hand_landmarks:
            cx = int(sum([lm.x for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark) * w)
            cy = int(sum([lm.y for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark) * h)
            hand_centers.append((cx, cy))
        
        # Select the hand that’s closest to the previously tracked center.
        if tracked_hand_center is None:
            tracked_hand_center = hand_centers[0]
            selected_hand = results.multi_hand_landmarks[0]
        else:
            best_distance = float('inf')
            best_index = 0
            for i, center in enumerate(hand_centers):
                dx = center[0] - tracked_hand_center[0]
                dy = center[1] - tracked_hand_center[1]
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < best_distance:
                    best_distance = distance
                    best_index = i
            selected_hand = results.multi_hand_landmarks[best_index]
            tracked_hand_center = hand_centers[best_index]

        # Draw bounding box and landmarks for the selected hand.
        x_min = int(min([lm.x for lm in selected_hand.landmark]) * w)
        x_max = int(max([lm.x for lm in selected_hand.landmark]) * w)
        y_min = int(min([lm.y for lm in selected_hand.landmark]) * h)
        y_max = int(max([lm.y for lm in selected_hand.landmark]) * h)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        mp_drawing.draw_landmarks(frame, selected_hand, mp_hands.HAND_CONNECTIONS)

        landmarks = selected_hand.landmark

        # --- Compute Joint Angles (MCP for each finger) ---
        # Thumb MCP (index 2)
        angle_thumb_mcp = calculate_angle(
            (landmarks[1].x, landmarks[1].y, landmarks[1].z),
            (landmarks[2].x, landmarks[2].y, landmarks[2].z),
            (landmarks[3].x, landmarks[3].y, landmarks[3].z)
        )

        # Index MCP (index 5)
        angle_index_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[5].x, landmarks[5].y, landmarks[5].z),
            (landmarks[6].x, landmarks[6].y, landmarks[6].z)
        )

        # Middle MCP (index 9)
        angle_middle_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[9].x, landmarks[9].y, landmarks[9].z),
            (landmarks[10].x, landmarks[10].y, landmarks[10].z)
        )

        # Ring MCP (index 13)
        angle_ring_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[13].x, landmarks[13].y, landmarks[13].z),
            (landmarks[14].x, landmarks[14].y, landmarks[14].z)
        )

        # Pinky MCP (index 17)
        angle_pinky_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[17].x, landmarks[17].y, landmarks[17].z),
            (landmarks[18].x, landmarks[18].y, landmarks[18].z)
        )

        # --- Map Joint Angles to Servo Commands ---
        # For servo 1 (Thumb): direct mapping.
        servo1 = int(angle_thumb_mcp)
        # For servos 2-5 (Index, Middle, Ring, Pinky): invert the mapping.
        servo2 = int(180 - angle_index_mcp)
        servo3 = int(180 - angle_middle_mcp)
        servo4 = int(180 - angle_ring_mcp)
        servo5 = int(180 - angle_pinky_mcp)

        # Create a command string (e.g., "30,150,140,130,120\n").
        command = f"{servo1},{servo2},{servo3},{servo4},{servo5}\n"
        arduino.write(command.encode())
        print("Sent servo command:", command)

    else:
        # If no hand detected, reset the tracker.
        tracked_hand_center = None

    cv2.imshow("Hand Detection with MCP Mapping", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup.
camera.release()
cv2.destroyAllWindows()