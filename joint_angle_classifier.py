import cv2
import mediapipe as mp
import time
import math
from openpyxl import load_workbook
import numpy as np
from tensorflow.keras.models import load_model
import joblib

wb = load_workbook("C:/Users/joela/Downloads/Senior_Project/joint_angle_data.xlsx")
ws = wb.active

# model = load_model("C:/Users/joela/Downloads/Senior_Project/1_2_joint_angle.h5")
# le = joblib.load("C:/Users/joela/Downloads/Senior_Project/1_2_joint_angle.pkl")
model = load_model("C:/Users/joela/Downloads/Senior_Project/1_10_joint_angle.h5")
le = joblib.load("C:/Users/joela/Downloads/Senior_Project/1_10_joint_angle.pkl")

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_drawing = mp.solutions.drawing_utils

# For naming landmarks
landmark_names = [
    "Wrist", "Thumb_CMC", "Thumb_MCP", "Thumb_SIP", "Thumb_Tip",
    "Index_MCP", "Index_PIP", "Index_DIP", "Index_Tip",
    "Middle_MCP", "Middle_PIP", "Middle_DIP", "Middle_Tip",
    "Ring_MCP", "Ring_PIP", "Ring_DIP", "Ring_Tip",
    "Pinky_MCP", "Pinky_PIP", "Pinky_DIP", "Pinky_Tip"
]


# Function to calculate the angle (in degrees) at point b given points a, b, c.
def calculate_angle(a, b, c):
    # a, b, c are tuples (x, y, z)
    ba = [a[i] - b[i] for i in range(3)]
    bc = [c[i] - b[i] for i in range(3)]
    dot_product = sum(ba[i] * bc[i] for i in range(3))
    norm_ba = math.sqrt(sum(ba[i] ** 2 for i in range(3)))
    norm_bc = math.sqrt(sum(bc[i] ** 2 for i in range(3)))
    if norm_ba * norm_bc == 0:
        return 0.0
    cosine_angle = dot_product / (norm_ba * norm_bc)
    # Clamp cosine to avoid numerical errors
    cosine_angle = max(min(cosine_angle, 1.0), -1.0)
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

# This variable will hold the center (in pixel coordinates) of the originally detected hand.
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
        # Compute the center (average x,y) for each detected hand.
        hand_centers = []
        for hand_landmarks in results.multi_hand_landmarks:
            cx = int(sum([lm.x for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark) * w)
            cy = int(sum([lm.y for lm in hand_landmarks.landmark]) / len(hand_landmarks.landmark) * h)
            hand_centers.append((cx, cy))

        # If we have not yet locked onto a hand, choose the first one.
        if tracked_hand_center is None:
            tracked_hand_center = hand_centers[0]
            selected_hand = results.multi_hand_landmarks[0]
        else:
            # Find the hand whose center is closest to the tracked center.
            best_distance = float('inf')
            best_index = 0
            for i, center in enumerate(hand_centers):
                dx = center[0] - tracked_hand_center[0]
                dy = center[1] - tracked_hand_center[1]
                distance = math.sqrt(dx * dx + dy * dy)
                if distance < best_distance:
                    best_distance = distance
                    best_index = i
            selected_hand = results.multi_hand_landmarks[best_index]
            # Update tracked center.
            tracked_hand_center = hand_centers[best_index]

        # Draw bounding box and landmarks for the selected hand.
        padding = 50
        x_min = int(min([landmark.x for landmark in hand_landmarks.landmark]) * w) - padding
        x_max = int(max([landmark.x for landmark in hand_landmarks.landmark]) * w) + padding
        y_min = int(min([landmark.y for landmark in hand_landmarks.landmark]) * h) - padding
        y_max = int(max([landmark.y for landmark in hand_landmarks.landmark]) * h) + padding
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        mp_drawing.draw_landmarks(frame, selected_hand, mp_hands.HAND_CONNECTIONS)

        # Shortcut to landmarks.
        landmarks = selected_hand.landmark

        # --- Compute Joint Angles ---
        # For Thumb (using indices 0,1,2,3,4)
        angle_thumb_cmc = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[1].x, landmarks[1].y, landmarks[1].z),
            (landmarks[2].x, landmarks[2].y, landmarks[2].z)
        )
        angle_thumb_mcp = calculate_angle(
            (landmarks[1].x, landmarks[1].y, landmarks[1].z),
            (landmarks[2].x, landmarks[2].y, landmarks[2].z),
            (landmarks[3].x, landmarks[3].y, landmarks[3].z)
        )
        angle_thumb_sip = calculate_angle(
            (landmarks[2].x, landmarks[2].y, landmarks[2].z),
            (landmarks[3].x, landmarks[3].y, landmarks[3].z),
            (landmarks[4].x, landmarks[4].y, landmarks[4].z)
        )

        # For Index Finger (using indices 0,5,6,7,8)
        angle_index_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[5].x, landmarks[5].y, landmarks[5].z),
            (landmarks[6].x, landmarks[6].y, landmarks[6].z)
        )
        angle_index_pip = calculate_angle(
            (landmarks[5].x, landmarks[5].y, landmarks[5].z),
            (landmarks[6].x, landmarks[6].y, landmarks[6].z),
            (landmarks[7].x, landmarks[7].y, landmarks[7].z)
        )
        angle_index_dip = calculate_angle(
            (landmarks[6].x, landmarks[6].y, landmarks[6].z),
            (landmarks[7].x, landmarks[7].y, landmarks[7].z),
            (landmarks[8].x, landmarks[8].y, landmarks[8].z)
        )

        # For Middle Finger (using indices 0,9,10,11,12)
        angle_middle_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[9].x, landmarks[9].y, landmarks[9].z),
            (landmarks[10].x, landmarks[10].y, landmarks[10].z)
        )
        angle_middle_pip = calculate_angle(
            (landmarks[9].x, landmarks[9].y, landmarks[9].z),
            (landmarks[10].x, landmarks[10].y, landmarks[10].z),
            (landmarks[11].x, landmarks[11].y, landmarks[11].z)
        )
        angle_middle_dip = calculate_angle(
            (landmarks[10].x, landmarks[10].y, landmarks[10].z),
            (landmarks[11].x, landmarks[11].y, landmarks[11].z),
            (landmarks[12].x, landmarks[12].y, landmarks[12].z)
        )

        # For Ring Finger (using indices 0,13,14,15,16)
        angle_ring_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[13].x, landmarks[13].y, landmarks[13].z),
            (landmarks[14].x, landmarks[14].y, landmarks[14].z)
        )
        angle_ring_pip = calculate_angle(
            (landmarks[13].x, landmarks[13].y, landmarks[13].z),
            (landmarks[14].x, landmarks[14].y, landmarks[14].z),
            (landmarks[15].x, landmarks[15].y, landmarks[15].z)
        )
        angle_ring_dip = calculate_angle(
            (landmarks[14].x, landmarks[14].y, landmarks[14].z),
            (landmarks[15].x, landmarks[15].y, landmarks[15].z),
            (landmarks[16].x, landmarks[16].y, landmarks[16].z)
        )

        # For Pinky Finger (using indices 0,17,18,19,20)
        angle_pinky_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[17].x, landmarks[17].y, landmarks[17].z),
            (landmarks[18].x, landmarks[18].y, landmarks[18].z)
        )
        angle_pinky_pip = calculate_angle(
            (landmarks[17].x, landmarks[17].y, landmarks[17].z),
            (landmarks[18].x, landmarks[18].y, landmarks[18].z),
            (landmarks[19].x, landmarks[19].y, landmarks[19].z)
        )
        angle_pinky_dip = calculate_angle(
            (landmarks[18].x, landmarks[18].y, landmarks[18].z),
            (landmarks[19].x, landmarks[19].y, landmarks[19].z),
            (landmarks[20].x, landmarks[20].y, landmarks[20].z)
        )

        # --- Overlay the angles near the joint vertices ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        color = (0, 255, 255)

        # Thumb
        pt_thumb_cmc = to_pixel_coords(landmarks[1], w, h)
        pt_thumb_mcp = to_pixel_coords(landmarks[2], w, h)
        pt_thumb_sip = to_pixel_coords(landmarks[3], w, h)
        cv2.putText(frame, f"{angle_thumb_cmc:.0f}", (pt_thumb_cmc[0], pt_thumb_cmc[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_thumb_mcp:.0f}", (pt_thumb_mcp[0], pt_thumb_mcp[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_thumb_sip:.0f}", (pt_thumb_sip[0], pt_thumb_sip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Index Finger
        pt_index_mcp = to_pixel_coords(landmarks[5], w, h)
        pt_index_pip = to_pixel_coords(landmarks[6], w, h)
        pt_index_dip = to_pixel_coords(landmarks[7], w, h)
        cv2.putText(frame, f"{angle_index_mcp:.0f}", (pt_index_mcp[0], pt_index_mcp[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_index_pip:.0f}", (pt_index_pip[0], pt_index_pip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_index_dip:.0f}", (pt_index_dip[0], pt_index_dip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Middle Finger
        pt_middle_mcp = to_pixel_coords(landmarks[9], w, h)
        pt_middle_pip = to_pixel_coords(landmarks[10], w, h)
        pt_middle_dip = to_pixel_coords(landmarks[11], w, h)
        cv2.putText(frame, f"{angle_middle_mcp:.0f}", (pt_middle_mcp[0], pt_middle_mcp[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_middle_pip:.0f}", (pt_middle_pip[0], pt_middle_pip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_middle_dip:.0f}", (pt_middle_dip[0], pt_middle_dip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Ring Finger
        pt_ring_mcp = to_pixel_coords(landmarks[13], w, h)
        pt_ring_pip = to_pixel_coords(landmarks[14], w, h)
        pt_ring_dip = to_pixel_coords(landmarks[15], w, h)
        cv2.putText(frame, f"{angle_ring_mcp:.0f}", (pt_ring_mcp[0], pt_ring_mcp[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_ring_pip:.0f}", (pt_ring_pip[0], pt_ring_pip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_ring_dip:.0f}", (pt_ring_dip[0], pt_ring_dip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Pinky Finger
        pt_pinky_mcp = to_pixel_coords(landmarks[17], w, h)
        pt_pinky_pip = to_pixel_coords(landmarks[18], w, h)
        pt_pinky_dip = to_pixel_coords(landmarks[19], w, h)
        cv2.putText(frame, f"{angle_pinky_mcp:.0f}", (pt_pinky_mcp[0], pt_pinky_mcp[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_pinky_pip:.0f}", (pt_pinky_pip[0], pt_pinky_pip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"{angle_pinky_dip:.0f}", (pt_pinky_dip[0], pt_pinky_dip[1] - 10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        angles = [angle_thumb_cmc, angle_thumb_mcp, angle_thumb_sip, angle_index_mcp, angle_index_pip, angle_index_dip,
                  angle_middle_mcp, angle_middle_pip, angle_middle_dip, angle_ring_mcp, angle_ring_pip, angle_ring_dip,
                  angle_pinky_mcp, angle_pinky_dip, angle_pinky_pip]
        angles_np = np.array(angles).reshape(1, -1)  # shape = (1, 15)

        pred_prob = model.predict(angles_np)
        pred_index = np.argmax(pred_prob, axis=1)
        pred_label = le.inverse_transform(pred_index)
        print("Predicted class:", pred_label[0])
    else:
        # No hand detected: clear the tracker so that when a hand reappears it will be re-selected.
        tracked_hand_center = None

    # Show the output frame.
    cv2.imshow("Hand Detection with Persistent Tracking", frame)

    # Detect key press:
    key = cv2.waitKey(1) & 0xFF
    # if key != 255:
    #     # print(f"Key pressed: {chr(key)}")
    #     keypress = chr(key)
    #     # print(angles)
    #     # print(len(angles))

# Cleanup.
camera.release()
cv2.destroyAllWindows()
