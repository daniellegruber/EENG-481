import cv2
import mediapipe as mp
import numpy as np
import time
import math

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

def put_line(img, text, y, font, font_scale=1, thickness=1, x_offset=10, color=(0, 0, 0), line_height=30):
    cv2.putText(img, text, (x_offset, y), font, font_scale, color, thickness, cv2.LINE_AA)
    return y + line_height

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
                distance = math.sqrt(dx*dx + dy*dy)
                if distance < best_distance:
                    best_distance = distance
                    best_index = i
            selected_hand = results.multi_hand_landmarks[best_index]
            # Update tracked center.
            tracked_hand_center = hand_centers[best_index]

        # Draw bounding box and landmarks for the selected hand.
        x_min = int(min([lm.x for lm in selected_hand.landmark]) * w)
        x_max = int(max([lm.x for lm in selected_hand.landmark]) * w)
        y_min = int(min([lm.y for lm in selected_hand.landmark]) * h)
        y_max = int(max([lm.y for lm in selected_hand.landmark]) * h)
        cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        mp_drawing.draw_landmarks(frame, selected_hand, mp_hands.HAND_CONNECTIONS)

        # Shortcut to landmarks.
        landmarks = selected_hand.landmark

        # --- Compute Joint Angles ---
        # For Thumb (using indices 0,1,2,3,4)
        print("<---------- Thumb ---------->")
        angle_thumb_cmc = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[1].x, landmarks[1].y, landmarks[1].z),
            (landmarks[2].x, landmarks[2].y, landmarks[2].z)
        )
        print(f"CMC: {angle_thumb_cmc}")
        angle_thumb_mcp = calculate_angle(
            (landmarks[1].x, landmarks[1].y, landmarks[1].z),
            (landmarks[2].x, landmarks[2].y, landmarks[2].z),
            (landmarks[3].x, landmarks[3].y, landmarks[3].z)
        )
        print(f"MCP: {angle_thumb_mcp}")
        angle_thumb_sip = calculate_angle(
            (landmarks[2].x, landmarks[2].y, landmarks[2].z),
            (landmarks[3].x, landmarks[3].y, landmarks[3].z),
            (landmarks[4].x, landmarks[4].y, landmarks[4].z)
        )
        print(f"SIP: {angle_thumb_sip}")

        # For Index Finger (using indices 0,5,6,7,8)
        print("<---------- Index ---------->")
        angle_index_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[5].x, landmarks[5].y, landmarks[5].z),
            (landmarks[6].x, landmarks[6].y, landmarks[6].z)
        )
        print(f"MCP: {angle_index_mcp}")
        angle_index_pip = calculate_angle(
            (landmarks[5].x, landmarks[5].y, landmarks[5].z),
            (landmarks[6].x, landmarks[6].y, landmarks[6].z),
            (landmarks[7].x, landmarks[7].y, landmarks[7].z)
        )
        print(f"PIP: {angle_index_pip}")
        angle_index_dip = calculate_angle(
            (landmarks[6].x, landmarks[6].y, landmarks[6].z),
            (landmarks[7].x, landmarks[7].y, landmarks[7].z),
            (landmarks[8].x, landmarks[8].y, landmarks[8].z)
        )
        print(f"DIP: {angle_index_dip}")

        # For Middle Finger (using indices 0,9,10,11,12)
        print("<---------- Middle ---------->")
        angle_middle_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[9].x, landmarks[9].y, landmarks[9].z),
            (landmarks[10].x, landmarks[10].y, landmarks[10].z)
        )
        print(f"MCP: {angle_middle_mcp}")
        angle_middle_pip = calculate_angle(
            (landmarks[9].x, landmarks[9].y, landmarks[9].z),
            (landmarks[10].x, landmarks[10].y, landmarks[10].z),
            (landmarks[11].x, landmarks[11].y, landmarks[11].z)
        )
        print(f"PIP: {angle_middle_pip}")
        angle_middle_dip = calculate_angle(
            (landmarks[10].x, landmarks[10].y, landmarks[10].z),
            (landmarks[11].x, landmarks[11].y, landmarks[11].z),
            (landmarks[12].x, landmarks[12].y, landmarks[12].z)
        )
        print(f"DIP: {angle_middle_dip}")

        # For Ring Finger (using indices 0,13,14,15,16)
        print("<---------- Ring ---------->")
        angle_ring_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[13].x, landmarks[13].y, landmarks[13].z),
            (landmarks[14].x, landmarks[14].y, landmarks[14].z)
        )
        print(f"MCP: {angle_ring_mcp}")
        angle_ring_pip = calculate_angle(
            (landmarks[13].x, landmarks[13].y, landmarks[13].z),
            (landmarks[14].x, landmarks[14].y, landmarks[14].z),
            (landmarks[15].x, landmarks[15].y, landmarks[15].z)
        )
        print(f"PIP: {angle_ring_pip}")
        angle_ring_dip = calculate_angle(
            (landmarks[14].x, landmarks[14].y, landmarks[14].z),
            (landmarks[15].x, landmarks[15].y, landmarks[15].z),
            (landmarks[16].x, landmarks[16].y, landmarks[16].z)
        )
        print(f"DIP: {angle_ring_dip}")

        # For Pinky Finger (using indices 0,17,18,19,20)
        print("<---------- Pinky ---------->")
        angle_pinky_mcp = calculate_angle(
            (landmarks[0].x, landmarks[0].y, landmarks[0].z),
            (landmarks[17].x, landmarks[17].y, landmarks[17].z),
            (landmarks[18].x, landmarks[18].y, landmarks[18].z)
        )
        print(f"MCP: {angle_pinky_mcp}")
        angle_pinky_pip = calculate_angle(
            (landmarks[17].x, landmarks[17].y, landmarks[17].z),
            (landmarks[18].x, landmarks[18].y, landmarks[18].z),
            (landmarks[19].x, landmarks[19].y, landmarks[19].z)
        )
        print(f"PIP: {angle_pinky_pip}")
        angle_pinky_dip = calculate_angle(
            (landmarks[18].x, landmarks[18].y, landmarks[18].z),
            (landmarks[19].x, landmarks[19].y, landmarks[19].z),
            (landmarks[20].x, landmarks[20].y, landmarks[20].z)
        )
        print(f"DIP: {angle_pinky_dip}")
        joint_angles = {"Thumb": [angle_thumb_cmc, angle_thumb_mcp, angle_thumb_sip], 
                        "Index": [angle_index_mcp, angle_index_pip, angle_index_dip], 
                        "Middle": [angle_middle_mcp, angle_middle_pip, angle_middle_dip], 
                        "Ring": [angle_ring_mcp, angle_ring_pip, angle_ring_dip], 
                        "Pinky": [angle_pinky_mcp, angle_pinky_pip, angle_pinky_dip]}

        print("<---------- Utility ---------->")
        # Get wrist and thumb tip landmarks
        wrist = landmarks[0]
        thumb_tip = landmarks[4]

        # Compute XYZ relative coordinates of Thumb Tip to Wrist
        relative_x = thumb_tip.x - wrist.x
        relative_y = thumb_tip.y - wrist.y
        relative_z = thumb_tip.z - wrist.z

        # Print coordinates
        print(f"Wrist Absolute -> X: {wrist.x:.4f}, Y: {wrist.y:.4f}, Z: {wrist.z:.4f}")
        print(f"Thumb Tip Relative to Wrist -> X: {relative_x:.4f}, Y: {relative_y:.4f}, Z: {relative_z:.4f}")

        # --- Overlay the angles near the joint vertices ---
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1
        color = (0, 255, 255)

        # Thumb
        pt_thumb_cmc = to_pixel_coords(landmarks[1], w, h)
        pt_thumb_mcp = to_pixel_coords(landmarks[2], w, h)
        pt_thumb_sip = to_pixel_coords(landmarks[3], w, h)
        cv2.putText(frame, f"CMC: {angle_thumb_cmc:.0f}", (pt_thumb_cmc[0], pt_thumb_cmc[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"MCP: {angle_thumb_mcp:.0f}", (pt_thumb_mcp[0], pt_thumb_mcp[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"SIP: {angle_thumb_sip:.0f}", (pt_thumb_sip[0], pt_thumb_sip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Index Finger
        pt_index_mcp = to_pixel_coords(landmarks[5], w, h)
        pt_index_pip = to_pixel_coords(landmarks[6], w, h)
        pt_index_dip = to_pixel_coords(landmarks[7], w, h)
        cv2.putText(frame, f"MCP: {angle_index_mcp:.0f}", (pt_index_mcp[0], pt_index_mcp[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"PIP: {angle_index_pip:.0f}", (pt_index_pip[0], pt_index_pip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"DIP: {angle_index_dip:.0f}", (pt_index_dip[0], pt_index_dip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Middle Finger
        pt_middle_mcp = to_pixel_coords(landmarks[9], w, h)
        pt_middle_pip = to_pixel_coords(landmarks[10], w, h)
        pt_middle_dip = to_pixel_coords(landmarks[11], w, h)
        cv2.putText(frame, f"MCP: {angle_middle_mcp:.0f}", (pt_middle_mcp[0], pt_middle_mcp[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"PIP: {angle_middle_pip:.0f}", (pt_middle_pip[0], pt_middle_pip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"DIP: {angle_middle_dip:.0f}", (pt_middle_dip[0], pt_middle_dip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Ring Finger
        pt_ring_mcp = to_pixel_coords(landmarks[13], w, h)
        pt_ring_pip = to_pixel_coords(landmarks[14], w, h)
        pt_ring_dip = to_pixel_coords(landmarks[15], w, h)
        cv2.putText(frame, f"MCP: {angle_ring_mcp:.0f}", (pt_ring_mcp[0], pt_ring_mcp[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"PIP: {angle_ring_pip:.0f}", (pt_ring_pip[0], pt_ring_pip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"DIP: {angle_ring_dip:.0f}", (pt_ring_dip[0], pt_ring_dip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Pinky Finger
        pt_pinky_mcp = to_pixel_coords(landmarks[17], w, h)
        pt_pinky_pip = to_pixel_coords(landmarks[18], w, h)
        pt_pinky_dip = to_pixel_coords(landmarks[19], w, h)
        cv2.putText(frame, f"MCP: {angle_pinky_mcp:.0f}", (pt_pinky_mcp[0], pt_pinky_mcp[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"PIP: {angle_pinky_pip:.0f}", (pt_pinky_pip[0], pt_pinky_pip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)
        cv2.putText(frame, f"DIP: {angle_pinky_dip:.0f}", (pt_pinky_dip[0], pt_pinky_dip[1]-10),
                    font, font_scale, color, thickness, cv2.LINE_AA)

        # Utility
        # Convert wrist and thumb tip to pixel coordinates
        pt_wrist = to_pixel_coords(wrist, w, h)
        pt_thumb_tip = to_pixel_coords(thumb_tip, w, h)

        # Overlay Wrist coordinates
        cv2.putText(frame, f"Wrist X: {wrist.x:.3f}", (pt_wrist[0] - 50, pt_wrist[1] - 20),
                    font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
        cv2.putText(frame, f"Wrist Y: {wrist.y:.3f}", (pt_wrist[0] - 50, pt_wrist[1] - 5),
                    font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
        cv2.putText(frame, f"Wrist Z: {wrist.z:.3f}", (pt_wrist[0] - 50, pt_wrist[1] + 10),
                    font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)

        # Overlay Thumb Tip relative coordinates
        cv2.putText(frame, f"X: {relative_x:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1] - 30),
                    font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
        cv2.putText(frame, f"Y: {relative_y:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1] - 15),
                    font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
        cv2.putText(frame, f"Z: {relative_z:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1]),
                    font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
        # Create side panel
        panel_width = 300
        panel = 255 * np.ones((h, panel_width, 3), dtype=np.uint8)
        y_pos = 30
        y_pos = put_line(panel, "Joint Angles", y_pos, font, color=(0, 0, 255), line_height=50)

        for finger, angles in joint_angles.items():
            y_pos = put_line(panel, f"<{finger}>", y_pos, font)
            for name, angle in zip(["Joint1", "Joint2", "Joint3"], angles):
                y_pos = put_line(panel, f"{name}: {angle:.1f} deg", y_pos, font)

        combined = np.hstack((frame, panel))
        cv2.imshow("Hand Detection with Angles", combined)
    else:
        tracked_hand_center = None
        cv2.imshow("Hand Detection with Angles", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Cleanup.
camera.release()
cv2.destroyAllWindows()