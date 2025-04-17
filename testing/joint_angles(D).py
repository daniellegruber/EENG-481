import cv2
import mediapipe as mp
import time
import math
import pyrealsense2 as rs
import numpy as np

# Initialize MediaPipe Hands.
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=2,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)
mp_drawing = mp.solutions.drawing_utils

# For naming landmarks (optional).
landmark_names = [
    "Wrist", "Thumb_CMC", "Thumb_MCP", "Thumb_SIP", "Thumb_Tip",
    "Index_MCP", "Index_PIP", "Index_DIP", "Index_Tip",
    "Middle_MCP", "Middle_PIP", "Middle_DIP", "Middle_Tip",
    "Ring_MCP", "Ring_PIP", "Ring_DIP", "Ring_Tip",
    "Pinky_MCP", "Pinky_PIP", "Pinky_DIP", "Pinky_Tip"
]

# Function to calculate the angle (in degrees) at point b given points a, b, and c.
def calculate_angle(a, b, c):
    # a, b, c are tuples/lists (x, y, z)
    ba = [a[i] - b[i] for i in range(3)]
    bc = [c[i] - b[i] for i in range(3)]
    dot_product = sum(ba[i] * bc[i] for i in range(3))
    norm_ba = math.sqrt(sum(ba[i] ** 2 for i in range(3)))
    norm_bc = math.sqrt(sum(bc[i] ** 2 for i in range(3)))
    if norm_ba * norm_bc == 0:
        return 0.0
    cosine_angle = dot_product / (norm_ba * norm_bc)
    # Clamp cosine to avoid numerical errors.
    cosine_angle = max(min(cosine_angle, 1.0), -1.0)
    angle = math.acos(cosine_angle)
    return math.degrees(angle)

# Helper to convert normalized landmark coordinates to pixel coordinates.
def to_pixel_coords(landmark, width, height):
    return int(landmark.x * width), int(landmark.y * height)

# --- Setup RealSense Pipeline ---
pipeline = rs.pipeline()
config = rs.config()
# Configure the streams: adjust resolution and framerate as needed.
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
# Align depth frame to color frame.
align = rs.align(rs.stream.color)

tracked_hand_center = None

try:
    while True:
        # Wait for a coherent pair of frames: depth and color.
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert color frame to a numpy array.
        frame = np.asanyarray(color_frame.get_data())
        # Flip the frame for a mirror view.
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
            
            # If no hand is currently tracked, select the first detected hand.
            if tracked_hand_center is None:
                tracked_hand_center = hand_centers[0]
                selected_hand = results.multi_hand_landmarks[0]
            else:
                # Find the hand whose center is closest to the previously tracked center.
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
                tracked_hand_center = hand_centers[best_index]

            # Draw a bounding box and landmarks for the selected hand.
            x_min = int(min([lm.x for lm in selected_hand.landmark]) * w)
            x_max = int(max([lm.x for lm in selected_hand.landmark]) * w)
            y_min = int(min([lm.y for lm in selected_hand.landmark]) * h)
            y_max = int(max([lm.y for lm in selected_hand.landmark]) * h)
            cv2.rectangle(frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
            mp_drawing.draw_landmarks(frame, selected_hand, mp_hands.HAND_CONNECTIONS)

            # For each landmark, get the 3D coordinate using depth.
            # We use the RealSense intrinsics from the depth frame.
            intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
            landmarks_3d = []
            for lm in selected_hand.landmark:
                px, py = to_pixel_coords(lm, w, h)
                depth_value = depth_frame.get_distance(px, py)
                point_3d = rs.rs2_deproject_pixel_to_point(intrinsics, [px, py], depth_value)
                landmarks_3d.append(point_3d)

            # --- Compute Joint Angles using 3D points ---
            # For Thumb (using indices 0,1,2,3,4)
            print("<---------- Thumb ---------->")
            angle_thumb_cmc = calculate_angle(landmarks_3d[0], landmarks_3d[1], landmarks_3d[2])
            print(f"CMC: {angle_thumb_cmc}")
            angle_thumb_mcp = calculate_angle(landmarks_3d[1], landmarks_3d[2], landmarks_3d[3])
            print(f"MCP: {angle_thumb_mcp}")
            angle_thumb_sip = calculate_angle(landmarks_3d[2], landmarks_3d[3], landmarks_3d[4])
            print(f"SIP: {angle_thumb_sip}")

            # For Index Finger (using indices 0,5,6,7,8)
            print("<---------- Index ---------->")
            angle_index_mcp = calculate_angle(landmarks_3d[0], landmarks_3d[5], landmarks_3d[6])
            print(f"MCP: {angle_index_mcp}")
            angle_index_pip = calculate_angle(landmarks_3d[5], landmarks_3d[6], landmarks_3d[7])
            print(f"PIP: {angle_index_pip}")
            angle_index_dip = calculate_angle(landmarks_3d[6], landmarks_3d[7], landmarks_3d[8])
            print(f"DIP: {angle_index_dip}")

            # For Middle Finger (using indices 0,9,10,11,12)
            print("<---------- Middle ---------->")
            angle_middle_mcp = calculate_angle(landmarks_3d[0], landmarks_3d[9], landmarks_3d[10])
            print(f"MCP: {angle_middle_mcp}")
            angle_middle_pip = calculate_angle(landmarks_3d[9], landmarks_3d[10], landmarks_3d[11])
            print(f"PIP: {angle_middle_pip}")
            angle_middle_dip = calculate_angle(landmarks_3d[10], landmarks_3d[11], landmarks_3d[12])
            print(f"DIP: {angle_middle_dip}")

            # For Ring Finger (using indices 0,13,14,15,16)
            print("<---------- Ring ---------->")
            angle_ring_mcp = calculate_angle(landmarks_3d[0], landmarks_3d[13], landmarks_3d[14])
            print(f"MCP: {angle_ring_mcp}")
            angle_ring_pip = calculate_angle(landmarks_3d[13], landmarks_3d[14], landmarks_3d[15])
            print(f"PIP: {angle_ring_pip}")
            angle_ring_dip = calculate_angle(landmarks_3d[14], landmarks_3d[15], landmarks_3d[16])
            print(f"DIP: {angle_ring_dip}")

            # For Pinky Finger (using indices 0,17,18,19,20)
            print("<---------- Pinky ---------->")
            angle_pinky_mcp = calculate_angle(landmarks_3d[0], landmarks_3d[17], landmarks_3d[18])
            print(f"MCP: {angle_pinky_mcp}")
            angle_pinky_pip = calculate_angle(landmarks_3d[17], landmarks_3d[18], landmarks_3d[19])
            print(f"PIP: {angle_pinky_pip}")
            angle_pinky_dip = calculate_angle(landmarks_3d[18], landmarks_3d[19], landmarks_3d[20])
            print(f"DIP: {angle_pinky_dip}")

            print("<---------- Utility ---------->")
            # For utility display, use the 2D pixel coordinates for overlay,
            # but use the 3D deprojected values for calculation.
            wrist = selected_hand.landmark[0]
            thumb_tip = selected_hand.landmark[4]
            pt_wrist = to_pixel_coords(wrist, w, h)
            pt_thumb_tip = to_pixel_coords(thumb_tip, w, h)

            # Get 3D coordinates for wrist and thumb tip.
            wrist_3d = landmarks_3d[0]
            thumb_tip_3d = landmarks_3d[4]
            # Compute relative coordinates of Thumb Tip to Wrist.
            relative_x = thumb_tip_3d[0] - wrist_3d[0]
            relative_y = thumb_tip_3d[1] - wrist_3d[1]
            relative_z = thumb_tip_3d[2] - wrist_3d[2]

            print(f"Wrist 3D -> X: {wrist_3d[0]:.4f}, Y: {wrist_3d[1]:.4f}, Z: {wrist_3d[2]:.4f}")
            print(f"Thumb Tip Relative to Wrist -> X: {relative_x:.4f}, Y: {relative_y:.4f}, Z: {relative_z:.4f}")

            # --- Overlay the angles near the joint vertices ---
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            color = (0, 255, 255)

            # Thumb overlays.
            pt_thumb_cmc = to_pixel_coords(selected_hand.landmark[1], w, h)
            pt_thumb_mcp = to_pixel_coords(selected_hand.landmark[2], w, h)
            pt_thumb_sip = to_pixel_coords(selected_hand.landmark[3], w, h)
            cv2.putText(frame, f"CMC: {angle_thumb_cmc:.0f}", (pt_thumb_cmc[0], pt_thumb_cmc[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"MCP: {angle_thumb_mcp:.0f}", (pt_thumb_mcp[0], pt_thumb_mcp[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"SIP: {angle_thumb_sip:.0f}", (pt_thumb_sip[0], pt_thumb_sip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)

            # Index Finger overlays.
            pt_index_mcp = to_pixel_coords(selected_hand.landmark[5], w, h)
            pt_index_pip = to_pixel_coords(selected_hand.landmark[6], w, h)
            pt_index_dip = to_pixel_coords(selected_hand.landmark[7], w, h)
            cv2.putText(frame, f"MCP: {angle_index_mcp:.0f}", (pt_index_mcp[0], pt_index_mcp[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"PIP: {angle_index_pip:.0f}", (pt_index_pip[0], pt_index_pip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"DIP: {angle_index_dip:.0f}", (pt_index_dip[0], pt_index_dip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)

            # Middle Finger overlays.
            pt_middle_mcp = to_pixel_coords(selected_hand.landmark[9], w, h)
            pt_middle_pip = to_pixel_coords(selected_hand.landmark[10], w, h)
            pt_middle_dip = to_pixel_coords(selected_hand.landmark[11], w, h)
            cv2.putText(frame, f"MCP: {angle_middle_mcp:.0f}", (pt_middle_mcp[0], pt_middle_mcp[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"PIP: {angle_middle_pip:.0f}", (pt_middle_pip[0], pt_middle_pip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"DIP: {angle_middle_dip:.0f}", (pt_middle_dip[0], pt_middle_dip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)

            # Ring Finger overlays.
            pt_ring_mcp = to_pixel_coords(selected_hand.landmark[13], w, h)
            pt_ring_pip = to_pixel_coords(selected_hand.landmark[14], w, h)
            pt_ring_dip = to_pixel_coords(selected_hand.landmark[15], w, h)
            cv2.putText(frame, f"MCP: {angle_ring_mcp:.0f}", (pt_ring_mcp[0], pt_ring_mcp[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"PIP: {angle_ring_pip:.0f}", (pt_ring_pip[0], pt_ring_pip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"DIP: {angle_ring_dip:.0f}", (pt_ring_dip[0], pt_ring_dip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)

            # Pinky Finger overlays.
            pt_pinky_mcp = to_pixel_coords(selected_hand.landmark[17], w, h)
            pt_pinky_pip = to_pixel_coords(selected_hand.landmark[18], w, h)
            pt_pinky_dip = to_pixel_coords(selected_hand.landmark[19], w, h)
            cv2.putText(frame, f"MCP: {angle_pinky_mcp:.0f}", (pt_pinky_mcp[0], pt_pinky_mcp[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"PIP: {angle_pinky_pip:.0f}", (pt_pinky_pip[0], pt_pinky_pip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)
            cv2.putText(frame, f"DIP: {angle_pinky_dip:.0f}", (pt_pinky_dip[0], pt_pinky_dip[1]-10),
                        font, font_scale, color, thickness, cv2.LINE_AA)

            # Utility overlays for wrist and thumb tip relative coordinates.
            cv2.putText(frame, f"Wrist 3D X: {wrist_3d[0]:.3f}", (pt_wrist[0] - 50, pt_wrist[1] - 20),
                        font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            cv2.putText(frame, f"Y: {wrist_3d[1]:.3f}", (pt_wrist[0] - 50, pt_wrist[1] - 5),
                        font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            cv2.putText(frame, f"Z: {wrist_3d[2]:.3f}", (pt_wrist[0] - 50, pt_wrist[1] + 10),
                        font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            cv2.putText(frame, f"Rel X: {relative_x:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1] - 30),
                        font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
            cv2.putText(frame, f"Rel Y: {relative_y:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1] - 15),
                        font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
            cv2.putText(frame, f"Rel Z: {relative_z:.3f}", (pt_thumb_tip[0] + 10, pt_thumb_tip[1]),
                        font, font_scale, (255, 0, 0), thickness, cv2.LINE_AA)
        else:
            # No hand detected: clear the tracker so that a hand is re-selected upon reappearance.
            tracked_hand_center = None

        # Display the output frame.
        cv2.imshow("Hand Detection with Depth", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    pipeline.stop()
    cv2.destroyAllWindows()