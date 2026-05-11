import cv2
import mediapipe as mp
import time
import numpy as np
from rustypot import Scs0009PyController
import serial

# =========================
# FSR Serial SETUP
# =========================
FORCE_LIMIT = 600
fsrSerial = serial.Serial("COM4", 115200, timeout=0.1)
fsrForce = [0, 0, 0, 0]

finger_map = {
    "thumb": 0,
    "index": 1,
    "middle": 2,
    "ring": 3
}

last_sent = {
    "index": 0.0,
    "middle": 0.0,
    "ring": 0.0,
    "thumb": 0.0
}

def read_fsr():
    global fsrForce

    line = fsrSerial.readline().decode(errors='ignore').strip()
    if not line:
        return

    parts = line.split(',')
    if len(parts) != 4:
        return

    try:
        fsrForce = [int(x) for x in parts]
    except:
        pass

def apply_force_limit(finger, target_angle):
    f = fsrForce[finger_map[finger]]

    if f > FORCE_LIMIT:
        # stop closing further
        return max(target_angle, last_sent[finger])

    return target_angle

def send_led_status():
    max_force = max(fsrForce)

    overload = max_force > FORCE_LIMIT

    white = 100 if not overload else 0
    red = 100 if overload else 0

    try:
        fsrSerial.write(f"{white},{red}\n".encode())
    except:
        pass

# =========================
# ROBOT SETUP
# =========================
MaxSpeed = 7
CloseSpeed = 3

MiddlePos = [1.4, 19.6, 12.6, -7.9, 13.9, 10.5, 17, -28.1] # calibrate

c = Scs0009PyController(
        serial_port="COM3",
        baudrate=1000000,
        timeout=0.5,
    )

# =========================
# ROBOT HELPER FUNCTIONS
# =========================
ID = {
    "index": (1, 2),
    "middle": (3, 4),
    "ring": (5, 6),
    "thumb": (7, 8)
}

def Move_Finger(finger, Angle_1, Angle_2, Speed):
    ID1, ID2 = ID[finger]

    c.write_goal_speed(ID1, Speed)
    time.sleep(0.0002)
    c.write_goal_speed(ID2, Speed)
    time.sleep(0.0002)
    Pos_1 = np.deg2rad(MiddlePos[ID1-1]+Angle_1)
    Pos_2 = np.deg2rad(MiddlePos[ID2-1]+Angle_2)
    c.write_goal_position(ID1, Pos_1)
    c.write_goal_position(ID2, Pos_2)
    time.sleep(0.005)


# =========================
# MEDIAPIPE SETUP
# =========================
BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

options = HandLandmarkerOptions(
    base_options=BaseOptions(model_asset_path="hand_landmarker.task"),
    running_mode=VisionRunningMode.VIDEO,
    num_hands=1
)

cap = cv2.VideoCapture(0)

# exponential smoothing w/ recursive formula: s(t) = ax(t) + (1-a)s(t-1)
filtered = {
    "index": 0.0,
    "middle": 0.0,
    "ring": 0.0,
    "thumb": 0.0
}
alpha = 0.3     # larger a -> more responsive, smaller a -> more smooth


# =========================
# ANGLE HELPERS
# =========================
def angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    ba = a - b
    bc = c - b

    cos_theta = np.dot(ba, bc) / (np.linalg.norm(ba) * np.linalg.norm(bc))
    return np.arccos(np.clip(cos_theta, -1.0, 1.0))

def normalize_angle(angle, straight, bent):
    x = (angle - straight) / (bent - straight)
    return np.clip(x, 0.0, 1.0)

def update_joint(prev, target, max_step=3.0, alpha=0.3):
    # velocity limit (prevents sudden jumps)
    #clamped = np.clip(target, prev - max_step, prev + max_step)

    # smoothing (low-pass filter)
    return alpha * target + (1 - alpha) * prev

# =========================
# LANDMARK DRAWING
# =========================
# MediaPipe hand connection pairs (21 landmarks)
HAND_CONNECTIONS = [
    # Wrist to base of each finger
    (0, 1), (0, 5), (0, 17),
    # Palm cross-connections
    (5, 9), (9, 13), (13, 17),
    # Thumb
    (1, 2), (2, 3), (3, 4),
    # Index
    (5, 6), (6, 7), (7, 8),
    # Middle
    (9, 10), (10, 11), (11, 12),
    # Ring
    (13, 14), (14, 15), (15, 16),
    # Pinky
    (17, 18), (18, 19), (19, 20),
]

def draw_hand_landmarks(frame, landmarks):
    h, w = frame.shape[:2]

    # Convert normalised coords to pixel coords
    pts = [(int(lm.x * w), int(lm.y * h)) for lm in landmarks]

    # Draw bones (green lines)
    for a, b in HAND_CONNECTIONS:
        cv2.line(frame, pts[a], pts[b], (0, 255, 80), 2, cv2.LINE_AA)

    # Draw joints (red filled circles)
    for x, y in pts:
        cv2.circle(frame, (x, y), 3, (0, 0, 220), -1, cv2.LINE_AA)
        # Thin white border so dots pop on any background
        cv2.circle(frame, (x, y), 3, (255, 255, 255), 1, cv2.LINE_AA)


# =========================
# MAIN LOOP
# =========================

with HandLandmarker.create_from_options(options) as landmarker:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        mp_image = mp.Image(
            image_format=mp.ImageFormat.SRGB,
            data=frame
        )

        result = landmarker.detect_for_video(mp_image, int(cap.get(cv2.CAP_PROP_POS_MSEC)))

        # =========================
        # HAND TRACKING
        # =========================
        if result.hand_landmarks:
            landmarks = result.hand_landmarks[0]
            lm = [(p.x, p.y) for p in landmarks]

            # Get angle w/ 3 joint calculation from landmarks (refer to png file)
            index_pip_flex = normalize_angle(angle(lm[5], lm[6], lm[8]), 3.14, 1.2)
            index_pip = -(index_pip_flex * 125 - 35)
            filtered["index"] = update_joint(filtered["index"], index_pip)

            middle_pip_flex = normalize_angle(angle(lm[9], lm[10], lm[12]), 3.14, 1.2)
            middle_pip = -(middle_pip_flex * 125 - 35)
            filtered["middle"] = update_joint(filtered["middle"], middle_pip)

            ring_pip_flex = normalize_angle(angle(lm[13], lm[14], lm[16]), 3.14, 1.2)
            ring_pip = -(ring_pip_flex * 125 - 35)
            filtered["ring"] = update_joint(filtered["ring"], ring_pip)

            thumb_pip_flex = normalize_angle(angle(lm[2], lm[3], lm[4]), 3, 2)
            thumb_pip = -(thumb_pip_flex * 125 - 35)
            filtered["thumb"] = update_joint(filtered["thumb"], thumb_pip)

            # Draw skeleton overlay on the frame
            draw_hand_landmarks(frame, landmarks)
        
        # =========================
        # CONTROL HAND
        # =========================
        read_fsr()

        index_angle = apply_force_limit("index",    filtered["index"])
        middle_angle = apply_force_limit("middle",  filtered["middle"])
        ring_angle = apply_force_limit("ring",      filtered["ring"])
        thumb_angle = apply_force_limit("thumb",    filtered["thumb"])

        last_sent["index"]  = index_angle
        last_sent["middle"] = middle_angle
        last_sent["ring"]   = ring_angle
        last_sent["thumb"]  = thumb_angle

        Move_Finger("index",    -index_angle,   index_angle,    MaxSpeed-2)
        Move_Finger("middle",   -middle_angle,  middle_angle,   MaxSpeed-2)
        Move_Finger("ring",     -ring_angle,    ring_angle,     MaxSpeed-2)
        Move_Finger("thumb",    -thumb_angle,   thumb_angle,    MaxSpeed-2)

        send_led_status()

        # print all finger angles
        # print(
        #     f"I:{filtered['index']:.1f} | "
        #     f"M:{filtered['middle']:.1f} | "
        #     f"R:{filtered['ring']:.1f} | "
        #     f"T:{filtered['thumb']:.1f}"
        # )
        print(
            f"I:{fsrForce[finger_map['index']]} | "
            f"M:{fsrForce[finger_map['middle']]} | "
            f"R:{fsrForce[finger_map['ring']]} | "
            f"T:{fsrForce[finger_map['thumb']]}"
        )

        # break with ESC
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) == 27:
            break


cap.release()
cv2.destroyAllWindows()
