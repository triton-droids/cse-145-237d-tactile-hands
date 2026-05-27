"""
Shared VR<->ROS plumbing for the ARCTOS teleop stack.

One publisher (vr_controller_publisher.py) owns the OpenVR connection and
publishes the right controller's pose + clutch. Every consumer (teleop,
visualizer) subscribes here instead of opening OpenVR itself — so there
is exactly one SteamVR client, and N read-only consumers.

Topics:
  /vr/right_controller/pose  geometry_msgs/PoseStamped  (room frame)
  /vr/clutch                 std_msgs/Bool              (B held)

This module has NO openvr and NO python-can dependency on purpose, so
the visualizer can import it without arm libraries.
"""

import json
import math
import os
import threading
import time

import numpy as np

def ros_init_no_signals():
    """
    rclpy.init() WITHOUT rclpy's SIGINT handler.

    By default rclpy installs its own signal handlers, which swallow
    Ctrl+C so the host program's `except KeyboardInterrupt` never runs —
    for the teleop that means it never stops the arm or frees the CAN
    port on exit. The host must own SIGINT, so disable rclpy's handlers.
    Safe to call if already initialized.
    """
    import rclpy

    if rclpy.ok():
        return
    try:
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except (ImportError, TypeError):
        # Very old rclpy without the option — fall back and re-assert
        # the default Python SIGINT handler afterwards.
        import signal
        rclpy.init()
        signal.signal(signal.SIGINT, signal.default_int_handler)


def disarm_term_signals():
    """
    Make the exit path uninterruptible. Our SIGINT/SIGTERM handler
    raises KeyboardInterrupt; the FIRST one breaks the main loop, but a
    SECOND arriving during cleanup (a re-press, or the launcher's SIGINT
    then SIGKILL) re-raises through the finally — KeyboardInterrupt is
    BaseException, not Exception, so it sails past `except Exception` and
    can skip stopping the arm / freeing the port. Call this as the FIRST
    line of every cleanup/finally so teardown completes atomically. The
    launcher's SIGKILL fallback is still the ultimate backstop.
    Main-thread only; best-effort.
    """
    import signal

    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, signal.SIG_IGN)
        except (ValueError, OSError):
            pass


TOPIC_POSE = "/vr/right_controller/pose"
TOPIC_CLUTCH = "/vr/clutch"
FRAME_ID = "vr_room"           # SteamVR standing room frame (OpenGL axes)

# Hand / capacitive finger sensing (Valve Index).
TOPIC_FINGERS = "/vr/finger_curls"   # std_msgs/Float32MultiArray, len 5
TOPIC_TRIGGER = "/vr/trigger"        # std_msgs/Float32, 0..1
TOPIC_GRIP = "/vr/grip"              # std_msgs/Float32, 0..1
# Fixed order of TOPIC_FINGERS .data — SteamVR flFingerCurl order.
# Each value 0.0 = finger straight/open, 1.0 = fully curled/closed.
# The AmazingHand has no pinky; the driver uses the first four.
FINGER_ORDER = ("thumb", "index", "middle", "ring", "pinky")

# One-Euro filter spec for a noisy source.
#   mincutoff : lower  = smoother but laggier at low speed
#   beta      : higher = less lag when moving fast
# Yaw has a few degrees of jitter so it is filtered; height is steadier
# (a position source) and is left unfiltered (filter=None below).
YAW_FILTER = {"mincutoff": 1.2, "beta": 0.012, "dcutoff": 1.0}

# ===========================================================================
# AXIS MAP — the single source of truth, imported by teleop AND visualizer.
# One dict per driven joint. Field reference:
#
#   joint         CAN joint id
#   source        control input:  "yaw" = controller heading (deg)
#                                  "y"   = controller height  (m)
#   scale         joint-degrees per source-unit (deg/deg or deg/m)
#   invert        flip mapping direction if the joint goes the wrong way
#   kp            P-gain: motor rpm per degree of joint position error
#   max_travel    max |target - engage angle|, degrees  (soft clamp)
#   max_rpm       per-joint speed cap, motor rpm
#   rpm_floor     min |rpm| once outside the deadband — a loaded joint
#                 stalls below its break-away torque otherwise
#   pos_deadband  |error| within this -> command 0 (no hunting)
#   filter        One-Euro spec dict, or None for pass-through
# ===========================================================================
AXES = [
    {
        "joint":        1,
        "source":       "yaw",
        "scale":        0.25,
        "invert":       True,
        "kp":           6.0,
        "max_travel":   45.0,
        "max_rpm":      80,
        "rpm_floor":    8,
        "pos_deadband": 1.0,
        "filter":       YAW_FILTER,
    },
    # --- J2 (shoulder) DISABLED ---------------------------------------
    # Removed from AXES so the teleop sends it no commands at all.
    # Re-enable by un-commenting this block.
    # {
    #     "joint":        2,
    #     "source":       "y",
    #     "scale":        240.0,
    #     "invert":       True,
    #     "kp":           6.0,
    #     "max_travel":   1000.0,   # intentionally large: soft travel
    #                               # clamp + runaway guard effectively off
    #     "max_rpm":      240,      # 3x J1 cap (was the shared 80)
    #     "rpm_floor":    45,
    #     "pos_deadband": 2.0,
    #     "filter":       None,
    # },
    # --- J3 (elbow) DISABLED ------------------------------------------
    # Followed controller height like J2. Removed from AXES so the
    # teleop sends it no commands at all. Re-enable by un-commenting.
    # {
    #     "joint":        3,
    #     "source":       "y",
    #     "scale":        240.0,
    #     "invert":       False,
    #     "kp":           6.0,
    #     "max_travel":   1000.0,   # intentionally large: soft travel
    #                               # clamp + runaway guard effectively off
    #     "max_rpm":      240,
    #     "rpm_floor":    45,
    #     "pos_deadband": 2.0,
    #     "filter":       None,
    # },
    # --- J4 (forearm roll) DISABLED -----------------------------------
    # Roll no longer drives J4 (removed from AXES, so the teleop sends it
    # no commands at all). Re-enable by un-commenting this block.
    # {
    #     # J4 (forearm roll) <- controller roll (wrist twist). Uses the
    #     # same deadzone as J1 (SRC_DEADBAND["roll"] = YAW_DEADBAND_DEG).
    #     "joint":        4,
    #     "source":       "roll",
    #     "scale":        1.0,
    #     "invert":       False,
    #     "kp":           6.0,
    #     "max_travel":   45.0,
    #     "max_rpm":      80,
    #     "rpm_floor":    8,
    #     "pos_deadband": 1.0,
    #     "filter":       None,
    # },
]

YAW_DEADBAND_DEG = 10.0   # +/-10° dead, then linear (deadband() is zero
                          # inside, continues from 0 at the edge — no step)
ROLL_DEADBAND_DEG = 25.0  # J4 wrist-twist triggers accidentally during
                          # normal motion; wide dead zone, then linear
Y_DEADBAND_M = 0.01
SRC_DEADBAND = {
    "yaw": YAW_DEADBAND_DEG,
    "y": Y_DEADBAND_M,
    "roll": ROLL_DEADBAND_DEG,
}
SRC_UNIT = {"yaw": "°", "y": "m", "roll": "°"}
# Angular sources wrap; their period (deg). None = linear (no wrap).
SRC_WRAP = {"yaw": 360.0, "y": None, "roll": 360.0}


# ---------------------------------------------------------------------------
# Pose math
# ---------------------------------------------------------------------------
def quat_to_mat(w, x, y, z):
    """Unit quaternion (w,x,y,z) -> 3x3 rotation matrix."""
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def mat_to_quat(R):
    """3x3 rotation matrix -> unit quaternion (w,x,y,z)."""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return w, x, y, z


def controller_yaw_deg(R):
    """
    Heading of the controller in the horizontal plane.

    SteamVR room frame is OpenGL: +X right, +Y up, -Z forward. The
    controller's forward axis in room space is R @ (0,0,-1). Yaw is its
    angle about +Y (up): 0 pointing forward, +90 pointing right.
    """
    fwd = R @ np.array([0.0, 0.0, -1.0])
    return math.degrees(math.atan2(fwd[0], -fwd[2]))


def controller_roll_deg(R):
    """
    Twist of the controller about its own pointing axis, referenced to
    gravity (room +Y up). 0 = controller upright; +/-90 = rolled onto
    its side; like turning a doorknob. Degenerate only when pointing
    straight up/down (roll/yaw gimbal) — fine for a wrist twist.

    Project the controller's right (R@[1,0,0]) and up (R@[0,1,0]) onto
    world vertical: roll = atan2(right_y, up_y).
    """
    right = R @ np.array([1.0, 0.0, 0.0])
    up = R @ np.array([0.0, 1.0, 0.0])
    return math.degrees(math.atan2(right[1], up[1]))


def controller_pitch_deg(R):
    """
    Elevation of the controller's pointing axis above the horizontal.
    0 = pointing level; +90 = pointing straight up; -90 = straight down.

    fwd = R@[0,0,-1]; pitch = atan2(fwd_up, |fwd_horizontal|). Pairs
    with controller_yaw_deg/controller_roll_deg for a full readout.
    """
    fwd = R @ np.array([0.0, 0.0, -1.0])
    horiz = math.hypot(fwd[0], fwd[2])
    return math.degrees(math.atan2(fwd[1], horiz))


# ---------------------------------------------------------------------------
# Frame calibration — a manual 6-DOF transform set with sliders
# (vr_calibration_gui.py), persisted to vr_calibration.json, and ALWAYS
# applied by the publisher (no A-button capture). The 6 params define a
# frame; every controller pose is re-expressed in it:
#   R_aligned = R_cal^T R
#   p_aligned = R_cal^T (p - t)        t = (x, y, z), R_cal from RPY
# All-zero params = identity (a no-op, raw room frame). The JSON is
# human-readable so it can be hand-edited too. The publisher hot-reloads
# the file on mtime change, so slider tweaks apply live.
# ---------------------------------------------------------------------------
CALIB_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "vr_calibration.json")

CAL_KEYS = ("x", "y", "z", "roll_deg", "pitch_deg", "yaw_deg")
DEFAULT_CAL = {k: 0.0 for k in CAL_KEYS}


def euler_to_mat(roll_deg, pitch_deg, yaw_deg):
    """RPY (degrees) -> 3x3 rotation, intrinsic Rz(yaw)·Ry(pitch)·Rx(roll).
    Axes are the SteamVR room frame: roll about X (right), pitch about Y
    (up), yaw about Z (back)."""
    r, p, y = (math.radians(a) for a in (roll_deg, pitch_deg, yaw_deg))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat_to_euler(R):
    """Inverse of euler_to_mat: 3x3 rotation -> (roll, pitch, yaw) in
    DEGREES for the Rz(yaw)·Ry(pitch)·Rx(roll) convention. Handles the
    pitch=±90 gimbal case (folds the indeterminate roll into yaw)."""
    R = np.asarray(R, dtype=float)
    sp = -float(R[2, 0])
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:                                  # gimbal lock
        roll = 0.0
        yaw = math.atan2(-R[0, 1], R[1, 1])
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def gesture_calibration(fwd_dir, up_dir):
    """Two captured controller pointing-axis directions (room coords)
    -> the 6-param calibration dict (rotation only; x/y/z = 0).

    Builds the frame where the 'forward' gesture maps to -Z and the
    'up' gesture to +Y, via Gram-Schmidt:
        right = unit(f × u);  up = right × f;  R_cal = [right|up|-f]
    Raises ValueError if the two directions are near-parallel (the
    cross product is then ill-defined — re-capture with a clear ~90°
    between forward and up)."""
    f = np.asarray(fwd_dir, dtype=float)
    u0 = np.asarray(up_dir, dtype=float)
    f = f / (np.linalg.norm(f) or 1.0)
    u0 = u0 / (np.linalg.norm(u0) or 1.0)
    right = np.cross(f, u0)
    n = np.linalg.norm(right)
    if n < 0.15:                           # ~ <8.6° apart -> reject
        raise ValueError(
            "forward and up gestures are too close to parallel "
            f"(sin={n:.3f}); point them ~90° apart and recapture")
    right /= n
    up = np.cross(right, f)
    R_cal = np.column_stack([right, up, -f])
    roll, pitch, yaw = mat_to_euler(R_cal)
    return {"x": 0.0, "y": 0.0, "z": 0.0,
            "roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw}


def cal_to_mats(d):
    """6-param dict -> (R_cal 3x3, t 3)."""
    R_cal = euler_to_mat(d["roll_deg"], d["pitch_deg"], d["yaw_deg"])
    t = np.array([d["x"], d["y"], d["z"]], dtype=float)
    return R_cal, t


def load_cal_params(path=CALIB_PATH):
    """Return the 6-param dict (defaults filled in for any missing /
    no / invalid file)."""
    d = dict(DEFAULT_CAL)
    try:
        with open(path) as f:
            raw = json.load(f)
        for k in CAL_KEYS:
            if k in raw:
                d[k] = float(raw[k])
    except (OSError, ValueError, TypeError):
        pass
    return d


def save_cal_params(d, path=CALIB_PATH):
    """Atomically write the 6-param dict as human-readable JSON."""
    out = {k: float(d.get(k, 0.0)) for k in CAL_KEYS}
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(out, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)               # atomic


def load_calibration(path=CALIB_PATH):
    """Convenience for the publisher: (R_cal 3x3, t 3) from the file."""
    return cal_to_mats(load_cal_params(path))


def apply_calibration(p, R, R_cal, t):
    """Re-express a controller pose in the calibrated 6-DOF frame.
    All-zero calibration -> (p, R) unchanged."""
    return R_cal.T @ (p - t), R_cal.T @ R


def read_source(name, p, R):
    """Map a controller pose to a control-source scalar."""
    if name == "yaw":
        return controller_yaw_deg(R)
    if name == "roll":
        return controller_roll_deg(R)
    if name == "y":
        return p[1]                       # room +Y, metres
    raise ValueError(f"unknown source {name!r}")


def source_offset(name, value, ref):
    """
    value - ref, but for angular sources (yaw) take the SHORTEST signed
    difference so crossing the atan2 +/-180 boundary doesn't produce a
    ~360 jump (which the P-loop would turn into a joint slam).
    """
    off = value - ref
    period = SRC_WRAP.get(name)
    if period is not None:
        off = (off + period / 2.0) % period - period / 2.0
    return off


def deadband(value, width):
    """Zero within +/- width, shifted so there is no step at the edge."""
    if abs(value) <= width:
        return 0.0
    return value - math.copysign(width, value)


# ---------------------------------------------------------------------------
# One-Euro filter (Casiez et al.) — low jitter at rest, low lag when moving.
# ---------------------------------------------------------------------------
class _LowPass:
    def __init__(self):
        self.y = None

    def filter(self, x, alpha):
        self.y = x if self.y is None else alpha * x + (1 - alpha) * self.y
        return self.y


class OneEuroFilter:
    def __init__(self, mincutoff=1.0, beta=0.0, dcutoff=1.0):
        self.mincutoff = mincutoff
        self.beta = beta
        self.dcutoff = dcutoff
        self._x = _LowPass()
        self._dx = _LowPass()
        self._t = None
        self._xprev = None

    @staticmethod
    def _alpha(cutoff, dt):
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def update(self, x, t):
        dt = 1e-2 if (self._t is None or t <= self._t) else (t - self._t)
        self._t = t
        dx = 0.0 if self._xprev is None else (x - self._xprev) / dt
        self._xprev = x
        edx = self._dx.filter(dx, self._alpha(self.dcutoff, dt))
        cutoff = self.mincutoff + self.beta * abs(edx)
        return self._x.filter(x, self._alpha(cutoff, dt))


class SourceFilter:
    """
    Per-axis One-Euro wrapper. For angular sources (yaw) it filters a
    continuous *unwrapped* signal so the +/-180 seam never enters the
    filter; the returned value stays continuous and source_offset()
    handles wrapping against the (also-filtered) reference. spec=None
    is a pass-through.
    """

    def __init__(self, name, spec):
        self.period = SRC_WRAP.get(name)
        self._f = OneEuroFilter(**spec) if spec else None
        self._unwrapped = None
        self._prev_raw = None

    def update(self, raw, t):
        if self._f is None:
            return raw
        if self.period is None:
            return self._f.update(raw, t)
        if self._unwrapped is None:
            self._unwrapped = raw
        else:
            half = self.period / 2.0
            d = (raw - self._prev_raw + half) % self.period - half
            self._unwrapped += d
        self._prev_raw = raw
        return self._f.update(self._unwrapped, t)


# ---------------------------------------------------------------------------
# ROS subscriber (used by every consumer)
# ---------------------------------------------------------------------------
class ControllerSubscriber:
    """
    Subscribes to the controller pose + clutch and keeps the latest of
    each, with wall-clock receive times so consumers can detect staleness
    (a teleop must NOT drive on a stale pose). Spins its own rclpy node on
    a background thread.
    """

    def __init__(self, node_name="vr_consumer"):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Bool

        self._rclpy = rclpy
        ros_init_no_signals()
        self._node = rclpy.create_node(node_name)
        self._lock = threading.Lock()
        self._p = None
        self._R = None
        self._pose_t = 0.0
        self._clutch = False
        self._clutch_t = 0.0

        self._node.create_subscription(
            PoseStamped, TOPIC_POSE, self._on_pose, 10
        )
        self._node.create_subscription(
            Bool, TOPIC_CLUTCH, self._on_clutch, 10
        )
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._spin = threading.Thread(
            target=self._exec.spin, daemon=True, name="vr-ros-spin"
        )
        self._spin.start()

        # rclpy's SignalHandlerOptions.NO is not enough — the DDS layer
        # still installs a SIGINT handler that swallows Ctrl+C, so the
        # host's finally (stop arm, free CAN port) never runs. Re-claim
        # SIGINT/SIGTERM *after* everything below us is up so ours wins;
        # both raise KeyboardInterrupt into the main thread. Must be
        # called from the main thread (it is: consumers build this there).
        import signal

        def _raise_kbint(_signo, _frame):
            raise KeyboardInterrupt

        try:
            signal.signal(signal.SIGINT, _raise_kbint)
            signal.signal(signal.SIGTERM, _raise_kbint)
        except (ValueError, OSError) as e:
            print(f"[vr_ros_io] could not claim signal handlers: {e}")

    def _on_pose(self, msg):
        q = msg.pose.orientation
        R = quat_to_mat(q.w, q.x, q.y, q.z)
        p = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])
        with self._lock:
            self._p = p
            self._R = R
            self._pose_t = time.time()

    def _on_clutch(self, msg):
        with self._lock:
            self._clutch = bool(msg.data)
            self._clutch_t = time.time()

    def latest(self):
        """
        Return (p, R, pose_age_s, clutch, clutch_age_s).
        p/R are None until the first pose arrives; ages are large
        (1e9) until the first message of that kind is received.
        """
        now = time.time()
        with self._lock:
            pose_age = (now - self._pose_t) if self._pose_t else 1e9
            clutch_age = (now - self._clutch_t) if self._clutch_t else 1e9
            return self._p, self._R, pose_age, self._clutch, clutch_age

    def shutdown(self):
        # Bounded — the spin thread is a daemon, so never block exit on
        # it (a hung shutdown here is what locked the CAN port before).
        disarm_term_signals()
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass


class HandSubscriber:
    """
    Subscribes to the Valve Index finger curls + trigger + grip for the
    AmazingHand driver. Same pattern as ControllerSubscriber: background
    rclpy spin, wall-clock staleness, claims SIGINT/SIGTERM so Ctrl+C
    cleanly tears the hand process down.

    latest() -> (curls, trigger, grip, age_s)
      curls  : list[5] floats in FINGER_ORDER (0=open .. 1=closed),
               or None until the first message
      trigger: float 0..1   grip: float 0..1
      age_s  : seconds since the most recent finger message (1e9 if none)
    """

    def __init__(self, node_name="vr_hand_consumer"):
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Float32, Float32MultiArray

        self._rclpy = rclpy
        ros_init_no_signals()
        self._node = rclpy.create_node(node_name)
        self._lock = threading.Lock()
        self._curls = None
        self._curls_t = 0.0
        self._trigger = 0.0
        self._grip = 0.0

        self._node.create_subscription(
            Float32MultiArray, TOPIC_FINGERS, self._on_fingers, 10
        )
        self._node.create_subscription(
            Float32, TOPIC_TRIGGER, self._on_trigger, 10
        )
        self._node.create_subscription(
            Float32, TOPIC_GRIP, self._on_grip, 10
        )
        self._exec = SingleThreadedExecutor()
        self._exec.add_node(self._node)
        self._spin = threading.Thread(
            target=self._exec.spin, daemon=True, name="vr-hand-spin"
        )
        self._spin.start()

        import signal

        def _raise_kbint(_signo, _frame):
            raise KeyboardInterrupt

        try:
            signal.signal(signal.SIGINT, _raise_kbint)
            signal.signal(signal.SIGTERM, _raise_kbint)
        except (ValueError, OSError) as e:
            print(f"[vr_ros_io] could not claim signal handlers: {e}")

    def _on_fingers(self, msg):
        with self._lock:
            self._curls = list(msg.data)
            self._curls_t = time.time()

    def _on_trigger(self, msg):
        with self._lock:
            self._trigger = float(msg.data)

    def _on_grip(self, msg):
        with self._lock:
            self._grip = float(msg.data)

    def latest(self):
        now = time.time()
        with self._lock:
            age = (now - self._curls_t) if self._curls_t else 1e9
            curls = list(self._curls) if self._curls is not None else None
            return curls, self._trigger, self._grip, age

    def shutdown(self):
        disarm_term_signals()
        try:
            self._exec.shutdown(timeout_sec=1.0)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass
        try:
            if self._rclpy.ok():
                self._rclpy.shutdown()
        except Exception:
            pass
