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

import math
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


TOPIC_POSE = "/vr/right_controller/pose"
TOPIC_CLUTCH = "/vr/clutch"
FRAME_ID = "vr_room"           # SteamVR standing room frame (OpenGL axes)

# ---------------------------------------------------------------------------
# Axis map — the single source of truth, imported by teleop AND visualizer.
#   source "yaw" -> controller heading (deg);  "y" -> controller height (m)
#   scale         -> joint-deg per source-unit
#   rpm_floor     -> min |rpm| outside deadband (loaded joints stall below
#                    break-away torque); pos_deadband -> |err| to command 0
# ---------------------------------------------------------------------------
# One-Euro filter spec for a noisy source. Lower mincutoff = smoother
# but laggier at low speed; higher beta = less lag when moving fast.
# Yaw tracking has a few degrees of jitter, so filter it; height is
# steadier and a position source, left unfiltered (filter=None).
YAW_FILTER = {"mincutoff": 1.2, "beta": 0.012, "dcutoff": 1.0}

AXES = [
    {
        "joint": 1, "source": "yaw",
        "scale": 1.0, "invert": True,
        "kp": 6.0, "max_travel": 45.0,
        "rpm_floor": 8, "pos_deadband": 1.0,
        "filter": YAW_FILTER,
    },
    {
        "joint": 2, "source": "y",
        "scale": 60.0, "invert": True,
        "kp": 6.0, "max_travel": 30.0,
        "rpm_floor": 45, "pos_deadband": 2.0,
        "filter": None,
    },
]

YAW_DEADBAND_DEG = 1.0
Y_DEADBAND_M = 0.01
SRC_DEADBAND = {"yaw": YAW_DEADBAND_DEG, "y": Y_DEADBAND_M}
SRC_UNIT = {"yaw": "°", "y": "m"}
# Angular sources wrap; their period (deg). None = linear (no wrap).
SRC_WRAP = {"yaw": 360.0, "y": None}


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


def read_source(name, p, R):
    """Map a controller pose to a control-source scalar."""
    if name == "yaw":
        return controller_yaw_deg(R)
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
