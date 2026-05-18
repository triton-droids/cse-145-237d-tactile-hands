"""
VR teleop for the ARCTOS arm — J1 (base yaw) + J2 (shoulder).

POSITION teleop with a deadman clutch. Hold the right Index controller's
B button to engage. While held:

  * J1 (base yaw)  <- controller YAW offset from clutch-in.
  * J2 (shoulder)  <- controller VERTICAL (room +Y) offset from clutch-in,
                      i.e. physically raise/lower your hand.

Release B and both joints stop where they are. Re-press recaptures all
references, so there is never a jump on re-engage.

How it drives the motors:
  The MKS firmware primitive we trust on hardware is speed mode (0xF6).
  Each joint is a software P-loop on top of it: read the joint encoder,
  error = target - current, command rpm = KP * error (capped). At the
  target, error -> 0 -> speed 0 -> it holds. No absolute-move (0xFE).

Per tick, for each axis:
  target = joint_ref + DIR * SCALE * (source_now - source_ref)
  target = clamp(target, joint_ref +/- MAX_TRAVEL_DEG)
  rpm    = clip(KP * (target - encoder), +/- MAX_RPM)

Safety nets:
  * Deadman: zero motion unless B is physically held.
  * Closed-loop on each encoder: position measured, not integrated.
  * Encoder feedback REQUIRED per joint — if a joint's samples go stale
    (no fresh reading within ENC_STALE_S) that joint commands speed 0
    rather than drive blind, and warns.
  * Each target clamped to +/- MAX_TRAVEL_DEG from that joint's engage
    angle. A hard runaway guard (encoder past clamp + margin) locks out
    EVERYTHING until you release and re-press B.
  * Any exit path (release, Ctrl+C, exception) stops the joints, then
    disconnect() emergency-stops every joint.

Prereqs:
    SteamVR running, right Index controller paired + tracked.
    SLCAN adapter plugged in, J1 and J2 powered.
    pip install openvr numpy python-can pyserial

Tunables are the UPPERCASE constants below — start conservative.
"""

import math
import os
import threading
import time

import numpy as np
import openvr

from arctos_arm import ArctosArm

HERE = os.path.dirname(os.path.abspath(__file__))
MANIFEST = os.path.join(HERE, "teleop_actions.json")

# --- Axis map -----------------------------------------------------------
# Each axis: joint, control source, scale, invert, P-gain, travel limit.
#   source "yaw" -> controller heading in degrees
#   source "y"   -> controller height in metres (room +Y)
#   SCALE units  -> joint-degrees per source-unit (deg/deg or deg/metre)
# rpm_floor: minimum commanded |rpm| once outside the deadband. A loaded
#   joint (J2 shoulder) has high stiction/gravity break-away torque: small
#   KP*err commands are below it and the joint stalls short of target, so
#   bump any nonzero command up to at least this. Verified break-away
#   ~40 rpm on J2; J1 (light) needs essentially none.
# pos_deadband: |error| within this -> command 0 (no hunting). Must be
#   wide enough that one floor-speed tick can't overshoot it.
AXES = [
    {
        "joint": 1, "source": "yaw",
        "scale": 1.0, "invert": True,
        "kp": 6.0, "max_travel": 45.0,
        "rpm_floor": 8, "pos_deadband": 1.0,
    },
    {
        "joint": 2, "source": "y",
        "scale": 60.0, "invert": False,
        "kp": 6.0, "max_travel": 30.0,
        "rpm_floor": 45, "pos_deadband": 2.0,
    },
]

# --- Source deadbands (reject hand jitter near the reference) -----------
YAW_DEADBAND_DEG = 1.0
Y_DEADBAND_M = 0.01

# --- Position P-loop (shared) ------------------------------------------
MAX_RPM = 80              # motor-RPM cap per joint
ACC = 5                   # speed-mode accel/decel rate (firmware units)

# --- Safety -------------------------------------------------------------
RUNAWAY_MARGIN_DEG = 10.0 # encoder past clamp + this -> hard lockout (all)
SEND_RPM_STEP = 3         # only resend a joint's speed when it changes >=
LOOP_HZ = 50.0
ENC_POLL_S = 0.08         # background encoder poll period (closed-loop rate)
ENC_STALE_S = 0.4         # no fresh sample within this -> command 0


def pose_from_openvr34(m34):
    """Return (position xyz metres, 3x3 rotation) from an OpenVR 3x4."""
    p = np.array([m34.m[0][3], m34.m[1][3], m34.m[2][3]])
    R = np.array(
        [
            [m34.m[0][0], m34.m[0][1], m34.m[0][2]],
            [m34.m[1][0], m34.m[1][1], m34.m[1][2]],
            [m34.m[2][0], m34.m[2][1], m34.m[2][2]],
        ]
    )
    return p, R


def controller_yaw_deg(R):
    """
    Heading of the controller in the horizontal plane.

    SteamVR room frame is OpenGL: +X right, +Y up, -Z forward. The
    controller's forward axis in room space is R @ (0,0,-1). Yaw is its
    angle about +Y (up): 0 when pointing forward, +90 when pointing right.
    """
    fwd = R @ np.array([0.0, 0.0, -1.0])
    return math.degrees(math.atan2(fwd[0], -fwd[2]))


class EncoderPoller:
    """
    Reads one or more joints' built-in MKS encoders on a background
    thread. read_encoder() blocks on the CAN response, which would stall
    the 50 Hz loop; we poll here and publish the latest (angle, stamp)
    per joint for the loop's position feedback.
    """

    def __init__(self, arm, joints):
        self._arm = arm
        self._joints = list(joints)
        self._lock = threading.Lock()
        self._angle = {j: None for j in self._joints}
        self._stamp = {j: 0.0 for j in self._joints}
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="enc-poll"
        )

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def latest(self, joint):
        with self._lock:
            return self._angle[joint], self._stamp[joint]

    def _run(self):
        while not self._stop.is_set():
            for j in self._joints:
                if self._stop.is_set():
                    break
                try:
                    a = self._arm.read_encoder(j)
                    with self._lock:
                        self._angle[j] = a
                        self._stamp[j] = time.time()
                except Exception as e:
                    print(f"[teleop] encoder poll failed J{j}: {e}")
            self._stop.wait(ENC_POLL_S)


def read_source(name, p, R):
    """Map a controller pose to a control-source scalar."""
    if name == "yaw":
        return controller_yaw_deg(R)
    if name == "y":
        return p[1]                       # room +Y, metres
    raise ValueError(f"unknown source {name!r}")


def deadband(value, width):
    """Zero within +/- width, and shift so there is no step at the edge."""
    if abs(value) <= width:
        return 0.0
    return value - math.copysign(width, value)


def main():
    print("[teleop] initializing SteamVR…")
    openvr.init(openvr.VRApplication_Other)
    vri = openvr.VRInput()
    vri.setActionManifestPath(MANIFEST)

    a_pose = vri.getActionHandle("/actions/arctos/in/hand_pose")
    a_clutch = vri.getActionHandle("/actions/arctos/in/clutch")
    set_main = vri.getActionSetHandle("/actions/arctos")

    active = (openvr.VRActiveActionSet_t * 1)()
    active[0].ulActionSet = set_main
    active[0].ulRestrictedToDevice = openvr.k_ulInvalidInputValueHandle
    active[0].ulSecondaryActionSet = 0
    active[0].nPriority = 0

    src_deadband = {"yaw": YAW_DEADBAND_DEG, "y": Y_DEADBAND_M}
    joints = [ax["joint"] for ax in AXES]
    period = 1.0 / LOOP_HZ

    print("[teleop] connecting to ARCTOS…")
    with ArctosArm() as arm:
        enc = EncoderPoller(arm, joints)
        enc.start()
        print(f"[teleop] joints {joints} ready (POSITION mode). "
              f"HOLD right-controller B to drive, release to stop. "
              f"Ctrl+C to quit.")

        engaged = False           # B held AND not locked out
        prev_clutch = False       # B state last tick (edge detect)
        locked_out = False        # runaway guard tripped; needs B re-press
        src_ref = {}              # source value per axis at engage
        joint_ref = {}            # encoder angle per joint at engage
        last_sent = {ax["joint"]: None for ax in AXES}
        last_t = time.time()

        try:
            while True:
                now = time.time()
                last_t = now

                vri.updateActionState(active)
                clutch = bool(
                    vri.getDigitalActionData(
                        a_clutch, openvr.k_ulInvalidInputValueHandle
                    ).bState
                )
                pose_data = vri.getPoseActionDataRelativeToNow(
                    a_pose,
                    openvr.TrackingUniverseStanding,
                    0.0,
                    openvr.k_ulInvalidInputValueHandle,
                )
                pose_ok = bool(pose_data.pose.bPoseIsValid)
                if pose_ok:
                    p, R = pose_from_openvr34(
                        pose_data.pose.mDeviceToAbsoluteTracking
                    )

                rising = clutch and not prev_clutch
                falling = (not clutch) and prev_clutch
                prev_clutch = clutch

                # Snapshot encoders for every joint this tick.
                enc_now = {}
                enc_fresh = {}
                for ax in AXES:
                    j = ax["joint"]
                    a_ang, a_stp = enc.latest(j)
                    enc_now[j] = a_ang
                    enc_fresh[j] = (
                        a_ang is not None and (now - a_stp) <= ENC_STALE_S
                    )

                if rising and pose_ok:
                    # Position mode needs a trustworthy absolute base for
                    # every joint, so refuse engage unless ALL are fresh.
                    if all(enc_fresh[ax["joint"]] for ax in AXES):
                        for ax in AXES:
                            j = ax["joint"]
                            src_ref[j] = read_source(ax["source"], p, R)
                            joint_ref[j] = enc_now[j]
                        locked_out = False
                        engaged = True
                        refs = " ".join(
                            f"J{ax['joint']}={joint_ref[ax['joint']]:+.1f}°"
                            for ax in AXES
                        )
                        print(f"[teleop] CLUTCH IN  {refs}")
                    else:
                        stale = [
                            ax["joint"] for ax in AXES
                            if not enc_fresh[ax["joint"]]
                        ]
                        print(f"[teleop] CLUTCH IN ignored — no fresh "
                              f"encoder for joints {stale}")

                if falling and engaged:
                    engaged = False
                    for ax in AXES:
                        arm.stop_joint(ax["joint"], ACC, wait=False)
                        last_sent[ax["joint"]] = 0
                    print("[teleop] CLUTCH OUT — stopping")

                # --- Per-axis position P-loop ------------------------
                cmd = {ax["joint"]: 0 for ax in AXES}
                if engaged and not locked_out and pose_ok:
                    for ax in AXES:
                        j = ax["joint"]
                        if not enc_fresh[j]:
                            cmd[j] = 0
                            print(f"[teleop] WARNING: J{j} encoder stale "
                                  f"— holding")
                            continue

                        d_src = deadband(
                            read_source(ax["source"], p, R) - src_ref[j],
                            src_deadband[ax["source"]],
                        )
                        d = -1.0 if ax["invert"] else 1.0
                        target = joint_ref[j] + d * ax["scale"] * d_src
                        target = float(np.clip(
                            target,
                            joint_ref[j] - ax["max_travel"],
                            joint_ref[j] + ax["max_travel"],
                        ))

                        # Hard runaway guard: a joint ran well past its
                        # band. Stop everything, force re-clutch.
                        if abs(enc_now[j] - joint_ref[j]) > (
                            ax["max_travel"] + RUNAWAY_MARGIN_DEG
                        ):
                            locked_out = True
                            engaged = False
                            cmd = {a["joint"]: 0 for a in AXES}
                            print(f"[teleop] RUNAWAY GUARD on J{j} "
                                  f"(enc {enc_now[j]:+.0f}° vs ref "
                                  f"{joint_ref[j]:+.0f}°) — release & "
                                  f"re-press B")
                            break

                        # +rpm increases the encoder angle (verified on
                        # J1 and J2 hardware); drive straight off the
                        # error, with a per-joint break-away floor so a
                        # loaded joint doesn't stall short of target.
                        err = target - enc_now[j]
                        if abs(err) <= ax["pos_deadband"]:
                            cmd[j] = 0
                        else:
                            r = ax["kp"] * err
                            floor = ax["rpm_floor"]
                            if abs(r) < floor:
                                r = math.copysign(floor, r)
                            cmd[j] = int(np.clip(r, -MAX_RPM, MAX_RPM))

                # Push commands; only hit the bus on meaningful change
                # (always send the settling 0).
                for ax in AXES:
                    j = ax["joint"]
                    r = cmd[j]
                    prev = last_sent[j]
                    if (
                        prev is None
                        or abs(r - prev) >= SEND_RPM_STEP
                        or (r == 0 and prev != 0)
                    ):
                        arm.set_joint_speed(j, r, ACC, wait=False)
                        last_sent[j] = r

                sleep = period - (time.time() - now)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print(f"\n[teleop] Ctrl+C — stopping joints {joints}")
        finally:
            enc.stop()
            for ax in AXES:
                try:
                    arm.stop_joint(ax["joint"], ACC, wait=False)
                except Exception as e:
                    print(f"[teleop] stop on exit J{ax['joint']}: {e}")
            # ArctosArm.__exit__ -> disconnect() emergency-stops all joints.

    openvr.shutdown()


if __name__ == "__main__":
    main()
