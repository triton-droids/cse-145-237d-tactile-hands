"""
VR teleop for the ARCTOS arm — J1 (base yaw) + J2 (shoulder).

POSITION teleop with a deadman clutch. The controller pose + B clutch
come from ROS (published by vr_controller_publisher.py) — this script no
longer touches OpenVR. Hold B to engage. While held:

  * J1 (base yaw)  <- controller YAW offset from clutch-in.
  * J2 (shoulder)  <- controller VERTICAL (room +Y) offset from clutch-in.

Release B and both joints stop. Re-press recaptures all references, so
there is never a jump on re-engage.

How it drives the motors:
  The MKS firmware primitive verified on hardware is speed mode (0xF6).
  Each joint is a software P-loop on top of it: read the joint encoder,
  error = target - current, command rpm = KP * error (capped), with a
  per-joint break-away rpm floor so a loaded joint doesn't stall short.

Per tick, per axis (see vr_ros_io.AXES):
  target = joint_ref + DIR * SCALE * (source_now - source_ref)
  target = clamp(target, joint_ref +/- MAX_TRAVEL_DEG)
  rpm    = clip(KP * (target - encoder), +/- MAX_RPM), floored

Safety nets:
  * Deadman: zero motion unless B is held.
  * Closed-loop on each encoder: position measured, not integrated.
  * Encoder feedback REQUIRED per joint — stale -> that joint commands 0.
  * VR feedback REQUIRED — if the ROS pose/clutch goes stale (publisher
    died, controller dropped) everything commands 0.
  * Per-axis travel clamp; hard runaway guard locks out all joints.
  * Any exit path stops the joints, then disconnect() E-stops all.

Run via run_vr_teleop.sh (sources ROS, starts publisher first).
"""

import math
import threading
import time

import numpy as np

from arctos_arm import ArctosArm
from vr_ros_io import (
    AXES,
    SRC_DEADBAND,
    ControllerSubscriber,
    deadband,
    read_source,
    source_offset,
)

# --- Position P-loop (shared cap) --------------------------------------
MAX_RPM = 80              # motor-RPM cap per joint
ACC = 5                   # speed-mode accel/decel rate (firmware units)

# --- Safety -------------------------------------------------------------
RUNAWAY_MARGIN_DEG = 10.0 # encoder past clamp + this -> hard lockout (all)
SEND_RPM_STEP = 3         # only resend a joint's speed when it changes >=
LOOP_HZ = 50.0
ENC_POLL_S = 0.08         # background encoder poll period (closed-loop rate)
ENC_STALE_S = 0.4         # no fresh encoder within this -> command 0
VR_STALE_S = 0.25         # no fresh pose/clutch within this -> command 0


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


def main():
    # SIGINT/SIGTERM -> KeyboardInterrupt is claimed by
    # ControllerSubscriber (after rclpy+DDS init, so it wins). On either
    # signal the finally-block runs: stop joints, disconnect, free the
    # CAN port. This is what makes Ctrl+C / launcher teardown reliable.
    joints = [ax["joint"] for ax in AXES]
    period = 1.0 / LOOP_HZ

    print("[teleop] subscribing to VR ROS topics…")
    vr = ControllerSubscriber(node_name="vr_teleop")

    print("[teleop] connecting to ARCTOS…")
    with ArctosArm() as arm:
        enc = EncoderPoller(arm, joints)
        enc.start()
        print(f"[teleop] joints {joints} ready (POSITION mode via ROS). "
              f"HOLD B to drive, release to stop. Ctrl+C to quit.")

        engaged = False           # B held AND not locked out
        prev_clutch = False       # B state last tick (edge detect)
        locked_out = False        # runaway guard tripped; needs B re-press
        src_ref = {}              # source value per axis at engage
        joint_ref = {}            # encoder angle per joint at engage
        last_sent = {ax["joint"]: None for ax in AXES}

        try:
            while True:
                now = time.time()

                p, R, pose_age, clutch, clutch_age = vr.latest()
                vr_ok = (
                    p is not None
                    and pose_age <= VR_STALE_S
                    and clutch_age <= VR_STALE_S
                )
                # A stale publisher must look like "released", not frozen
                # in whatever the last clutch value was.
                if not vr_ok:
                    clutch = False

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

                if rising and vr_ok:
                    if all(enc_fresh[ax["joint"]] for ax in AXES):
                        for ax in AXES:
                            j = ax["joint"]
                            src_ref[j] = read_source(ax["source"], p, R)
                            joint_ref[j] = enc_now[j]
                        locked_out = False
                        engaged = True
                        refs = " ".join(
                            f"J{ax['joint']}="
                            f"{joint_ref[ax['joint']]:+.1f}°"
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
                if engaged and not locked_out and vr_ok:
                    for ax in AXES:
                        j = ax["joint"]
                        if not enc_fresh[j]:
                            cmd[j] = 0
                            print(f"[teleop] WARNING: J{j} encoder stale "
                                  f"— holding")
                            continue

                        d_src = deadband(
                            source_offset(
                                ax["source"],
                                read_source(ax["source"], p, R),
                                src_ref[j],
                            ),
                            SRC_DEADBAND[ax["source"]],
                        )
                        d = -1.0 if ax["invert"] else 1.0
                        target = joint_ref[j] + d * ax["scale"] * d_src
                        target = float(np.clip(
                            target,
                            joint_ref[j] - ax["max_travel"],
                            joint_ref[j] + ax["max_travel"],
                        ))

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
                        # J1 and J2); drive straight off the error with a
                        # per-joint break-away floor.
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
            vr.shutdown()
            # ArctosArm.__exit__ -> disconnect() E-stops all joints.


if __name__ == "__main__":
    main()
