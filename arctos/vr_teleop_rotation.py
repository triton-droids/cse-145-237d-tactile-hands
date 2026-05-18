"""
VR rotation teleop for the ARCTOS arm — J1 base yaw only.

POSITION teleop with a deadman clutch. Hold the right Index controller's
B button to engage; the controller's yaw (heading) offset from the moment
you pressed B maps 1:1 (scaled by SCALE) to a J1 *target angle*. The arm
servos to that angle and holds it. Twist back, it follows back. Release B
and it stops where it is.

How it actually drives the motor:
  The MKS firmware primitive we trust on hardware is speed mode (0xF6).
  So position control is a software P-loop: each tick we read the joint
  encoder, compute error = target - current, and command a proportional
  joint speed (rpm = KP * error, capped). At the target, error -> 0 ->
  speed 0 -> it holds. No absolute-move (0xFE) firmware command needed.

Mapping:
  * On B press  -> capture reference yaw AND reference joint angle
                   (a fresh encoder read). No motion yet.
  * While held  -> target = joint_ref + DIR * SCALE * (yaw - yaw_ref),
                   clamped to +/- MAX_TRAVEL_DEG of joint_ref.
                   rpm = clip(KP * (target - encoder), +/- MAX_RPM).
  * On B release-> speed 0. Re-press recaptures both references, so
                   there is never a jump on re-engage.

Safety nets:
  * Deadman: zero motion unless B is physically held.
  * Closed-loop on the encoder: position is measured, not integrated.
  * Encoder feedback REQUIRED — if samples go stale (no fresh reading
    within ENC_STALE_S) the loop commands speed 0 rather than drive
    blind, and warns.
  * Target is clamped to +/- MAX_TRAVEL_DEG from the engage point. A
    further hard runaway guard (encoder drifts past the clamp + margin)
    locks out until you release and re-press B.
  * Any exit path (release, Ctrl+C, exception) stops J1, then
    disconnect() emergency-stops every joint.

Prereqs:
    SteamVR running, right Index controller paired + tracked.
    SLCAN adapter plugged in, J1 powered.
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

# --- Which joint --------------------------------------------------------
JOINT = 1                 # J1 base yaw

# --- Position mapping ---------------------------------------------------
SCALE = 1.0               # joint degrees per controller-yaw degree (1:1)
INVERT = True             # flip which way controller twist maps to J1
YAW_DEADBAND_DEG = 1.0    # ignore controller yaw within this of the ref

# --- Position P-loop ----------------------------------------------------
KP = 6.0                  # motor RPM per degree of joint position error
POS_DEADBAND_DEG = 1.0    # within this of target -> command 0 (no hunting)
MAX_RPM = 80              # motor-RPM cap (J1 gear 24.6 -> ~19 deg/s at joint)
ACC = 5                   # speed-mode accel/decel rate (firmware units)

# --- Safety -------------------------------------------------------------
MAX_TRAVEL_DEG = 45.0     # max target offset from the engage point
RUNAWAY_MARGIN_DEG = 10.0 # encoder past clamp + this -> hard lockout
SEND_RPM_STEP = 3         # only resend speed when it changes by >= this
LOOP_HZ = 50.0
ENC_POLL_S = 0.08         # background encoder poll period (closed-loop rate)
ENC_STALE_S = 0.4         # no fresh sample within this -> command 0


def mat_from_openvr34(m34):
    return np.array(
        [
            [m34.m[0][0], m34.m[0][1], m34.m[0][2]],
            [m34.m[1][0], m34.m[1][1], m34.m[1][2]],
            [m34.m[2][0], m34.m[2][1], m34.m[2][2]],
        ]
    )


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
    Reads one joint's built-in MKS encoder on a background thread.

    read_encoder() blocks on the CAN response, which would stall the
    50 Hz teleop loop. We poll it here every ENC_POLL_S and publish the
    latest (angle, stamp) for the loop's position feedback.
    """

    def __init__(self, arm, joint):
        self._arm = arm
        self._joint = joint
        self._lock = threading.Lock()
        self._angle = None        # last good joint angle, degrees
        self._stamp = 0.0         # time.time() of that reading
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, daemon=True, name="enc-poll"
        )

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=2.0)

    def latest(self):
        with self._lock:
            return self._angle, self._stamp

    def _run(self):
        while not self._stop.is_set():
            try:
                a = self._arm.read_encoder(self._joint)
                with self._lock:
                    self._angle = a
                    self._stamp = time.time()
            except Exception as e:
                print(f"[teleop] encoder poll failed: {e}")
            self._stop.wait(ENC_POLL_S)


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

    direction = -1.0 if INVERT else 1.0
    period = 1.0 / LOOP_HZ

    print("[teleop] connecting to ARCTOS…")
    with ArctosArm() as arm:
        enc = EncoderPoller(arm, JOINT)
        enc.start()
        print(f"[teleop] J{JOINT} ready (POSITION mode). HOLD right-controller "
              f"B to drive, release to stop. Ctrl+C to quit.")

        engaged = False           # B held AND not locked out
        prev_clutch = False       # B state last tick (edge detect)
        locked_out = False        # runaway guard tripped; needs B re-press
        yaw_ref = 0.0             # controller yaw at engage
        joint_ref = 0.0           # joint angle at engage (absolute base)
        last_sent_rpm = None      # so we only resend on meaningful change
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
                yaw = (
                    controller_yaw_deg(
                        mat_from_openvr34(
                            pose_data.pose.mDeviceToAbsoluteTracking
                        )
                    )
                    if pose_ok
                    else None
                )

                rising = clutch and not prev_clutch
                falling = (not clutch) and prev_clutch
                prev_clutch = clutch

                enc_angle, enc_stamp = enc.latest()
                enc_fresh = (
                    enc_angle is not None
                    and (now - enc_stamp) <= ENC_STALE_S
                )

                if rising and pose_ok:
                    # Position mode needs a trustworthy absolute base.
                    if enc_fresh:
                        yaw_ref = yaw
                        joint_ref = enc_angle
                        locked_out = False
                        engaged = True
                        print(f"[teleop] CLUTCH IN  yaw_ref={yaw_ref:+.1f}° "
                              f"joint_ref={joint_ref:+.1f}°")
                    else:
                        print("[teleop] CLUTCH IN ignored — no fresh "
                              "encoder yet, cannot anchor position")

                if falling and engaged:
                    engaged = False
                    arm.stop_joint(JOINT, ACC, wait=False)
                    last_sent_rpm = 0
                    print("[teleop] CLUTCH OUT — stopping")

                # --- Position P-loop ---------------------------------
                rpm = 0
                if engaged and not locked_out:
                    if not pose_ok or yaw is None:
                        rpm = 0                       # no command source
                    elif not enc_fresh:
                        rpm = 0                       # never drive blind
                        print("[teleop] WARNING: encoder stale — holding")
                    else:
                        # Controller yaw offset -> joint target.
                        d_yaw = yaw - yaw_ref
                        if abs(d_yaw) <= YAW_DEADBAND_DEG:
                            d_yaw = 0.0
                        else:
                            d_yaw -= math.copysign(YAW_DEADBAND_DEG, d_yaw)

                        target = joint_ref + direction * SCALE * d_yaw
                        target = float(np.clip(
                            target,
                            joint_ref - MAX_TRAVEL_DEG,
                            joint_ref + MAX_TRAVEL_DEG,
                        ))

                        # Hard runaway guard: measured angle ran well past
                        # the allowed band -> stop and require re-clutch.
                        if abs(enc_angle - joint_ref) > (
                            MAX_TRAVEL_DEG + RUNAWAY_MARGIN_DEG
                        ):
                            rpm = 0
                            locked_out = True
                            engaged = False
                            print(f"[teleop] RUNAWAY GUARD "
                                  f"(enc {enc_angle:+.0f}° vs "
                                  f"ref {joint_ref:+.0f}°) — release & "
                                  f"re-press B")
                        else:
                            # +rpm increases the encoder angle (verified on
                            # hardware), so drive straight off the error.
                            err = target - enc_angle
                            if abs(err) <= POS_DEADBAND_DEG:
                                rpm = 0
                            else:
                                rpm = int(np.clip(
                                    KP * err, -MAX_RPM, MAX_RPM
                                ))

                # Only hit the bus when the command meaningfully changes
                # (always send the settling 0).
                if (
                    last_sent_rpm is None
                    or abs(rpm - last_sent_rpm) >= SEND_RPM_STEP
                    or (rpm == 0 and last_sent_rpm != 0)
                ):
                    arm.set_joint_speed(JOINT, rpm, ACC, wait=False)
                    last_sent_rpm = rpm

                sleep = period - (time.time() - now)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print(f"\n[teleop] Ctrl+C — stopping J{JOINT}")
        finally:
            enc.stop()
            try:
                arm.stop_joint(JOINT, ACC, wait=False)
            except Exception as e:
                print(f"[teleop] stop on exit: {e}")
            # ArctosArm.__exit__ -> disconnect() emergency-stops all joints.

    openvr.shutdown()


if __name__ == "__main__":
    main()
