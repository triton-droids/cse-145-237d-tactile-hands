"""
VR rotation teleop for the ARCTOS arm — J1 base yaw only.

Safest possible first-light teleop: ONE joint, VELOCITY control, deadman
clutch. Hold the right Index controller's B button to engage; the
controller's yaw (heading) offset from the moment you pressed B drives
the base joint's *speed*. Release B and the joint decelerates to a stop.

Mapping:
  * On B press  -> capture reference yaw. No motion yet.
  * While held  -> rpm = GAIN * (yaw_now - yaw_ref), deadbanded + capped.
                   Turn the controller right, the base rotates one way;
                   turn left, the other. Hold it still -> arm holds still.
  * On B release-> speed 0 (graceful decel). Re-press re-zeros the
                   reference, so there is never a jump on re-engage.

Safety nets (first light):
  * Deadman: zero motion unless B is physically held.
  * MAX_RPM cap keeps joint speed gentle.
  * Open-loop travel estimate: if the integrated commanded motion exceeds
    MAX_TRAVEL_DEG from the engage point, motion is locked out until you
    release and re-press B. This is an ESTIMATE (no encoder feedback in
    the loop) — keep clear of cable-wrap range and keep a hand on E-stop.
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

# --- Mapping / feel -----------------------------------------------------
GAIN = 4.0                # motor RPM per degree of controller yaw offset
DEADBAND_DEG = 4.0        # ignore yaw offsets smaller than this (hand jitter)
MAX_RPM = 80              # motor-RPM cap (J1 gear 24.6 -> ~19 deg/s at joint)
ACC = 5                   # speed-mode accel/decel rate (firmware units)
INVERT = True             # flip if the arm rotates opposite the controller

# --- Safety -------------------------------------------------------------
MAX_TRAVEL_DEG = 45.0     # joint travel from engage before lockout
SEND_RPM_STEP = 3         # only resend speed when it changes by >= this
LOOP_HZ = 50.0
ENC_POLL_S = 0.3          # how often the background thread reads the encoder
ENC_STALE_S = 1.5         # warn if no fresh encoder sample for this long


def mat_from_openvr34(m34):
    R = np.array(
        [
            [m34.m[0][0], m34.m[0][1], m34.m[0][2]],
            [m34.m[1][0], m34.m[1][1], m34.m[1][2]],
            [m34.m[2][0], m34.m[2][1], m34.m[2][2]],
        ]
    )
    return R


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

    read_encoder() blocks up to ~1 s on the CAN response, which would
    stall the 50 Hz teleop loop. So we poll it here every ENC_POLL_S and
    publish the latest (angle, stamp) for the loop to sample lock-free-ish.
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

    gear = ArctosArm.GEAR_RATIOS[JOINT]
    period = 1.0 / LOOP_HZ

    print("[teleop] connecting to ARCTOS…")
    with ArctosArm() as arm:
        enc = EncoderPoller(arm, JOINT)
        enc.start()
        print(f"[teleop] J{JOINT} ready. HOLD right-controller B to drive, "
              f"release to stop. Ctrl+C to quit.")

        engaged = False           # B currently held AND not locked out
        prev_clutch = False       # B state last tick (edge detect)
        locked_out = False        # travel-limit tripped; needs B re-press
        yaw_ref = 0.0             # controller yaw captured at engage
        enc_ref = None            # encoder angle at engage (None = no sample)
        last_enc_stamp = 0.0      # stamp of last encoder sample consumed
        est_since_sample = 0.0    # open-loop joint deg since that sample
        stale_warned = False      # so the stale-encoder warning fires once
        last_sent_rpm = None      # so we only resend on meaningful change
        last_t = time.time()

        try:
            while True:
                now = time.time()
                dt = now - last_t
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

                if rising and pose_ok:
                    yaw_ref = yaw
                    enc_ref = enc_angle           # may be None if no sample yet
                    last_enc_stamp = enc_stamp
                    est_since_sample = 0.0
                    locked_out = False
                    stale_warned = False
                    engaged = True
                    ref_str = (
                        f"{enc_ref:+.1f}°" if enc_ref is not None else "n/a"
                    )
                    print(f"[teleop] CLUTCH IN  ref_yaw={yaw_ref:+.1f}° "
                          f"enc_ref={ref_str}")

                if falling and engaged:
                    engaged = False
                    arm.stop_joint(JOINT, ACC, wait=False)
                    last_sent_rpm = 0
                    print("[teleop] CLUTCH OUT — stopping")

                # Decide commanded RPM for this tick.
                rpm = 0
                if engaged and not locked_out and pose_ok:
                    err = yaw - yaw_ref
                    if abs(err) <= DEADBAND_DEG:
                        rpm = 0
                    else:
                        adj = err - math.copysign(DEADBAND_DEG, err)
                        rpm = int(np.clip(GAIN * adj, -MAX_RPM, MAX_RPM))
                        if INVERT:
                            rpm = -rpm

                    # Travel since engage. Encoder is authoritative; the
                    # open-loop term (joint deg/s = rpm*6/gear) only fills
                    # the gap between the ~ENC_POLL_S encoder samples.
                    est_since_sample += (rpm * 6.0 / gear) * dt
                    if enc_ref is not None and enc_angle is not None:
                        if enc_stamp != last_enc_stamp:
                            last_enc_stamp = enc_stamp   # fresh sample…
                            est_since_sample = 0.0       # …encoder now exact
                            stale_warned = False
                        elif (
                            not stale_warned
                            and now - last_enc_stamp > ENC_STALE_S
                        ):
                            print("[teleop] WARNING: encoder stale, "
                                  "limit running open-loop")
                            stale_warned = True
                        travel = (enc_angle - enc_ref) + est_since_sample
                    else:
                        travel = est_since_sample        # no encoder yet

                    if abs(travel) > MAX_TRAVEL_DEG:
                        rpm = 0
                        locked_out = True
                        engaged = False
                        print(f"[teleop] TRAVEL LIMIT ({travel:+.0f}°) "
                              f"— release & re-press B to continue")

                # Only hit the bus when the command meaningfully changes.
                if last_sent_rpm is None or abs(rpm - last_sent_rpm) >= SEND_RPM_STEP or (
                    rpm == 0 and last_sent_rpm != 0
                ):
                    arm.set_joint_speed(JOINT, rpm, ACC, wait=False)
                    last_sent_rpm = rpm

                sleep = period - (time.time() - now)
                if sleep > 0:
                    time.sleep(sleep)

        except KeyboardInterrupt:
            print("\n[teleop] Ctrl+C — stopping J{}".format(JOINT))
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
