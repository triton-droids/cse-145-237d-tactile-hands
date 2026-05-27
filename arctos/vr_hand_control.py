"""
AmazingHand control from Valve Index finger sensing (via ROS).

Subscribes to the capacitive finger curls published by
vr_controller_publisher.py and drives the AmazingHand's 8 SCS servos
(rustypot Scs0009PyController) so the physical hand mirrors your hand.

Run under SYSTEM python with ROS sourced (rustypot is there, not conda):
    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 vr_hand_control.py            # DRY: no motion
    /usr/bin/python3 vr_hand_control.py --live     # drives servos

SAFETY
  * DRY by default — subscribes and prints the servo targets it WOULD
    send; never enables torque, never writes. Use this to verify the
    pipeline and the numbers before anything moves.
  * --live refuses to start unless HAND_MIDDLE_POS has been set to this
    hand's real calibration (the shipped value is a placeholder; wrong
    offsets drive servos into their mechanical stops).
  * Stale finger data (publisher down / controller dropped) -> HOLD the
    last goal. Never goes limp mid-grip, never snaps.
  * FORCE-STOP -> reads the ESP32 FSRs (FSR_PORT) and, against the object-
    aware threshold from vr_object_force.py (ROS; DEFAULT_FORCE_LIMIT when
    absent/stale), refuses to close a finger any further once its FSR
    exceeds the limit. Opening is always allowed. Disable with --no-fsr.
  * Exit / Ctrl+C -> open the hand, then disable torque (gentle).

Finger curl convention (vr_ros_io.FINGER_ORDER): 0.0 = open/straight,
1.0 = fully curled. AmazingHand has no pinky; the 5th curl is ignored.
"""

import argparse
import time

import numpy as np

from vr_ros_io import (
    DEFAULT_FORCE_LIMIT,
    FINGER_ORDER,
    FSR_FINGER_ORDER,
    HandSubscriber,
    disarm_term_signals,
)

SERIAL_PORT = "/dev/ttyACM1"      # the QinHeng 1A86 servo bus (not the CAN)
BAUD = 1_000_000

# CALIBRATION — per-servo neutral offsets, degrees, indexed by
# (servo_id - 1). Taken from this project's hand_control/angle_control.py
# (same MiddlePos[ID-1]+angle formula and servo-id pairs as here), i.e.
# this physical hand's calibration.
HAND_MIDDLE_POS = [1.4, 19.6, 12.6, -7.9, 13.9, 10.5, 17, -28.1]
HAND_CALIBRATED = True            # from hand_control/angle_control.py

# Finger -> (servo_id_1, servo_id_2). Curl drives them differentially,
# exactly like AmazingHand_Demo's Move_* (open -35/+35, close +90/-90).
FINGER_SERVOS = {
    "thumb":  (7, 8),
    "index":  (1, 2),
    "middle": (3, 4),
    "ring":   (5, 6),
}
OPEN_A1, CLOSE_A1 = -35.0, 90.0   # finger angle_1: open -> close
OPEN_A2, CLOSE_A2 = 35.0, -90.0   # finger angle_2 (opposite — differential)

SERVO_SPEED = 5                   # rustypot goal speed (demo MaxSpeed-2)
CURL_ALPHA = 0.35                 # EMA smoothing on incoming curls
LOOP_HZ = 30.0
VR_STALE_S = 0.25                 # older finger data -> hold last goal

# --- FSR force feedback (the AmazingHand's stop-when-grasped behavior) ------
# The ESP32 (firmware/firmware.ino) streams 4 FSR channels @115200 and accepts
# "white,red\n" LED duty back. Channel order = FSR_FINGER_ORDER. The per-object
# threshold arrives over ROS from vr_object_force.py; until one does (object
# node down / nothing detected) the hand uses DEFAULT_FORCE_LIMIT.
FSR_PORT = "/dev/ttyUSB0"         # the ESP32 FSR board (NOT the SCS servo bus)
FSR_BAUD = 115200
FORCE_STALE_S = 1.0               # older force-limit msg -> fall back to default


class FsrReader:
    """Owns the FSR serial: non-blocking read of the freshest 4-channel frame,
    plus optional white/red overload-LED feedback (same protocol as the demo's
    hand_control/angle_control.py)."""

    def __init__(self, port=FSR_PORT, baud=FSR_BAUD):
        import serial
        self.ser = serial.Serial(port, baud, timeout=0.05)
        self.force = [0, 0, 0, 0]

    def read(self):
        """Drain whatever is waiting; keep the last complete valid frame."""
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) == 4:
                    try:
                        self.force = [int(x) for x in parts]
                    except ValueError:
                        pass
        except Exception:
            pass
        return self.force

    def set_leds(self, white, red):
        try:
            self.ser.write(f"{int(white)},{int(red)}\n".encode())
        except Exception:
            pass

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def finger_targets_deg(curl):
    """A curl 0..1 -> (angle_1, angle_2) degrees, linear open->close."""
    c = float(np.clip(curl, 0.0, 1.0))
    a1 = OPEN_A1 + c * (CLOSE_A1 - OPEN_A1)
    a2 = OPEN_A2 + c * (CLOSE_A2 - OPEN_A2)
    return a1, a2


def servo_goals(curls):
    """
    Map the 5 published curls to {servo_id: goal_rad} using the
    per-servo calibration. Thumb,index,middle,ring; pinky ignored.
    """
    goals = {}
    for fname, (id1, id2) in FINGER_SERVOS.items():
        c = curls[FINGER_ORDER.index(fname)]
        a1, a2 = finger_targets_deg(c)
        goals[id1] = np.deg2rad(HAND_MIDDLE_POS[id1 - 1] + a1)
        goals[id2] = np.deg2rad(HAND_MIDDLE_POS[id2 - 1] + a2)
    return goals


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live", action="store_true",
                    help="actually drive servos (default: dry run)")
    ap.add_argument("--port", default=SERIAL_PORT)
    ap.add_argument("--fsr-port", default=FSR_PORT,
                    help="ESP32 FSR serial port for force-stop "
                         "(default %(default)s)")
    ap.add_argument("--no-fsr", action="store_true",
                    help="disable FSR force-stop (pure VR teleop)")
    ap.add_argument("--no-led", action="store_true",
                    help="don't drive the white/red overload LEDs")
    args = ap.parse_args()

    if args.live and not HAND_CALIBRATED:
        raise SystemExit(
            "[hand] REFUSING --live: HAND_MIDDLE_POS is the placeholder. "
            "Set this hand's calibration and HAND_CALIBRATED=True first."
        )

    mode = "LIVE (driving servos)" if args.live else "DRY (no motion)"
    print(f"[hand] {mode}. Subscribing to VR finger topics…")
    vr = HandSubscriber(node_name="vr_hand_control")

    # FSR force-stop. Best-effort: if the board isn't there, warn loudly and
    # keep running as pure teleop rather than refusing to move the hand.
    fsr = None
    if args.no_fsr:
        print("[hand] FSR force-stop DISABLED (--no-fsr).")
    else:
        try:
            fsr = FsrReader(args.fsr_port)
            print(f"[hand] FSR force-stop ON ({args.fsr_port}).")
        except Exception as e:
            print(f"[hand] WARNING: FSR open failed on {args.fsr_port}: {e}")
            print("[hand] continuing WITHOUT force-stop (pure teleop).")

    c = None
    ids = sorted({i for pair in FINGER_SERVOS.values() for i in pair})
    if args.live:
        from rustypot import Scs0009PyController
        c = Scs0009PyController(
            serial_port=args.port, baudrate=BAUD, timeout=0.5
        )
        for i in ids:
            c.write_torque_enable(i, 1)
            c.write_goal_speed(i, SERVO_SPEED)
        # Ease to OPEN before tracking so we never snap from rest.
        for i, g in servo_goals([0.0] * 5).items():
            c.write_goal_position(i, g)
        time.sleep(0.6)

    period = 1.0 / LOOP_HZ
    ema = None
    last_goals = None
    last_curls = None       # last COMMANDED curls (post-clamp) -> force-stop ref
    n = 0
    try:
        while True:
            t0 = time.time()
            curls, trig, grip, age = vr.latest()

            # Object-aware threshold; fall back to default if the object node
            # is down or hasn't published a fresh value.
            flimit, label, flage = vr.force_limit()
            limit = (flimit if (flimit is not None and flage <= FORCE_STALE_S)
                     else DEFAULT_FORCE_LIMIT)
            force = fsr.read() if fsr is not None else None

            clamped = []
            if curls is not None and age <= VR_STALE_S:
                cs = np.array(curls[:5], dtype=float)
                ema = cs if ema is None else (
                    CURL_ALPHA * cs + (1.0 - CURL_ALPHA) * ema
                )
                target = ema.copy()

                # FORCE-STOP: a finger over its limit may not close any further
                # than last commanded (it may still open). Curl-domain version
                # of angle_control.apply_force_limit.
                if force is not None and last_curls is not None:
                    for fname in FINGER_SERVOS:
                        ch = FSR_FINGER_ORDER.index(fname)
                        fi = FINGER_ORDER.index(fname)
                        if force[ch] > limit and target[fi] > last_curls[fi]:
                            target[fi] = last_curls[fi]
                            clamped.append(fname)

                goals = servo_goals(target)
                last_goals = goals
                last_curls = target
            else:
                goals = last_goals      # hold (may be None before 1st msg)

            if goals is not None and args.live:
                for i, g in goals.items():
                    c.write_goal_position(i, g)

            # Overload LEDs: white = ok, red = a finger is over the limit.
            if fsr is not None and not args.no_led:
                overload = force is not None and max(force) > limit
                fsr.set_leds(0 if overload else 100, 100 if overload else 0)

            n += 1
            if n % int(LOOP_HZ) == 0:
                if goals is None:
                    print(f"[hand] waiting for finger data (age={age:.2f})")
                else:
                    shown = " ".join(
                        f"{f}={ema[FINGER_ORDER.index(f)]:.2f}"
                        for f in FINGER_SERVOS
                    )
                    stale = "" if age <= VR_STALE_S else "  [STALE-HOLD]"
                    obj = f"  obj={label or '-'} lim={limit:g}"
                    fstr = ("  F=" + ",".join(
                        str(force[FSR_FINGER_ORDER.index(f)])
                        for f in FINGER_SERVOS)) if force is not None else ""
                    stop = ("  STOP[" + ",".join(clamped) + "]") if clamped else ""
                    print(f"[hand] {shown}  trig={trig:.2f} "
                          f"grip={grip:.2f}{obj}{fstr}{stop}{stale}")

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[hand] Ctrl+C")
    finally:
        # Atomic teardown: a 2nd signal must not skip open + torque-off.
        disarm_term_signals()
        if args.live and c is not None:
            try:
                for i, g in servo_goals([0.0] * 5).items():
                    c.write_goal_position(i, g)   # open
                time.sleep(0.6)
                for i in ids:
                    c.write_torque_enable(i, 2)   # 2 = torque off (limp)
                print("[hand] opened and torque off")
            except Exception as e:
                print(f"[hand] cleanup error: {e}")
        if fsr is not None:
            fsr.set_leds(0, 0)
            fsr.close()
        vr.shutdown()


if __name__ == "__main__":
    main()
