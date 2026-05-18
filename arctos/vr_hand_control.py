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
  * Exit / Ctrl+C -> open the hand, then disable torque (gentle).

Finger curl convention (vr_ros_io.FINGER_ORDER): 0.0 = open/straight,
1.0 = fully curled. AmazingHand has no pinky; the 5th curl is ignored.
"""

import argparse
import time

import numpy as np

from vr_ros_io import FINGER_ORDER, HandSubscriber

SERIAL_PORT = "/dev/ttyACM1"      # the QinHeng 1A86 servo bus (not the CAN)
BAUD = 1_000_000

# !!! CALIBRATION — per-servo neutral offsets, degrees, indexed by
# (servo_id - 1). The shipped value is the AmazingHand_Demo PLACEHOLDER.
# Replace with THIS hand's calibration before using --live.
HAND_MIDDLE_POS = [3, 0, -5, -8, -2, 5, -12, 0]
HAND_CALIBRATED = False           # set True once HAND_MIDDLE_POS is real

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
    args = ap.parse_args()

    if args.live and not HAND_CALIBRATED:
        raise SystemExit(
            "[hand] REFUSING --live: HAND_MIDDLE_POS is the placeholder. "
            "Set this hand's calibration and HAND_CALIBRATED=True first."
        )

    mode = "LIVE (driving servos)" if args.live else "DRY (no motion)"
    print(f"[hand] {mode}. Subscribing to VR finger topics…")
    vr = HandSubscriber(node_name="vr_hand_control")

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
    n = 0
    try:
        while True:
            t0 = time.time()
            curls, trig, grip, age = vr.latest()

            if curls is not None and age <= VR_STALE_S:
                cs = np.array(curls[:5], dtype=float)
                ema = cs if ema is None else (
                    CURL_ALPHA * cs + (1.0 - CURL_ALPHA) * ema
                )
                goals = servo_goals(ema)
                last_goals = goals
            else:
                goals = last_goals      # hold (may be None before 1st msg)

            if goals is not None and args.live:
                for i, g in goals.items():
                    c.write_goal_position(i, g)

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
                    print(f"[hand] {shown}  trig={trig:.2f} "
                          f"grip={grip:.2f}{stale}")

            dt = period - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        print("\n[hand] Ctrl+C")
    finally:
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
        vr.shutdown()


if __name__ == "__main__":
    main()
