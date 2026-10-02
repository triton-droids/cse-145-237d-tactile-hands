#!/usr/bin/env python3
"""Diagnose why a joint answers queries but will not move.

Queries motor status (0xF1), then fires one small relative move (0xFD) and
prints every frame that comes back for a few seconds.  The reply pattern tells
you where it is stuck:

  FD 01 then FD 02   -> motor moved and finished; nothing is wrong
  FD 01 then silence -> command accepted but the shaft never turned
                        (motor disabled, no torque, or stalled)
  FD 00              -> command explicitly rejected
  no FD reply at all -> board is ignoring serial motion commands, which
                        usually means its work mode is a pulse/step-dir mode
                        (CR_*) rather than a serial mode (SR_*)

  python3 motion_debug.py 6            # joint 6, 3 degrees
  python3 motion_debug.py 6 --deg 5 --rpm 50
"""
import argparse
import time

import arctos_arm
from arctos_arm import ArctosArm, CMD_QUERY_STATUS


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("joint", help="joint number, or comma list to move together "
                                  "(e.g. '5,6' for the differential wrist)")
    ap.add_argument("--deg", type=float, default=3.0)
    ap.add_argument("--rpm", type=int, default=50)
    ap.add_argument("--acc", type=int, default=5)
    ap.add_argument("--watch", type=float, default=None,
                    help="seconds to watch after the move frame (default: sized "
                         "from the move so it is not cut short)")
    ap.add_argument("--alt", action="store_true",
                    help="flip the sign for every other joint in the list; on a "
                         "differential wrist this drives the other axis")
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    # Force frame logging on regardless of how the env was set.
    arctos_arm._CAN_DEBUG = True

    joints = [int(x) for x in args.joint.split(",") if x.strip()]

    arm = ArctosArm(com_port=args.port)
    arm.JOINTS = tuple(joints)
    arm.current_angles = {j: 0.0 for j in joints}
    arm.connect()

    try:
        before = dict(arm.current_angles)
        print(f"\n--- encoder before: {arm.current_angles} ---\n")

        print("--- querying motor status (0xF1) ---")
        for j in joints:
            fut = arm._register_pending(j, CMD_QUERY_STATUS)
            arm._send_frame(j, bytes([CMD_QUERY_STATUS]))
            try:
                print(f"  joint {j} status = {fut.result(timeout=2.0)}")
            except Exception as e:
                print(f"  joint {j} no status reply: {e}")

        print(f"\n--- sending move @ {args.rpm} rpm (not waiting) ---")
        commanded = {}
        for idx, j in enumerate(joints):
            deg = -args.deg if (args.alt and idx % 2) else args.deg
            commanded[j] = deg
            print(f"    joint {j}: {deg:+}deg")
            arm.move_joint(j, deg, rpm=args.rpm, acc=args.acc, wait=False)

        # disconnect() fires an emergency stop, so a watch window shorter than
        # the move truncates it mid-flight and looks like a shortfall.
        if args.watch is None:
            revs = max(abs(commanded[j]) * arm.GEAR_RATIOS[j] / 360.0
                       for j in joints)
            args.watch = round(revs / max(args.rpm, 1) * 60.0 + 2.0, 1)
            print(f"--- watch window auto-sized to {args.watch}s for this move ---")

        print(f"--- watching {args.watch}s, polling status + encoder ---")
        print("    (status: 0=fail 1=stopped 2=accel 3=decel 4=full-speed 5=homing)")
        deadline = time.time() + args.watch
        while time.time() < deadline:
            time.sleep(0.5)
            t = args.watch - (deadline - time.time())
            cells = []
            for j in joints:
                sfut = arm._register_pending(j, CMD_QUERY_STATUS)
                arm._send_frame(j, bytes([CMD_QUERY_STATUS]))
                try:
                    status = sfut.result(timeout=1.0)
                except Exception:
                    status = "?"
                try:
                    deg_s = f"{arm.read_encoder(j):+.4f}"
                except Exception:
                    deg_s = "?"
                cells.append(f"J{j} status={status} enc={deg_s}")
            print(f"    t={t:4.1f}s  " + "  |  ".join(cells))

        # move_joint is RELATIVE, so what matters is the delta from where the
        # joint started, not its absolute encoder position.
        print("\n--- results (relative move, so delta is what counts) ---")
        for j in joints:
            try:
                after = arm.read_encoder(j)
                moved = after - before[j]
                err = moved - commanded[j]
                verdict = "OK" if abs(err) < 0.5 else "SHORT"
                print(f"  joint {j}: moved {moved:+.4f} deg of "
                      f"{commanded[j]:+} commanded  (err {err:+.4f})  {verdict}")
                print(f"            encoder {before[j]:+.4f} -> {after:+.4f}")
            except Exception as e:
                print(f"  joint {j}: encoder read failed: {e}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
