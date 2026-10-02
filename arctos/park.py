#!/usr/bin/env python3
"""
Drive the arm to its park pose, so it can be powered down safely.

Why this exists: encoders reset to 0 on power-on, wherever the arm happens to
be. If you ALWAYS park at the same physical pose before cutting power, then at
the next power-up the arm is physically at that pose and the encoder reads 0 --
they agree, and no homing is needed.

That only holds if the park pose IS the zero reference. If you park somewhere
else and then treat power-on zero as the origin, the reference creeps by that
offset every cycle. See the warning this prints for any joint whose park target
is not 0.

  python3 park.py --dry-run
  python3 park.py
  python3 park.py --only 3

Joints move one at a time, distal first, so the arm folds up rather than
swinging its whole mass at once.
"""
import argparse

from arctos_arm import ArctosArm, ArctosError, SoftLimitExceeded

# Park target per joint. 0.0 means "the zero reference", which is what makes
# the power-cycle trick work: park here, cut power, and the next power-on reads
# 0 at this exact pose. Every joint parks at its own zero, which is why the
# zeros are defined at the FOLDED end of each joint.
PARK_POSE = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}

# Distal first: the wrist and forearm fold before the heavy proximal joints.
PARK_ORDER = (6, 5, 4, 3, 2, 1)


def parse_args():
    p = argparse.ArgumentParser(description="park the arm for power-down")
    p.add_argument("--rpm", type=int, default=30)
    p.add_argument("--acc", type=int, default=3)
    p.add_argument("--only", type=int, action="append",
                   help="park only these joints (repeatable)")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--port", default=None)
    return p.parse_args()


def main():
    args = parse_args()
    order = [j for j in PARK_ORDER if not args.only or j in args.only]

    arm = ArctosArm(com_port=args.port)
    arm.connect()

    try:
        print("\n  joint      now        park      move")
        print("  " + "-" * 42)
        plan = {}
        for j in order:
            now = arm.read_encoder(j)
            target = PARK_POSE[j]
            plan[j] = (now, target)
            print(f"   J{j}    {now:+9.2f}  {target:+9.2f}  {target - now:+9.2f}")

        offset = [j for j in order if PARK_POSE[j] != 0.0]
        if offset:
            print("\n  NOTE: these joints park away from their zero: "
                  + ", ".join(f"J{j}@{PARK_POSE[j]:+g}°" for j in offset))
            print("  Power-on will read 0° there, so re-home them rather than")
            print("  assuming park == origin, or the reference creeps each cycle.")

        unhomed = [j for j in order if j not in arm._homed and j in arm.JOINT_LIMITS]
        if unhomed:
            print("\n  WARNING: not zeroed this session: "
                  + ", ".join(f"J{j}" for j in unhomed))
            print("  Their 'now' readings are relative to this power-on pose,")
            print("  not to the real origin — parking may not go where you expect.")

        if args.dry_run:
            print("\n  dry run — nothing moved.")
            return

        if not args.yes:
            print("\n  The arm will move to the park pose, one joint at a time.")
            if input("  Type YES to proceed: ").strip() != "YES":
                print("  Aborted.")
                return

        for j in order:
            now, target = plan[j]
            delta = target - now
            if abs(delta) < 0.05:
                print(f"   J{j}: already parked")
                continue
            print(f"   J{j}: moving {delta:+.2f}° ...", end="", flush=True)
            try:
                arm.move_joint(j, delta, rpm=args.rpm, acc=args.acc, wait=True)
                print(f" now {arm.read_encoder(j):+.3f}°")
            except SoftLimitExceeded as e:
                print(f" refused: {e}")
            except ArctosError as e:
                print(f" failed: {e}")

        print("\n  Parked. Safe to power down.")
    except KeyboardInterrupt:
        print("\n  interrupted — emergency stop")
        arm.emergency_stop_all()
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
