#!/usr/bin/env python3
"""
Establish the arm's origin, so every joint angle means something fixed.

Sensored joints (J2, J3) home themselves: the joint creeps toward a limit
sensor while we poll the input pins over CAN (0x34), and stops the moment a
sensor bit changes. Non-contact, no wear on printed parts.

This deliberately does NOT use the firmware's homing (0x91) or its EndLimit
function. EndLimit only takes effect after a firmware homing cycle, and when it
trips it UNLOCKS THE SHAFT -- which on the gravity-loaded J2/J3 means the arm
drops. Reading the sensors ourselves avoids both problems.

Sensorless joints (J1, J4, J5, J6) are continuous, so their zero is wherever
you park them: position the joint, confirm, and it is zeroed in place.

  python3 home.py 3              # home J3 against its sensor
  python3 home.py 3 --dir +1     # seek the other direction
  python3 home.py 1 --manual     # zero J1 wherever it currently sits
  python3 home.py --dry-run 3    # show sensor state, move nothing
"""
import argparse
from concurrent.futures import TimeoutError as FutureTimeout

from arctos_arm import (
    ArctosArm, ArctosError, LimitHit, SoftLimitExceeded, CommandTimeout,
    CMD_MOVE_RELATIVE,
)

# Joints wired with magnetic limit sensors on this arm.
SENSORED = (2, 3)

IN_1, IN_2 = 0x01, 0x02
INPUT_MASK = IN_1 | IN_2


def parse_args():
    p = argparse.ArgumentParser(description="establish the arm origin")
    p.add_argument("joint", type=int)
    p.add_argument("--dir", type=int, choices=[-1, 1], default=-1,
                   help="direction to seek the sensor (default -1)")
    p.add_argument("--coarse-step", type=float, default=1.0)
    p.add_argument("--fine-step", type=float, default=0.1)
    p.add_argument("--rpm", type=int, default=15)
    p.add_argument("--fine-rpm", type=int, default=5)
    p.add_argument("--backoff", type=float, default=2.0)
    p.add_argument("--acc", type=int, default=2)
    p.add_argument("--max-seek", type=float, default=400.0,
                   help="give up after this much travel with no sensor change")
    p.add_argument("--manual", action="store_true",
                   help="zero at the current position, no seeking")
    p.add_argument("--no-zero", action="store_true",
                   help="find the sensor but do not set zero")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--yes", action="store_true")
    p.add_argument("--port", default=None)
    return p.parse_args()


def inputs(arm, joint):
    return arm.read_io(joint) & INPUT_MASK


def fmt_inputs(v):
    return f"IN_1={v & 1} IN_2={(v >> 1) & 1}"


def step_timeout(arm, joint, deg, rpm):
    revs = abs(deg) * arm.GEAR_RATIOS[joint] / 360.0
    return max(2.0, revs / max(rpm, 1) * 60.0 + 1.5)


def seek(arm, joint, direction, step, rpm, acc, baseline, args, label):
    """
    Creep until an input pin differs from `baseline`.

    A trip is a FALLING edge only: an input that was high in `baseline` and is
    now low (homeTrig is configured Low-active). Watching for any change would
    make *leaving* a sensor look like arriving at one -- which is exactly the
    case when homing at one end and then seeking the other.

    Returns (position, tripped_mask) or (position, 0) if the seek ran out of
    travel. Checks the sensor before each step as well as after, so a joint
    already sitting on its sensor is detected without moving at all.
    """
    start = arm.read_encoder(joint)
    for _ in range(int(args.max_seek / step) + 2):
        now = inputs(arm, joint)
        tripped = baseline & ~now & INPUT_MASK   # high -> low only
        if tripped:
            return arm.read_encoder(joint), tripped

        try:
            fut = arm.move_joint(joint, direction * step, rpm=rpm, acc=acc,
                                 wait=False, on_limit="raise")
            fut.result(timeout=step_timeout(arm, joint, step, rpm))
        except LimitHit:
            return arm.read_encoder(joint), INPUT_MASK    # firmware saw it first
        except (FutureTimeout, SoftLimitExceeded, ArctosError) as e:
            arm._fail_pending(joint, CMD_MOVE_RELATIVE,
                              CommandTimeout("home seek step"))
            try:
                arm.emergency_stop(joint)
            except ArctosError:
                pass
            raise ArctosError(f"seek stopped before reaching a sensor: {e}")

        pos = arm.read_encoder(joint)
        print(f"    {label}: {pos:+8.3f}°  {fmt_inputs(inputs(arm, joint))}",
              end="\r", flush=True)
        if abs(pos - start) > args.max_seek:
            return pos, 0
    return arm.read_encoder(joint), 0


def main():
    args = parse_args()
    arm = ArctosArm(com_port=args.port)
    arm.connect()
    arm.TRAVEL_LIMITS = dict(ArctosArm.TRAVEL_LIMITS)
    arm.TRAVEL_LIMITS[args.joint] = args.max_seek + 10.0

    try:
        pos = arm.read_encoder(args.joint)
        try:
            base = inputs(arm, args.joint)
            print(f"\nJ{args.joint} at {pos:+.3f}°   sensors: {fmt_inputs(base)}")
        except ArctosError as e:
            print(f"\nJ{args.joint} at {pos:+.3f}°   sensors: unreadable ({e})")
            base = None

        if args.joint not in SENSORED and not args.manual:
            print(f"\nJ{args.joint} has no limit sensors on this arm.")
            print("Position it where you want zero, then re-run with --manual.")
            return

        if args.dry_run:
            print("\ndry run — nothing moved.")
            return

        if args.manual:
            if not args.yes:
                print(f"\nThis marks J{args.joint}'s CURRENT position as zero.")
                if input("Type YES to proceed: ").strip() != "YES":
                    print("Aborted.")
                    return
            arm.zero_here(args.joint)
            print(f"  J{args.joint} zeroed at its current position.")
            return

        if base is None:
            print("\nCannot read the sensors — aborting rather than seeking blind.")
            return
        if base != INPUT_MASK:
            print(f"\n  note: an input already reads low ({fmt_inputs(base)}) — "
                  f"the joint is sitting on a sensor. Leaving it is ignored; "
                  f"only a NEW sensor going low counts as a trip.")

        if not args.yes:
            d = "+" if args.dir > 0 else "-"
            print(f"\nJ{args.joint} will creep in the {d} direction at "
                  f"{args.rpm} rpm until a sensor changes.")
            print("Clear the workspace and keep the power cutoff within reach.")
            if input("Type YES to proceed: ").strip() != "YES":
                print("Aborted.")
                return

        print(f"\n  coarse seek, {args.coarse_step}° @ {args.rpm} rpm...")
        pos, tripped = seek(arm, args.joint, args.dir, args.coarse_step,
                            args.rpm, args.acc, base, args, "seek")
        if not tripped:
            print(f"\n  no sensor change within {args.max_seek:g}° — "
                  f"check wiring with check_sensors.py")
            return
        which = "IN_1(En)" if tripped & IN_1 else "IN_2(Dir)"
        print(f"\n  sensor {which} tripped at {pos:+.3f}°          ")

        print(f"  backing off {args.backoff}° for a slow re-approach...")
        arm.move_joint(args.joint, -args.dir * args.backoff,
                       rpm=args.fine_rpm, acc=args.acc, wait=True,
                       on_limit="raise")
        base2 = inputs(arm, args.joint)
        pos, tripped = seek(arm, args.joint, args.dir, args.fine_step,
                            args.fine_rpm, args.acc, base2, args, "fine")
        if not tripped:
            print("\n  sensor did not re-trip on the slow approach — "
                  "using the coarse reading.")
        else:
            print(f"\n  confirmed at {pos:+.3f}°          ")

        if args.no_zero:
            print(f"\n  --no-zero: leaving the encoder as-is. "
                  f"Sensor position is {pos:+.3f}°.")
            return

        arm.zero_here(args.joint)
        print(f"\n  J{args.joint} ORIGIN SET at the sensor. "
              f"It now reads {arm.read_encoder(args.joint):+.3f}°.")
        print("  Re-run this before each session and every angle will line up.")
    except KeyboardInterrupt:
        print("\ninterrupted — emergency stop")
        arm.emergency_stop_all()
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
