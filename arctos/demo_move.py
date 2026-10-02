#!/usr/bin/env python3
"""Coordinated whole-arm move: all six joints driving together.

Every pose is expressed as an OFFSET from wherever the arm is when the script
starts, never as an absolute angle.  Encoders reset on power-on, so absolute
targets mean nothing until the arm has been homed -- offsets are safe either
way, and the arm always returns to where it began.

  python3 demo_move.py --dry-run          # print the plan, move nothing
  python3 demo_move.py                    # run it, 8 deg amplitude
  python3 demo_move.py --amplitude 15 --rpm 80
"""
import argparse
import sys
import time

from arctos_arm import ArctosArm, ArctosError

# Each pose is a per-joint multiplier of --amplitude, applied as an offset from
# the starting pose.  J2/J3 carry the arm's weight against gravity, so they get
# smaller multipliers than the lighter distal joints.
SEQUENCE = [
    ("reach out",   {1:  0.0, 2: -0.6, 3:  0.6, 4:  0.0, 5:  0.5, 6:  0.5}),
    ("turn left",   {1:  1.0, 2: -0.6, 3:  0.6, 4:  0.5, 5:  0.5, 6: -0.5}),
    ("turn right",  {1: -1.0, 2: -0.6, 3:  0.6, 4: -0.5, 5: -0.5, 6:  0.5}),
    ("draw back",   {1:  0.0, 2:  0.4, 3: -0.4, 4:  0.0, 5: -0.5, 6: -0.5}),
    ("home",        {1:  0.0, 2:  0.0, 3:  0.0, 4:  0.0, 5:  0.0, 6:  0.0}),
]


def parse_args():
    p = argparse.ArgumentParser(description="coordinated whole-arm move")
    p.add_argument("--amplitude", type=float, default=8.0,
                   help="degrees, scaled per joint by the sequence")
    p.add_argument("--rpm", type=int, default=60)
    p.add_argument("--acc", type=int, default=5)
    p.add_argument("--pause", type=float, default=0.8,
                   help="seconds to settle between poses")
    p.add_argument("--repeat", type=int, default=1)
    p.add_argument("--dry-run", action="store_true",
                   help="print the pose plan without moving")
    p.add_argument("--port", default=None)
    return p.parse_args()


def describe(name, targets, start):
    offsets = " ".join(f"J{j}{targets[j] - start[j]:+6.1f}"
                       for j in sorted(targets))
    return f"{name:<12} {offsets}"


def main():
    args = parse_args()

    if not args.dry_run:
        print("The whole arm will move. Clear the workspace and keep the power "
              "cutoff within reach.")
        if input("Type YES to proceed: ").strip() != "YES":
            sys.exit("Aborted.")

    arm = ArctosArm(com_port=args.port)
    arm.connect()

    try:
        start = dict(arm.current_angles)
        print("\nStarting pose (all offsets are relative to this):")
        print("  " + "  ".join(f"J{j}={start[j]:+.2f}" for j in sorted(start)))
        print()

        # Parked at a limit, half the offsets would be clamped -- the joint
        # still "succeeds", it just travels less, which silently flatters the
        # drift figure. Rather than make the operator jog to mid-range, slide
        # each joint's whole offset pattern into its legal window. Same total
        # travel, just biased so nothing is cut off.
        bias = {j: 0.0 for j in start}
        for j in start:
            window = arm._limit_window(j)
            if window is None:
                continue
            lo, hi, _ = window
            offs = [scale[j] * args.amplitude for _, scale in SEQUENCE]
            span_lo, span_hi = start[j] + min(offs), start[j] + max(offs)
            if span_lo < lo:
                bias[j] = lo - span_lo
            elif span_hi > hi:
                bias[j] = hi - span_hi
            if bias[j] and (span_hi - span_lo) > (hi - lo):
                print(f"  J{j}: range too wide for its limits — reduce "
                      f"--amplitude")
                bias[j] = 0.0
        shifted = {j: b for j, b in bias.items() if abs(b) > 1e-9}
        if shifted:
            print("  Auto-biased into the legal range (no jogging needed): "
                  + ", ".join(f"J{j}{b:+.2f}°" for j, b in sorted(shifted.items())))
            print()

        # Where "home" now lands, and therefore what drift is measured against.
        home = {j: start[j] + bias[j] for j in start}

        for cycle in range(args.repeat):
            if args.repeat > 1:
                print(f"--- cycle {cycle + 1}/{args.repeat} ---")
            for name, scale in SEQUENCE:
                targets = {j: home[j] + scale[j] * args.amplitude
                           for j in start}
                print("  " + describe(name, targets, start))
                if args.dry_run:
                    continue
                arm.set_joint_angles(
                    targets[1], targets[2], targets[3],
                    targets[4], targets[5], targets[6],
                    rpm=args.rpm, acc=args.acc, wait=True,
                )
                time.sleep(args.pause)

        if not args.dry_run:
            print("\nRepeatability — final position vs the home target:")
            worst = 0.0
            for j in sorted(start):
                now = arm.read_encoder(j)
                drift = now - home[j]
                worst = max(worst, abs(drift))
                print(f"  J{j}: {now:+8.3f}  (target {home[j]:+8.3f}, "
                      f"drift {drift:+.3f}°)")
            print(f"\n  worst drift over {args.repeat} cycle(s): {worst:.3f}°")

    except KeyboardInterrupt:
        print("\nInterrupted — emergency stop")
        arm.emergency_stop_all()
        raise
    except ArctosError as e:
        print(f"\nMove failed: {e}")
        arm.emergency_stop_all()
        raise
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
