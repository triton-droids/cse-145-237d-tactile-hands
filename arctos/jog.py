#!/usr/bin/env python3
"""
Interactive jog tool for the ARCTOS arm.

Drives one joint at a time in small keypress-sized steps so you can walk a
joint up to its mechanical stop while watching it, then record that position
as a limit. Writes joint_limits.json for the safety framework to consume.

Safety model: every jog is a single bounded step that completes before the
next key is read, and the arm's TRAVEL_LIMITS guard is extended by exactly
one step at a time -- so a stuck key or a bug can never move further than one
step past where you have already been. --max-travel is the hard ceiling.

  python3 jog.py                      # 1.0 deg steps, 30 rpm
  python3 jog.py --step 0.5 --rpm 20

Keys:
  1-6        select joint
  7 / 8      select coupled wrist axis (both motors together)
  , / .      jog negative / positive   (also left / right arrow)
  - / =      step size down / up
  [ / ]      record current position as this joint's MIN / MAX
  z          set THIS joint's zero here (confirms with y)
  n          mark axis CONTINUOUS (turns freely, no stop)
  c          clear this joint's recorded limits (this session only)
  s          save joint_limits.json (merges; never drops other fields)
  S          save the current pose by name (poses.json)
  p          print full state
  x          emergency stop all
  q          quit (stops motors, saves nothing)
"""
import argparse
import json
import os
import sys
import termios
import time
import tty

from concurrent.futures import TimeoutError as FutureTimeout

from arctos_arm import (
    ArctosArm, ArctosError, SoftLimitExceeded, LimitHit, CommandTimeout,
    CMD_MOVE_RELATIVE, LIMITS_PATH,
)
from poses import save_pose, load_poses, describe


class SensorLimit(ArctosError):
    """
    A magnetic limit sensor tripped and the firmware stopped the move (FD 03).

    Definitive: the joint is at a real, repeatable end of travel. Record it
    without hesitation.
    """


class StopReached(ArctosError):
    """
    A step was commanded but never reported completion.

    With locked-rotor protection enabled the board cuts the drive when the
    joint cannot follow, and simply stops answering -- so a step that times
    out is the mechanical stop announcing itself. For a limit-finding tool
    that is a result, not an error.
    """

STEP_SIZES = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]
DEFAULT_MARGIN = 2.0

JOINT_NAMES = {
    1: "base yaw", 2: "shoulder", 3: "elbow",
    4: "forearm roll", 5: "wrist B", 6: "wrist C",
    7: "wrist SUM", 8: "wrist DIFF",
}

# J5/J6 drive a differential: neither motor alone corresponds to a wrist axis.
# Axes 7 and 8 are the coupled combinations that DO -- 7 drives both motors the
# same direction, 8 drives them opposite. Which one is pitch and which is roll
# depends on the build, so jog each and label them from what you observe.
# Axis value 7 = (t5 + t6)/2, axis 8 = (t5 - t6)/2, matching those drives.
VIRTUAL_AXES = {
    7: {5: +1.0, 6: +1.0},
    8: {5: +1.0, 6: -1.0},
}


def parse_args():
    p = argparse.ArgumentParser(description="interactive joint jog / limit finder")
    p.add_argument("--step", type=float, default=1.0, help="initial step (deg)")
    p.add_argument("--rpm", type=int, default=30)
    p.add_argument("--acc", type=int, default=3)
    p.add_argument("--max-travel", type=float, default=180.0,
                   help="hard ceiling on travel from the start pose (deg)")
    p.add_argument("--margin", type=float, default=None,
                   help="degrees backed off each newly recorded stop when "
                        "saving (default: the file's margin_deg, else "
                        f"{DEFAULT_MARGIN:g})")
    p.add_argument("--port", default=None)
    return p.parse_args()


class Getch:
    """Read single keypresses, decoding arrow keys, restoring the tty after."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.saved = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)

    def key(self):
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        # escape sequence: arrows arrive as ESC [ A/B/C/D
        if sys.stdin.read(1) != "[":
            return ch
        return {"C": ".", "D": ",", "A": "=", "B": "-"}.get(sys.stdin.read(1), ch)


def read_axis(arm, axis):
    """Current value of a real joint or a coupled wrist axis."""
    if axis not in VIRTUAL_AXES:
        return arm.read_encoder(axis)
    signs = VIRTUAL_AXES[axis]
    return sum(signs[j] * arm.read_encoder(j) for j in signs) / len(signs)


def axis_start(arm, axis):
    """Session-start value of an axis, in the same coordinates as read_axis."""
    if axis not in VIRTUAL_AXES:
        return arm._start_angles.get(axis, 0.0)
    signs = VIRTUAL_AXES[axis]
    return sum(signs[j] * arm._start_angles.get(j, 0.0)
               for j in signs) / len(signs)


def status(arm, joint, step, recorded, angle):
    lim = recorded.get(joint, {})
    if lim.get("continuous"):
        lo = hi = "CONT"
    else:
        lo = f"{lim['min']:+.1f}" if "min" in lim else "--"
        hi = f"{lim['max']:+.1f}" if "max" in lim else "--"
    start = axis_start(arm, joint)
    # Must fit an 80-column terminal: anything wider wraps, and the \r redraw
    # then only rewinds to the start of the wrapped row, smearing the display.
    return (f"J{joint} {JOINT_NAMES[joint]:<12} pos {angle:+8.2f} "
            f"trav {angle - start:+7.2f} stp {step:>4.1f} "
            f"lim {lo}/{hi}")


def read_limits_doc(path):
    """
    The limits file as a dict; {} if it does not exist yet.

    Raises ValueError on a file that exists but cannot be parsed, so a save
    never silently replaces a damaged file with a fragment of it.
    """
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        doc = json.load(f)
    if not isinstance(doc, dict):
        raise ValueError(f"{path}: expected a JSON object")
    return doc


def load_limits(path):
    """
    Reload previously measured ends so a later session adds to the file
    instead of replacing it. Reads the raw measured values, not the
    margin-backed-off ones, so the margin is never applied twice.
    """
    try:
        doc = read_limits_doc(path)
    except (OSError, ValueError) as e:
        print(f"  (could not read {path}: {e})")
        return {}
    out = {}
    for key, lim in doc.get("joints", {}).items():
        try:
            axis = int(key)
        except ValueError:
            continue
        entry = {}
        if lim.get("continuous"):
            entry["continuous"] = True
        if "measured_min" in lim:
            entry["min"] = float(lim["measured_min"])
        if "measured_max" in lim:
            entry["max"] = float(lim["measured_max"])
        if entry:
            out[axis] = entry
    return out


def _same(a, b):
    return a is not None and abs(float(a) - b) < 5e-4


def save_limits(recorded, margin, path):
    """
    Merge recorded ends into the limits file. Returns (saved_count, skipped).

    Merges rather than rewrites: the file also carries fields a jog session
    knows nothing about -- park_is_origin flags (which confirm_parked() needs),
    notes, and bounds tuned by hand. So:

      * joints not saved this time are left exactly as they are;
      * a bound is recomputed (measured end -/+ margin) only when that
        measured end actually changed -- an unchanged end keeps its bound;
      * a park_is_origin joint whose measured range contains 0 always keeps
        0 inside its window, or park.py could never return it to the origin.

    The file is replaced atomically, so an interrupted save cannot leave it
    half-written.
    """
    doc = read_limits_doc(path)
    joints = doc.setdefault("joints", {})
    before = json.dumps(joints, sort_keys=True)
    saved, skipped = 0, []
    for j, lim in sorted(recorded.items()):
        if lim.get("continuous"):
            # No mechanical stop: the axis turns freely. Recorded as such so
            # the safety layer knows there is nothing to bound it against.
            entry = joints.setdefault(str(j), {})
            for k in ("min", "max", "measured_min", "measured_max"):
                entry.pop(k, None)
            entry["continuous"] = True
            saved += 1
            continue
        if "min" not in lim or "max" not in lim:
            skipped.append((j, [k for k in ("min", "max") if k not in lim]))
            continue
        entry = joints.setdefault(str(j), {})
        entry.pop("continuous", None)
        lo_m, hi_m = round(lim["min"], 3), round(lim["max"], 3)
        keep_lo = _same(entry.get("measured_min"), lo_m) and "min" in entry
        keep_hi = _same(entry.get("measured_max"), hi_m) and "max" in entry
        lo = entry["min"] if keep_lo else lo_m + margin
        hi = entry["max"] if keep_hi else hi_m - margin
        if lo >= hi:  # margin ate the whole range
            lo, hi = lo_m, hi_m
        if entry.get("park_is_origin") and lo_m <= 0.0 <= hi_m:
            lo, hi = min(lo, 0.0), max(hi, 0.0)
        entry.update(min=round(lo, 3), max=round(hi, 3),
                     measured_min=lo_m, measured_max=hi_m)
        saved += 1

    doc.setdefault("_warning", (
        "Angles are relative to the encoder zero of the session in which "
        "they were measured. Encoders reset on power-on, so these are only "
        "valid across reboots once the arm is homed to a repeatable zero."
    ))
    doc.setdefault("reference", "encoder zero set with jog.py 'z' or home.py")
    doc.setdefault("margin_deg", margin)
    if json.dumps(joints, sort_keys=True) != before:
        doc["measured_at"] = time.strftime("%Y-%m-%d %H:%M:%S")

    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")
    os.replace(tmp, path)
    return saved, skipped


def main():
    args = parse_args()
    step_idx = min(range(len(STEP_SIZES)),
                   key=lambda i: abs(STEP_SIZES[i] - args.step))

    if args.margin is None:
        try:
            args.margin = float(read_limits_doc(LIMITS_PATH)
                                .get("margin_deg", DEFAULT_MARGIN))
        except (OSError, ValueError):
            args.margin = DEFAULT_MARGIN

    arm = ArctosArm(com_port=args.port)
    arm.connect()
    # Own copy so extending the guard never mutates the class-level table.
    arm.TRAVEL_LIMITS = dict(ArctosArm.TRAVEL_LIMITS)

    joint = 6                      # start distal: least consequence if wrong
    recorded = load_limits(LIMITS_PATH)
    if recorded:
        print("  reloaded previously measured ends for: "
              + ", ".join(f"J{j}" for j in sorted(recorded)))
    angle = read_axis(arm, joint)

    print("Keys:" + __doc__.split("Keys:")[1])
    print("\n  NOTE: 1-8 only SELECT an axis. Press , or . to actually move.")
    print(f"\nlimits file: {LIMITS_PATH}")
    print(f"hard ceiling: ±{args.max_travel:g}° from start pose\n")
    print(status(arm, joint, STEP_SIZES[step_idx], recorded, angle))

    try:
        with Getch() as g:
            while True:
                try:
                    k = g.key()
                except KeyboardInterrupt:
                    print("\n\ninterrupted — stopping motors")
                    arm.emergency_stop_all()
                    return
                step = STEP_SIZES[step_idx]

                if k == "q":
                    print("\nquit")
                    return
                elif k == "x":
                    arm.emergency_stop_all()
                    print("\n*** EMERGENCY STOP ***")
                elif k in "12345678":
                    joint = int(k)
                    angle = read_axis(arm, joint)
                    print(f"\033[2K\rselected J{joint} "
                          f"({JOINT_NAMES[joint]}) — press , or . to move it")
                elif k == "-":
                    step_idx = max(0, step_idx - 1)
                elif k == "=":
                    step_idx = min(len(STEP_SIZES) - 1, step_idx + 1)
                elif k in (",", "."):
                    delta = step if k == "." else -step
                    try:
                        angle = jog(arm, joint, delta, args)
                    except SensorLimit as e:
                        angle = read_axis(arm, joint)
                        print(f"\n  === SENSOR LIMIT: {e}")
                        print(f"      J{joint} at {angle:+.3f}° — this is a real "
                              f"end of travel. Press [ or ] to record it.")
                    except StopReached as e:
                        angle = read_axis(arm, joint)
                        print(f"\n  *** STOPPED (no completion): {e}")
                        print(f"      J{joint} rests at {angle:+.3f}° — this is "
                              f"NOT confirmed as a limit. Could be a stop, a "
                              f"stall under load, or fouling. Check before "
                              f"recording.")
                    except (SoftLimitExceeded, ArctosError) as e:
                        angle = read_axis(arm, joint)
                        print(f"\n  refused: {e}")
                    except KeyboardInterrupt:
                        # Ctrl+C mid-move: stop the motor and stay in the tool
                        # rather than unwinding with a traceback.
                        try:
                            arm.emergency_stop_all()
                        except ArctosError:
                            pass
                        try:
                            angle = read_axis(arm, joint)
                        except ArctosError:
                            pass
                        print("\n  *** interrupted mid-move — motors stopped. "
                              "Press q to quit.")
                elif k == "[":
                    recorded.setdefault(joint, {})["min"] = angle
                    print(f"\n  J{joint} MIN recorded at {angle:+.3f}°")
                elif k == "]":
                    recorded.setdefault(joint, {})["max"] = angle
                    print(f"\n  J{joint} MAX recorded at {angle:+.3f}°")
                elif k == "c":
                    recorded.pop(joint, None)
                    print(f"\n  J{joint} limits cleared for this session "
                          f"(the file keeps them until you record both ends "
                          f"again and save)")
                elif k == "z":
                    # Zeroing rewrites the board's reference, so make it a
                    # deliberate two-key action -- a stray keypress must not
                    # silently move the origin.
                    if joint in VIRTUAL_AXES:
                        print("\n  zero applies to a real joint (1-6), "
                              "not a coupled axis")
                        continue
                    print(f"\n  Set J{joint} zero at {angle:+.3f}°? "
                          f"press y to confirm, anything else to cancel", end="",
                          flush=True)
                    if g.key() != "y":
                        print("\n  cancelled")
                        continue
                    try:
                        arm.zero_here(joint)
                        angle = read_axis(arm, joint)
                        print(f"\n  J{joint} ORIGIN SET — now reads "
                              f"{angle:+.3f}°")
                    except ArctosError as e:
                        print(f"\n  zero failed: {e}")
                elif k == "n":
                    recorded.setdefault(joint, {})["continuous"] = True
                    recorded[joint].pop("min", None)
                    recorded[joint].pop("max", None)
                    print(f"\n  J{joint} marked CONTINUOUS (no mechanical stop)")
                elif k == "s":
                    try:
                        n, skipped = save_limits(recorded, args.margin,
                                                 LIMITS_PATH)
                    except (OSError, ValueError) as e:
                        print(f"\n  NOT saved — {LIMITS_PATH} is unreadable "
                              f"({e}). Fix or move it, then press s again.")
                        continue
                    print(f"\n  saved {n} axis range(s) -> {LIMITS_PATH}")
                    for j, missing in skipped:
                        print(f"    SKIPPED J{j}: no {' and no '.join(missing)} "
                              f"recorded (press [ and ] for both ends, "
                              f"or n if it turns freely)")
                elif k == "S":
                    # Shift-S, so a stray 's' still means "save limits".
                    # Reading a name needs the tty back in line mode.
                    termios.tcsetattr(g.fd, termios.TCSADRAIN, g.saved)
                    try:
                        name = input("\n  pose name (blank to cancel): ").strip()
                    finally:
                        tty.setcbreak(g.fd)
                    if not name:
                        print("  cancelled")
                    else:
                        try:
                            angles = {j: arm.read_encoder(j) for j in arm.JOINTS}
                        except ArctosError as e:
                            print(f"  could not read all joints: {e}")
                        else:
                            save_pose(name, angles, homed=sorted(arm._homed))
                            print(f"  saved pose {name!r}: "
                                  + " ".join(f"J{j}={a:+.2f}"
                                             for j, a in sorted(angles.items())))
                            if not arm._homed:
                                print("  WARNING: no joint has a confirmed "
                                      "origin, so this pose cannot be replayed "
                                      "until one is established.")
                elif k == "p":
                    print("\n  recorded so far:")
                    for j in sorted(recorded):
                        print(f"    J{j}: {recorded[j]}")
                    for name, pose in sorted(load_poses().items()):
                        print("    " + describe(name, pose))
                    continue
                else:
                    continue

                print("\033[2K\r" + status(arm, joint, STEP_SIZES[step_idx],
                                         recorded, angle), end="", flush=True)
    finally:
        arm.disconnect()


def step_timeout(arm, moves, rpm):
    """Seconds a step should need, plus slack -- the stop-detection window."""
    worst = max(abs(d) * arm.GEAR_RATIOS[j] / 360.0 for j, d in moves.items())
    return max(2.0, worst / max(rpm, 1) * 60.0 + 1.5)


def jog(arm, axis, delta, args):
    """
    Move one step, extending the travel guard by exactly this step.

    For a coupled wrist axis both motors are dispatched together and then
    waited on as a batch -- moving them one after the other would walk the
    differential along a diagonal that can foul a stop the straight path
    clears.
    """
    moves = {axis: delta} if axis not in VIRTUAL_AXES else {
        j: sign * delta for j, sign in VIRTUAL_AXES[axis].items()
    }

    plan = {}
    for j, d in moves.items():
        current = arm.read_encoder(j)
        start = arm._start_angles.get(j, current)
        needed = abs(current + d - start)
        if needed > args.max_travel:
            raise SoftLimitExceeded(
                f"joint {j}: {needed:.1f}° from start exceeds the "
                f"--max-travel ceiling of {args.max_travel:g}° — this is the "
                f"tool's software ceiling, NOT a mechanical stop. Press n if "
                f"this axis turns freely, or restart with a larger "
                f"--max-travel."
            )
        plan[j] = (d, current, needed)

    # Only widen the guard once every motor in the step has cleared its check,
    # so a refusal on the second motor leaves the first one's limit untouched.
    for j, (_, _, needed) in plan.items():
        if needed > arm.TRAVEL_LIMITS.get(j, 0.0):
            arm.TRAVEL_LIMITS[j] = needed

    futures = [
        arm.move_joint(j, d, rpm=args.rpm, acc=args.acc,
                       wait=False, on_limit="raise", current=cur)
        for j, (d, cur, _) in plan.items()
    ]
    tmo = step_timeout(arm, {j: d for j, (d, _, _) in plan.items()}, args.rpm)
    try:
        for f in futures:
            f.result(timeout=tmo)
    except FutureTimeout:
        # No completion frame: the joint could not follow. Cut the drive and
        # clear the pending futures so the next keypress starts clean.
        for j in plan:
            arm._fail_pending(j, CMD_MOVE_RELATIVE,
                              CommandTimeout("jog step timed out"))
            try:
                arm.emergency_stop(j)
            except ArctosError:
                pass
        raise StopReached(
            f"axis {axis}: step of {delta:+.2f}° did not complete within "
            f"{tmo:.1f}s — the joint is against a stop"
        )
    except LimitHit as e:
        # FD 03: the firmware stopped on a physical limit sensor. Unlike a
        # timeout this is unambiguous -- the joint reached a real end of
        # travel and the board said so. Highest-confidence limit there is.
        raise SensorLimit(f"axis {axis}: LIMIT SENSOR tripped ({e})")
    return read_axis(arm, axis)


if __name__ == "__main__":
    main()
