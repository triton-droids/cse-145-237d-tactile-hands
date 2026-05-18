"""
Standalone VR controller visualizer — a debug aid for the teleop.

Reads the SAME ROS topics the teleop consumes (published by
vr_controller_publisher.py), so "what you see is what the robot gets".
It does NOT touch the arm and does NOT open OpenVR — run it alongside
the teleop, or by itself with just the publisher:

    # via the launcher (publisher + teleop + this):
    ./run_vr_teleop.sh --viz
    # or standalone, with the publisher already running:
    /usr/bin/python3 vr_visualizer.py

It mirrors the teleop's clutch logic for display only: press B to
capture a local reference; it then shows, per axis, the deadbanded
source offset and the joint target the teleop would command (scale +
invert from vr_ros_io.AXES). Nothing here moves a motor.

Modes:
  * 3D mode (needs matplotlib): live plot — controller position + trail,
    orientation triad, clutch reference, info panel.   pip install matplotlib
  * Text mode (no extra deps): one-line live readout. Auto-used if
    matplotlib is missing, or force with --text.

Ctrl+C to quit.
"""

import argparse
import collections
import time

import numpy as np

from vr_ros_io import (
    AXES,
    SRC_DEADBAND,
    SRC_UNIT,
    ControllerSubscriber,
    deadband,
    read_source,
)

TRAIL_LEN = 240
VR_STALE_S = 0.25


def axis_targets(p, R, src_ref):
    """Per-axis (source value, deadbanded offset, joint-target delta)."""
    out = {}
    for ax in AXES:
        j = ax["joint"]
        src = read_source(ax["source"], p, R)
        if src_ref is None or j not in src_ref:
            out[j] = (src, None, None)
            continue
        d = deadband(src - src_ref[j], SRC_DEADBAND[ax["source"]])
        sign = -1.0 if ax["invert"] else 1.0
        out[j] = (src, d, sign * ax["scale"] * d)
    return out


def fmt_axes(tgt):
    parts = []
    for ax in AXES:
        j = ax["joint"]
        src, d, dt = tgt[j]
        u = SRC_UNIT[ax["source"]]
        if dt is None:
            parts.append(f"J{j}[{ax['source']}={src:+.2f}{u}]")
        else:
            parts.append(
                f"J{j}[{ax['source']}={src:+.2f}{u} "
                f"Δ={d:+.2f}{u} → {dt:+.1f}°]"
            )
    return "  ".join(parts)


def poll(vr):
    """(ok, p, R, clutch) with staleness folded into ok."""
    p, R, pose_age, clutch, clutch_age = vr.latest()
    ok = (
        p is not None
        and pose_age <= VR_STALE_S
        and clutch_age <= VR_STALE_S
    )
    return ok, p, R, (clutch if ok else False)


def run_text(vr):
    print("[viz] TEXT mode. Hold B to set reference. Ctrl+C to quit.")
    src_ref = None
    prev_clutch = False
    while True:
        ok, p, R, clutch = poll(vr)
        if clutch and not prev_clutch and ok:
            src_ref = {ax["joint"]: read_source(ax["source"], p, R)
                       for ax in AXES}
        if not clutch and prev_clutch:
            src_ref = None
        prev_clutch = clutch

        if not ok:
            line = "no fresh VR data (publisher up? controller tracked?)"
        else:
            tgt = axis_targets(p, R, src_ref)
            cl = "B-DOWN" if clutch else "  --  "
            line = (f"pos[{p[0]:+.2f} {p[1]:+.2f} {p[2]:+.2f}]m  "
                    f"{cl}  {fmt_axes(tgt)}")
        print("\r\033[K" + line, end="", flush=True)
        time.sleep(1.0 / 30.0)


def run_3d(vr):
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    print("[viz] 3D mode. Hold B to set reference. Close window or "
          "Ctrl+C to quit.")
    fig = plt.figure(figsize=(8, 7))
    ax = fig.add_subplot(111, projection="3d")
    trail = collections.deque(maxlen=TRAIL_LEN)
    src_ref = None
    ref_p = None
    prev_clutch = False

    while plt.fignum_exists(fig.number):
        ok, p, R, clutch = poll(vr)
        if clutch and not prev_clutch and ok:
            src_ref = {a["joint"]: read_source(a["source"], p, R)
                       for a in AXES}
            ref_p = p.copy()
        if not clutch and prev_clutch:
            src_ref = None
            ref_p = None
        prev_clutch = clutch

        ax.cla()
        ax.set_xlabel("X right (m)")
        ax.set_ylabel("Y up / J2 source (m)")
        ax.set_zlabel("Z (m)")
        ax.set_title("VR right controller — SteamVR room frame (via ROS)")

        if ok:
            trail.append(p.copy())
            T = np.array(trail)
            ax.plot(T[:, 0], T[:, 1], T[:, 2], lw=1, alpha=0.5,
                    color="gray")
            ax.scatter(*p, s=80, color="tab:blue")

            L = 0.12
            for vec, c, lbl in (
                (R @ [0, 0, -1], "tab:red", "fwd"),
                (R @ [0, 1, 0], "tab:green", "up"),
                (R @ [1, 0, 0], "tab:orange", "right"),
            ):
                ax.quiver(p[0], p[1], p[2], vec[0] * L, vec[1] * L,
                          vec[2] * L, color=c)
                ax.text(p[0] + vec[0] * L, p[1] + vec[1] * L,
                        p[2] + vec[2] * L, lbl, color=c, fontsize=8)

            if ref_p is not None:
                ax.scatter(*ref_p, s=120, marker="*", color="tab:purple")
                ax.plot([ref_p[0], p[0]], [ref_p[1], p[1]],
                        [ref_p[2], p[2]], "--", color="tab:purple",
                        alpha=0.6)

            c = p
            r = 0.4
            ax.set_xlim(c[0] - r, c[0] + r)
            ax.set_ylim(c[1] - r, c[1] + r)
            ax.set_zlim(c[2] - r, c[2] + r)

            tgt = axis_targets(p, R, src_ref)
            lines = [
                f"clutch: {'B-DOWN' if clutch else 'released'}",
                f"pos (m): {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}",
            ]
            for ax_ in AXES:
                j = ax_["joint"]
                s, d, dt = tgt[j]
                u = SRC_UNIT[ax_["source"]]
                if dt is None:
                    lines.append(f"J{j}: {ax_['source']}={s:+.2f}{u}")
                else:
                    lines.append(
                        f"J{j}: {ax_['source']}={s:+.2f}{u}  "
                        f"Δ={d:+.2f}{u}  target {dt:+.1f}°"
                    )
            ax.text2D(0.02, 0.98, "\n".join(lines),
                      transform=ax.transAxes, va="top", fontsize=9,
                      family="monospace")
        else:
            ax.text2D(0.5, 0.5,
                      "no fresh VR data\n(publisher up? controller "
                      "tracked?)",
                      transform=ax.transAxes, ha="center", color="red")

        plt.pause(1.0 / 30.0)

    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", action="store_true",
                    help="force text mode even if matplotlib is available")
    args = ap.parse_args()

    use_3d = not args.text
    if use_3d:
        try:
            import matplotlib  # noqa: F401
        except ImportError:
            print("[viz] matplotlib not installed — text mode. "
                  "For 3D: pip install matplotlib")
            use_3d = False

    print("[viz] subscribing to VR ROS topics…")
    vr = ControllerSubscriber(node_name="vr_visualizer")
    try:
        (run_3d if use_3d else run_text)(vr)
    except KeyboardInterrupt:
        print("\n[viz] bye")
    finally:
        vr.shutdown()


if __name__ == "__main__":
    main()
