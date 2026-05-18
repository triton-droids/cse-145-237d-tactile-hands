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


def run_3d(vr, target_fps):
    import matplotlib
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    backend = matplotlib.get_backend().lower()
    if "agg" in backend and backend != "qtagg":
        print(f"[viz] WARNING: non-interactive backend '{backend}' — no "
              f"window will show. Install a GUI backend: "
              f"pip install PyQt6   (then it uses QtAgg). Falling back "
              f"to text mode.")
        return run_text(vr)

    print(f"[viz] 3D mode ({matplotlib.get_backend()}), target "
          f"{target_fps:.0f} FPS. Hold B to set reference. Close window "
          f"or Ctrl+C to quit.")
    try:
        plt.ion()
        fig = plt.figure(figsize=(8, 7))
        # Force the GUI canvas to realize now so a backend/plugin
        # failure (e.g. missing libxcb-cursor0) is catchable here
        # instead of aborting mid-run.
        fig.canvas.draw()
    except Exception as e:
        print(f"[viz] 3D backend failed to start ({e}). If this is the "
              f"Qt xcb plugin: sudo apt install libxcb-cursor0. Falling "
              f"back to text mode.")
        return run_text(vr)
    ax = fig.add_subplot(111, projection="3d")
    # Plot axes are remapped so "up" is visually up:
    #   plot-X = room X (right), plot-Y = room Z (fwd/back),
    #   plot-Z = room Y (up / J2 source).
    ax.set_xlabel("X right (m)")
    ax.set_ylabel("Z fwd/back (m)")
    ax.set_zlabel("Y up / J2 source (m)")
    ax.set_title("VR right controller — SteamVR room frame (via ROS)")
    HALF = 0.4  # half-extent of the view cube (m)

    # --- Persistent artists: created once, data updated each frame ----
    (trail_ln,) = ax.plot([], [], [], lw=1, alpha=0.5, color="gray")
    (pt_ln,) = ax.plot([], [], [], "o", ms=9, color="tab:blue")
    (ref_ln,) = ax.plot([], [], [], "*", ms=14, color="tab:purple")
    (refseg_ln,) = ax.plot([], [], [], "--", color="tab:purple",
                           alpha=0.6)
    triad = {
        c: ax.plot([], [], [], color=col, lw=2)[0]
        for c, col in (("fwd", "tab:red"), ("up", "tab:green"),
                       ("right", "tab:orange"))
    }
    info = ax.text2D(0.02, 0.98, "", transform=ax.transAxes, va="top",
                     fontsize=9, family="monospace")

    trail = collections.deque(maxlen=TRAIL_LEN)
    src_ref = None
    ref_p = None
    prev_clutch = False
    center = None
    period = 1.0 / target_fps
    fps_t = time.time()
    fps_n = 0
    fps = 0.0

    def set3d(ln, xs, ys, zs):
        # Inputs are room (x, y, z); swap y<->z so room-up (Y) is the
        # plot's vertical axis.
        ln.set_data(xs, zs)
        ln.set_3d_properties(ys)

    while plt.fignum_exists(fig.number):
        t0 = time.time()
        ok, p, R, clutch = poll(vr)
        if clutch and not prev_clutch and ok:
            src_ref = {a["joint"]: read_source(a["source"], p, R)
                       for a in AXES}
            ref_p = p.copy()
        if not clutch and prev_clutch:
            src_ref = None
            ref_p = None
        prev_clutch = clutch

        if ok:
            trail.append(p.copy())
            T = np.array(trail)
            set3d(trail_ln, T[:, 0], T[:, 1], T[:, 2])
            set3d(pt_ln, [p[0]], [p[1]], [p[2]])

            L = 0.12
            for key, vec in (("fwd", R @ [0, 0, -1]),
                             ("up", R @ [0, 1, 0]),
                             ("right", R @ [1, 0, 0])):
                set3d(triad[key],
                      [p[0], p[0] + vec[0] * L],
                      [p[1], p[1] + vec[1] * L],
                      [p[2], p[2] + vec[2] * L])

            if ref_p is not None:
                set3d(ref_ln, [ref_p[0]], [ref_p[1]], [ref_p[2]])
                set3d(refseg_ln, [ref_p[0], p[0]], [ref_p[1], p[1]],
                      [ref_p[2], p[2]])
            else:
                set3d(ref_ln, [], [], [])
                set3d(refseg_ln, [], [], [])

            # Recenter only when the controller nears the cube edge —
            # constant autoscale is what made the old version crawl.
            if center is None or np.any(np.abs(p - center) > HALF * 0.6):
                center = p.copy()
                # plot X=room X, plot Y=room Z, plot Z=room Y (up).
                ax.set_xlim(center[0] - HALF, center[0] + HALF)
                ax.set_ylim(center[2] - HALF, center[2] + HALF)
                ax.set_zlim(center[1] - HALF, center[1] + HALF)

            tgt = axis_targets(p, R, src_ref)
            lines = [
                f"FPS: {fps:4.1f}",
                f"clutch: {'B-DOWN' if clutch else 'released'}",
                f"pos (m): {p[0]:+.3f} {p[1]:+.3f} {p[2]:+.3f}",
            ]
            for ax_ in AXES:
                j = ax_["joint"]
                s, d, dt = tgt[j]
                u = SRC_UNIT[ax_["source"]]
                lines.append(
                    f"J{j}: {ax_['source']}={s:+.2f}{u}" if dt is None
                    else f"J{j}: {ax_['source']}={s:+.2f}{u}  "
                         f"Δ={d:+.2f}{u}  target {dt:+.1f}°"
                )
            info.set_text("\n".join(lines))
            info.set_color("black")
        else:
            info.set_text("no fresh VR data\n(publisher up? "
                          "controller tracked?)")
            info.set_color("red")

        fig.canvas.draw_idle()
        fig.canvas.flush_events()

        fps_n += 1
        if time.time() - fps_t >= 0.5:
            fps = fps_n / (time.time() - fps_t)
            fps_t = time.time()
            fps_n = 0

        dt = period - (time.time() - t0)
        if dt > 0:
            time.sleep(dt)

    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--text", action="store_true",
                    help="force text mode even if matplotlib is available")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="target 3D render FPS (default 30)")
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
        if use_3d:
            run_3d(vr, args.fps)
        else:
            run_text(vr)
    except KeyboardInterrupt:
        print("\n[viz] bye")
    finally:
        vr.shutdown()


if __name__ == "__main__":
    main()
