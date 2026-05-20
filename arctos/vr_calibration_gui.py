"""
Manual + gesture 6-DOF calibration window.

6 rows for the controller->robot frame transform (X Y Z metres, Roll
Pitch Yaw degrees). Each row has a slider, a "-"/"+" nudge pair, and a
typable value box. Every change is written immediately and atomically to
vr_calibration.json; the publisher hot-reloads that file on change, so
the robot reacts live. Values also LOAD from the file on start.

GESTURE auto-calibration (orientation only):
  * "Capture FWD" — point the controller straight forward, click.
  * "Capture UP"  — point it straight up, click.
  Once both are captured it fits the frame (forward -> -Z, up -> +Y)
  and drops the result into the Roll/Pitch/Yaw sliders (X/Y/Z stay 0).
  Needs the publisher running (it subscribes to the live pose). If ROS
  is unavailable the sliders still work; only capture is disabled.

Run under SYSTEM python with ROS sourced (same as the visualizer):

    source /opt/ros/jazzy/setup.bash
    /usr/bin/python3 vr_calibration_gui.py
    # or via the launcher (publisher + this):
    ./run_vr_teleop.sh --cal

"Reset" zeros all 6. "Reload" re-reads the file. Close the window or
Ctrl+C to quit — the saved calibration stays in effect.
"""

import argparse
import os
import signal
import sys
import time

import numpy as np

from vr_ros_io import (
    CALIB_PATH,
    CAL_KEYS,
    DEFAULT_CAL,
    gesture_calibration,
    load_cal_params,
    save_cal_params,
)

# (key, label, min, max, nudge-step, decimals)
SPECS = [
    ("x",         "X (m)",      -1.0,   1.0,   0.01, 3),
    ("y",         "Y (m)",      -1.0,   1.0,   0.01, 3),
    ("z",         "Z (m)",      -1.0,   1.0,   0.01, 3),
    ("roll_deg",  "Roll (deg)", -180.0, 180.0, 1.0,  1),
    ("pitch_deg", "Pitch (deg)", -180.0, 180.0, 1.0, 1),
    ("yaw_deg",   "Yaw (deg)",  -180.0, 180.0, 1.0,  1),
]

CAP_SECONDS = 0.5     # how long to average the pointing axis per capture
POSE_FRESH_S = 0.3    # reject captures if the pose is older than this


def main():
    argparse.ArgumentParser(description=__doc__).parse_args()

    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, Slider, TextBox

    backend = matplotlib.get_backend().lower()
    if "agg" in backend and backend != "qtagg":
        raise SystemExit(
            f"[cal] non-interactive backend '{backend}' — no window. "
            f"Install a GUI backend: pip install PyQt6  (gives QtAgg)."
        )

    # Live pose source for gesture capture. Optional: without ROS the
    # sliders still work, capture is just disabled.
    vr = None
    try:
        from vr_ros_io import ControllerSubscriber
        vr = ControllerSubscriber(node_name="vr_cal_gui")
        print("[cal] subscribed to VR pose (gesture capture enabled).")
    except Exception as e:
        print(f"[cal] ROS unavailable ({e}); gesture capture disabled, "
              f"sliders still work.")

    params = load_cal_params(CALIB_PATH)
    print(f"[cal] editing {CALIB_PATH}")
    print(f"[cal] loaded: {params}")

    fig = plt.figure(figsize=(9.5, 5.4))
    fig.canvas.manager.set_window_title("ARCTOS VR calibration")
    fig.suptitle("6-DOF controller->robot transform  "
                 "(saves live — publisher hot-reloads)", fontsize=10)

    sliders, boxes, decimals = {}, {}, {}
    rng = {k: (lo, hi) for k, _l, lo, hi, _s, _d in SPECS}
    # Re-entrancy guard: set_val on a Slider/TextBox fires its own
    # callbacks; without this the slider<->box<->save loop recurses.
    busy = {"on": False}

    def fmt(key, v):
        return f"{v:.{decimals[key]}f}"

    def commit(key, v, lo, hi):
        """Clamp, sync slider+box, persist — the single source of truth."""
        if busy["on"]:
            return
        busy["on"] = True
        try:
            v = max(lo, min(hi, float(v)))
            sliders[key].set_val(v)
            boxes[key].set_val(fmt(key, v))
            save_cal_params({k: sliders[k].val for k in CAL_KEYS},
                            CALIB_PATH)
        finally:
            busy["on"] = False

    n = len(SPECS)
    top, bot = 0.83, 0.26
    row_h = (top - bot) / n
    for i, (key, label, lo, hi, step, dec) in enumerate(SPECS):
        decimals[key] = dec
        y = top - (i + 1) * row_h + row_h * 0.18
        h = row_h * 0.5
        v0 = max(lo, min(hi, params.get(key, 0.0)))

        ax_s = fig.add_axes([0.12, y, 0.46, h])
        s = Slider(ax_s, label, lo, hi, valinit=v0)
        sliders[key] = s

        ax_minus = fig.add_axes([0.61, y, 0.05, h])
        ax_box = fig.add_axes([0.685, y, 0.13, h])
        ax_plus = fig.add_axes([0.825, y, 0.05, h])
        b_minus = Button(ax_minus, "-")
        b_plus = Button(ax_plus, "+")
        tb = TextBox(ax_box, "", initial=fmt(key, v0))
        boxes[key] = tb

        # Bind per-row (default args capture loop vars).
        def on_slide(val, k=key, lo=lo, hi=hi):
            commit(k, val, lo, hi)

        def on_submit(text, k=key, lo=lo, hi=hi):
            try:
                v = float(text)
            except ValueError:
                v = sliders[k].val            # bad input -> revert
            commit(k, v, lo, hi)

        def on_minus(_evt, k=key, st=step, lo=lo, hi=hi):
            commit(k, sliders[k].val - st, lo, hi)

        def on_plus(_evt, k=key, st=step, lo=lo, hi=hi):
            commit(k, sliders[k].val + st, lo, hi)

        s.on_changed(on_slide)
        tb.on_submit(on_submit)
        b_minus.on_clicked(on_minus)
        b_plus.on_clicked(on_plus)
        # Keep refs alive (Button/TextBox must not be GC'd).
        ax_minus._w, ax_plus._w, ax_box._w = b_minus, b_plus, tb

    # ---- status line + gesture/util button row ----------------------
    status = fig.text(0.5, 0.205, "", ha="center", fontsize=9,
                      family="monospace")

    def say(msg, color="black"):
        status.set_text(msg)
        status.set_color(color)
        fig.canvas.draw_idle()

    caps = {"fwd": None, "up": None}

    def grab_dir(which):
        """Average the controller's pointing axis (room -Z) for
        CAP_SECONDS. Returns the unit direction, or None if no fresh
        pose arrived."""
        samples = []
        t0 = time.time()
        while time.time() - t0 < CAP_SECONDS:
            _p, R, age, _c, _ca = vr.latest()
            if R is not None and age <= POSE_FRESH_S:
                samples.append(np.asarray(R) @ np.array([0.0, 0.0, -1.0]))
            time.sleep(0.02)
        if not samples:
            return None
        return np.mean(samples, axis=0)

    def try_solve():
        if caps["fwd"] is None or caps["up"] is None:
            return
        try:
            d = gesture_calibration(caps["fwd"], caps["up"])
        except ValueError as e:
            say(f"gesture failed: {e}", "tab:red")
            return
        for k in ("roll_deg", "pitch_deg", "yaw_deg"):
            commit(k, d[k], *rng[k])
        say(f"calibrated: R={d['roll_deg']:+.1f} "
            f"P={d['pitch_deg']:+.1f} Y={d['yaw_deg']:+.1f}  (saved)",
            "tab:green")
        print(f"[cal] gesture calibration applied: {d}")

    def capture(which):
        if vr is None:
            say("ROS not available — gesture capture disabled",
                "tab:red")
            return
        say(f"capturing {which.upper()} … hold still", "tab:blue")
        v = grab_dir(which)
        if v is None:
            say(f"{which.upper()} capture: no fresh pose "
                f"(publisher up? controller tracked?)", "tab:red")
            return
        caps[which] = v
        have = " ".join(s.upper() for s in ("fwd", "up")
                        if caps[s] is not None)
        say(f"{which.upper()} captured. have: [{have}]", "tab:green")
        try_solve()

    # Row of action buttons along the bottom.
    specs_btn = [
        ("Capture FWD", lambda _e: capture("fwd")),
        ("Capture UP",  lambda _e: capture("up")),
        ("Reset",       None),
        ("Reload",      None),
    ]
    bw, gap = 0.19, 0.02
    x0 = (1.0 - (len(specs_btn) * bw + (len(specs_btn) - 1) * gap)) / 2
    btns = []
    for j, (lbl, cb) in enumerate(specs_btn):
        ax_b = fig.add_axes([x0 + j * (bw + gap), 0.06, bw, 0.08])
        b = Button(ax_b, lbl)
        btns.append(b)
        if cb is not None:
            b.on_clicked(cb)

    def do_reset(_):
        caps["fwd"] = caps["up"] = None
        for k, _l, lo, hi, _s, _d in SPECS:
            commit(k, DEFAULT_CAL[k], lo, hi)
        say("reset to identity", "black")
        print("[cal] reset to identity")

    def do_reload(_):
        d = load_cal_params(CALIB_PATH)
        for k, _l, lo, hi, _s, _d in SPECS:
            commit(k, d.get(k, 0.0), lo, hi)
        say("reloaded from file", "black")
        print(f"[cal] reloaded from file: {d}")

    btns[2].on_clicked(do_reset)
    btns[3].on_clicked(do_reload)

    if vr is not None:
        say("gesture: point FWD -> Capture FWD, point UP -> Capture UP")
    else:
        say("ROS not available — sliders only (capture disabled)",
            "tab:red")

    # ControllerSubscriber reclaims SIGINT and *raises* KeyboardInterrupt
    # at an arbitrary point — inside the Qt loop PyQt swallows it and the
    # window never closes. Override with a cooperative handler that just
    # closes the figures so plt.show() returns and teardown runs.
    def _stop(_signo, _frame):
        try:
            plt.close("all")
        except Exception:
            pass

    if vr is not None:
        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)

    print("[cal] window open. Drag / nudge / type / gesture-capture — "
          "saves instantly. Close window or Ctrl+C to quit.")
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    print("[cal] bye (saved calibration stays in effect)")
    if vr is not None:
        vr.shutdown()
        # rclpy/DDS daemon thread std::terminate()s on normal teardown;
        # this tool owns no hardware, so flush and hard-exit cleanly.
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
