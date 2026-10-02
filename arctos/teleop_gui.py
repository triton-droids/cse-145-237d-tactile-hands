#!/usr/bin/env python3
"""
Teleop panel for the ARCTOS arm.

Per-joint nudge buttons with live encoder readout, an always-visible emergency
stop, and the limit window each joint is currently held to. Tkinter only -- no
extra dependencies, same stack as vr_calibration_gui.py.

All CAN traffic happens on one worker thread that owns the ArctosArm; the UI
posts commands to a queue and polls a shared state dict. Tkinter is not
thread-safe, so no widget is ever touched from the worker.

  python3 teleop_gui.py
  python3 teleop_gui.py --rpm 20
"""
import argparse
import queue
import threading
import tkinter as tk
from tkinter import ttk

from arctos_arm import ArctosArm, ArctosError, SoftLimitExceeded

JOINT_NAMES = {
    1: "base yaw", 2: "shoulder", 3: "elbow",
    4: "forearm roll", 5: "wrist B", 6: "wrist C",
}
STEPS = [0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]


class ArmWorker(threading.Thread):
    """Owns the arm. Commands in via queue, state out via a dict + lock."""

    def __init__(self, port=None, rpm=20, acc=2):
        super().__init__(daemon=True)
        self.cmds = queue.Queue()
        self.rpm = rpm
        self.acc = acc
        self._port = port
        self._lock = threading.Lock()
        self._state = {"connected": False, "status": "connecting...",
                       "angles": {}, "windows": {}, "busy": False}
        self._stop = False
        self.arm = None

    def snapshot(self):
        with self._lock:
            return dict(self._state, angles=dict(self._state["angles"]),
                        windows=dict(self._state["windows"]))

    def _set(self, **kw):
        with self._lock:
            self._state.update(kw)

    def post(self, kind, **kw):
        self.cmds.put((kind, kw))

    def estop(self):
        """
        Stop now, from the UI thread -- NOT via the command queue.

        The worker blocks for the whole of a jog (a 10° J3 step at 20 rpm is
        ~12 s), so a queued stop would wait behind the very move it is meant
        to stop. Queued jogs are discarded too, or they would run afterwards.
        emergency_stop_all() waits for acks, so it runs on its own thread to
        keep the window responsive; it also releases the worker's blocked
        move.
        """
        while True:
            try:
                self.cmds.get_nowait()
            except queue.Empty:
                break
        arm = self.arm
        if arm is None:
            return

        def stop():
            arm.emergency_stop_all()
            self._set(status="EMERGENCY STOP — all joints stopped")

        threading.Thread(target=stop, daemon=True, name="estop").start()

    def run(self):
        try:
            self.arm = ArctosArm(com_port=self._port)
            self.arm.connect()
        except Exception as e:
            self._set(status=f"connect failed: {e}", connected=False)
            return
        self._set(connected=True, status="ready")
        self._refresh()

        while not self._stop:
            try:
                kind, kw = self.cmds.get(timeout=0.4)
            except queue.Empty:
                if not self._state.get("busy"):
                    self._refresh()
                continue
            try:
                self._handle(kind, kw)
            except SoftLimitExceeded as e:
                self._set(status=f"blocked: {e}")
            except ArctosError as e:
                self._set(status=f"error: {e}")
            except Exception as e:                      # noqa: BLE001
                self._set(status=f"unexpected: {e}")
            finally:
                self._set(busy=False)

        try:
            self.arm.disconnect()
        except Exception:
            pass

    def _handle(self, kind, kw):
        if kind == "jog":
            j, delta = kw["joint"], kw["delta"]
            self._set(busy=True, status=f"J{j} {delta:+.2f}°...")
            self.arm.move_joint(j, delta, rpm=self.rpm, acc=self.acc,
                                wait=True, on_limit="clamp")
            self._set(status=f"J{j} moved {delta:+.2f}°")
        elif kind == "zero":
            j = kw["joint"]
            self.arm.zero_here(j)
            self._set(status=f"J{j} zeroed here")
        elif kind == "confirm_parked":
            self.arm.confirm_parked()
            self._set(status="origin confirmed — measured limits enforced")
        elif kind == "speed":
            self.rpm = kw["rpm"]
            self._set(status=f"speed {self.rpm} rpm")
        self._refresh()

    def _refresh(self):
        angles, windows = {}, {}
        for j in self.arm.JOINTS:
            try:
                angles[j] = self.arm.read_encoder(j)
            except ArctosError:
                angles[j] = None
            windows[j] = self.arm._limit_window(j)
        self._set(angles=angles, windows=windows)

    def shutdown(self):
        self._stop = True


class TeleopUI:
    def __init__(self, root, worker):
        self.worker = worker
        self.root = root
        root.title("ARCTOS teleop")
        self.step = tk.DoubleVar(value=1.0)

        top = ttk.Frame(root, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        ttk.Label(top, text="step (deg)").grid(row=0, column=0, padx=(0, 4))
        self.step_box = ttk.Combobox(top, width=6, state="readonly",
                                     values=[str(s) for s in STEPS])
        self.step_box.set("1.0")
        self.step_box.grid(row=0, column=1, padx=(0, 16))

        ttk.Label(top, text="speed (rpm)").grid(row=0, column=2, padx=(0, 4))
        self.rpm_box = ttk.Spinbox(top, from_=5, to=120, increment=5, width=6,
                                   command=self._speed)
        self.rpm_box.set(str(worker.rpm))
        self.rpm_box.grid(row=0, column=3, padx=(0, 16))

        stop = tk.Button(top, text="EMERGENCY STOP", bg="#c0392b", fg="white",
                         font=("TkDefaultFont", 11, "bold"),
                         command=worker.estop)
        stop.grid(row=0, column=4, ipadx=10, ipady=4)

        grid = ttk.Frame(root, padding=(8, 0, 8, 8))
        grid.grid(row=1, column=0, sticky="nsew")
        headers = ["joint", "position", "limits", "", "", "zero"]
        for c, h in enumerate(headers):
            ttk.Label(grid, text=h, font=("TkDefaultFont", 9, "bold")
                      ).grid(row=0, column=c, padx=4, pady=(6, 2), sticky="w")

        self.pos_labels, self.lim_labels = {}, {}
        for i, j in enumerate(sorted(JOINT_NAMES), start=1):
            ttk.Label(grid, text=f"J{j}  {JOINT_NAMES[j]}").grid(
                row=i, column=0, sticky="w", padx=4)
            self.pos_labels[j] = ttk.Label(grid, text="--", width=11,
                                           anchor="e", font="TkFixedFont")
            self.pos_labels[j].grid(row=i, column=1, padx=4)
            self.lim_labels[j] = ttk.Label(grid, text="--", width=26,
                                           font="TkFixedFont")
            self.lim_labels[j].grid(row=i, column=2, padx=4, sticky="w")
            ttk.Button(grid, text="  −  ", width=4,
                       command=lambda jj=j: self._jog(jj, -1)
                       ).grid(row=i, column=3, padx=2)
            ttk.Button(grid, text="  +  ", width=4,
                       command=lambda jj=j: self._jog(jj, +1)
                       ).grid(row=i, column=4, padx=2)
            ttk.Button(grid, text="set 0", width=6,
                       command=lambda jj=j: worker.post("zero", joint=jj)
                       ).grid(row=i, column=5, padx=4)

        bottom = ttk.Frame(root, padding=8)
        bottom.grid(row=2, column=0, sticky="ew")
        ttk.Button(bottom, text="Confirm arm is at park pose",
                   command=lambda: worker.post("confirm_parked")
                   ).grid(row=0, column=0, sticky="w")
        ttk.Label(bottom, text="(only if you have LOOKED at the arm — this "
                               "enables the measured limits)",
                  foreground="#777").grid(row=0, column=1, padx=8, sticky="w")

        self.status = ttk.Label(root, text="connecting...", relief="sunken",
                                anchor="w", padding=4)
        self.status.grid(row=3, column=0, sticky="ew")
        root.columnconfigure(0, weight=1)

        self._tick()

    def _speed(self):
        try:
            self.worker.post("speed", rpm=int(float(self.rpm_box.get())))
        except ValueError:
            pass

    def _jog(self, joint, sign):
        try:
            step = float(self.step_box.get())
        except ValueError:
            step = 1.0
        self.worker.post("jog", joint=joint, delta=sign * step)

    def _tick(self):
        s = self.worker.snapshot()
        self.status.config(text=s["status"])
        for j, lbl in self.pos_labels.items():
            a = s["angles"].get(j)
            lbl.config(text="--" if a is None else f"{a:+9.3f}°")
            w = s["windows"].get(j)
            if w is None:
                self.lim_labels[j].config(text="unbounded", foreground="#b00")
            else:
                lo, hi, src = w
                colour = "#060" if src == "measured" else "#a60"
                self.lim_labels[j].config(
                    text=f"[{lo:+7.1f},{hi:+7.1f}] {src[:8]}",
                    foreground=colour)
        self.root.after(200, self._tick)


def main():
    ap = argparse.ArgumentParser(description="ARCTOS teleop panel")
    ap.add_argument("--rpm", type=int, default=20)
    ap.add_argument("--acc", type=int, default=2)
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    worker = ArmWorker(port=args.port, rpm=args.rpm, acc=args.acc)
    worker.start()

    root = tk.Tk()
    TeleopUI(root, worker)
    try:
        root.mainloop()
    finally:
        worker.shutdown()


if __name__ == "__main__":
    main()
