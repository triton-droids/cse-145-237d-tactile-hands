# VR Teleop — Valve Index → ARCTOS arm + AmazingHand

Right-hand Valve Index drives the **ARCTOS 6-DOF arm** (closed-loop on
the MKS SERVO encoders) and the **AmazingHand** (8 SCS0009 servos)
from the Index's capacitive finger sensing. A 3D visualizer and a
6-DOF calibration window run alongside.

## Quick start

```bash
# (one-time) system deps, see "Requirements" below
./run_vr_teleop.sh                       # everything: pub + teleop + viz + cal + hand
./run_vr_teleop.sh --no-hand             # everything but the hand
./run_vr_teleop.sh --no-viz --no-cal     # bare teleop + hand
./run_vr_teleop.sh --no-viz --no-cal --no-hand   # bare teleop
./run_vr_teleop.sh --text                # text-mode visualizer
```

`Ctrl+C` once tears the whole stack down cleanly. If something is ever
stuck (orphaned process, busy serial port), run **`bash cleanup.sh`**.

## Architecture

```
                  +----------------------------+
   SteamVR <----> | vr_controller_publisher.py | (the ONLY OpenVR client)
                  +-------------+--------------+
                                | ROS 2 topics
                                | /vr/right_controller/pose   (PoseStamped)
                                | /vr/clutch                  (Bool, B button)
                                | /vr/finger_curls            (Float32MultiArray, 5)
                                | /vr/trigger , /vr/grip      (Float32)
                                v
   +-------------------+   +----------------+   +------------------+
   | vr_teleop_rot.py  |   | vr_hand_ctl.py |   |  vr_visualizer.py|
   |  (ARCTOS arm)     |   |  (AmazingHand) |   |  (3D / text)     |
   +-------------------+   +----------------+   +------------------+
            |                       |                    ^
            v                       v                    |
       /dev/ttyACM0            /dev/ttyACM1     +------------------+
       (CANable, MKS CAN)      (SCS bus)        |vr_calibration_gui|
                                                |  edits the JSON  |
                                                +------------------+
                                                          ^
                                                          | hot-reload
                                                          v
                                                  vr_calibration.json
                                                  (6-DOF transform)
```

One publisher owns SteamVR. Every consumer reads ROS — no second OpenVR
client, no shared state to corrupt.

## Requirements

| Layer | Used as | Why |
|---|---|---|
| `/usr/bin/python3` (system Python) | runs every VR script | `rclpy` is not importable from conda |
| ROS 2 Jazzy at `/opt/ros/jazzy` | sourced by the launcher | `rclpy`, message types |
| SteamVR | runs alongside | OpenVR action manifest in `teleop_actions.json` |
| `openvr`, `python-can`, `rustypot`, `matplotlib`, `PyQt6` | pip into system python | controller, MKS CAN, SCS servos, plots, GUI |
| `libxcb-cursor0` | apt | Qt6 xcb platform plugin |

One-shot install: `bash /tmp/install_vr_ros_deps.sh` (the script that
got us here).

### Devices

- **CANable** (USB VID `16d0:117e`) at `/dev/ttyACM0` — MKS SERVO42/57D CAN bus.
- **QinHeng CH340-ish** (USB VID `1a86:55d3`) at `/dev/ttyACM1` — AmazingHand SCS bus.
- The arm-port resolver prefers the CANable by USB VID — wrong port for the arm is impossible.
- Be in the `dialout` group (`sudo usermod -aG dialout $USER`, then re-login).

## The launcher

`run_vr_teleop.sh` sources ROS, starts everything in dependency order
(publisher first), and on exit tears every child down with `SIGINT`
first (so the teleop stops the arm + frees CAN, the hand opens + goes
limp, the GUIs save and close) and `SIGKILL` after ≤ 5 s as a backstop.

| Flag | Effect |
|---|---|
| _(none)_ | publisher + teleop + visualizer + calibration GUI + hand (`--live`) |
| `--no-viz` | skip the visualizer |
| `--no-cal` | skip the calibration sliders window |
| `--no-hand` | skip the AmazingHand |
| `--text` | text-mode visualizer instead of 3D |

Unknown flags are rejected (including the old `--viz/--cal/--hand`,
which are now defaults).

## Scripts

| File | Role |
|---|---|
| `vr_controller_publisher.py` | OpenVR → ROS. Reads pose / clutch / skeletal finger curls / trigger / grip. Applies the 6-DOF calibration from `vr_calibration.json` (hot-reloaded on mtime change) before publishing. |
| `vr_teleop_rotation.py` | The actual arm teleop. Subscribes to pose+clutch, runs a per-joint position P-loop (closed on each MKS encoder) over the arm in speed mode. |
| `vr_hand_control.py` | Subscribes to `/vr/finger_curls`, drives the 8 SCS servos. `--live` actually moves; default is a dry-run print. |
| `vr_visualizer.py` | 3D matplotlib (or text) view of the controller, trail, orientation triad, clutch reference, RPY readout, and what each joint would be commanded to. |
| `vr_calibration_gui.py` | 6 slider rows (X Y Z m, Roll Pitch Yaw deg) with `-`/`+` nudges, typable boxes, plus **Capture FWD** / **Capture UP** gesture calibration. Writes `vr_calibration.json`. |
| `vr_ros_io.py` | Shared library: topic names, axis config (`AXES`), filtering, calibration math, ROS subscribers. No `openvr`, no `python-can` — viz and cal can import it without arm/HMD deps. |
| `cleanup.sh` | Kill any orphaned VR processes and free `ttyACM0`/`ttyACM1`. Safe to run anytime. |
| `arctos_arm.py` | Pre-existing arm driver. The teleop only adds `set_joint_speed()` / `stop_joint()` on top. See the other `README.md`. |

## Files written/read

| File | Owner | Schema |
|---|---|---|
| `vr_calibration.json` | written by `vr_calibration_gui.py`, read by `vr_controller_publisher.py` | 6 floats: `x,y,z` (m), `roll_deg,pitch_deg,yaw_deg`. All-zero ⇒ identity (no-op). Hand-editable. |
| `teleop_actions.json` | SteamVR action manifest (poses, B clutch, A calibrate, trigger, grip, skeleton). |
| `teleop_binding.json` | Knuckles binding for the actions above. |

## Calibration

Two ways, both edit the same file and the publisher hot-reloads on
every change.

**Manual (sliders).** Drag, click `-`/`+`, or type a number in the box.
Identity is `0` everywhere.

**Gesture (auto, orientation-only).**
1. Point the controller **straight forward** → click **Capture FWD**.
2. Point it **straight up** → click **Capture UP**.

The GUI fits the frame so "forward" maps to `−Z` and "up" maps to
`+Y` (Gram-Schmidt: `right = unit(f × u)`, `up = right × f`,
`R_cal = [right | up | −f]`), writes the resulting RPY into the
sliders, and saves. `X/Y/Z` stay 0.

This **does not eliminate gimbal lock** at vertical pointing — it
relocates it so your normal operating posture (forward) is well-
conditioned. The yaw readout will still be noisy/undefined when you
point near-straight-up.

## Joint mapping

Defined in `vr_ros_io.AXES`. While B is held:

| Joint | Source | Notes |
|---|---|---|
| J1 (base yaw) | controller yaw | scale 0.25, ±45°, 10° centered deadband, One-Euro jitter filter |
| J2 (shoulder) | controller **room +Y** (height) | inverted, scale 240, fast (up to 240 rpm) |
| J3 (elbow) | controller **room +Y** | same as J2, **not** inverted — moves with J2 when you raise/lower your hand |
| J4 (forearm roll) | **disabled** (commented out in `AXES`) | un-comment the block to re-enable |
| J5 / J6 | not driven | differentially coupled, not implemented in `arctos_arm.py` |

Per-axis, per tick:
```
target = joint_ref + dir * scale * (source_now - source_ref)
target = clamp(target, joint_ref ± max_travel)
rpm    = clip(KP * (target - encoder), ±max_rpm)   # with per-joint rpm_floor
```

## Controls

| Input | Action |
|---|---|
| **B** held | deadman clutch — joints only move while held. Releasing snaps everything to a stop and re-pressing re-zeros references (no jump). |
| controller pose | drives J1/J2/J3 per the table above |
| skeletal finger curls | drive the AmazingHand (4 fingers, pinky ignored — there are only 4 on the hand) |
| trigger / grip | published as fallback if skeletal data isn't available (curls = `clip(trig + 0.5·grip)`) |

## Safety nets (in the teleop)

- **Deadman clutch** — release B and every joint commands 0.
- **Closed-loop on encoders** — each joint reads its MKS encoder; targets aren't integrated.
- **Stale data → 0** — no fresh ROS pose/clutch within 0.25 s, or no fresh encoder within 0.4 s, commands 0 for that joint.
- **Per-axis travel clamp** — target capped to `joint_ref ± max_travel`.
- **Runaway guard** — encoder past the clamp + margin ⇒ all joints lock out; release B and re-press to recover.
- **Atomic teardown** — finally-blocks first call `disarm_term_signals()` (SIG_IGN on SIGINT/SIGTERM) so a second Ctrl+C during cleanup can't skip arm-stop / port-free.
- **`arctos_arm.__exit__`** — E-stops every joint on disconnect, always.

## Troubleshooting

**"X isn't working" / hand or arm idle.** 95 % of the time the
publisher (or X itself) isn't running. Check:
`pgrep -af 'vr_controller_publisher|vr_teleop_rotation|vr_hand_control'`.
If empty / partial, run `bash cleanup.sh && ./run_vr_teleop.sh`.

**Processes won't die on `Ctrl+C`.** Run `bash cleanup.sh`. The
launcher's `cleanup()` has been hardened against the previous deadlock,
but a wedged old instance from before that fix can still get stuck —
`cleanup.sh` is the universal reset.

**`Device or resource busy` on `/dev/ttyACM*`.** Another VR process
still holds it. `cleanup.sh` lists holders and SIGKILLs them.

**`PermissionError` on a `ttyACM*`.** You're not in `dialout`. Add
yourself and re-login.

**Yaw jumps / J1 unstable.** Most likely you're pointing the
controller near-vertical (gimbal lock — known and noted in the
docstrings). Lower the controller, or re-do the gesture calibration to
pin "forward" at your real neutral pose.

**`Aborted (core dumped)` on viz exit.** Pre-existing rclpy/DDS daemon-
thread quirk on interpreter teardown. The viz and cal GUI now
`os._exit(0)` after their cleanup to suppress it; if you ever see it
again, that's the cause, not a bug in the teleop.

**SteamVR can't see the action set.** Re-launch SteamVR after the first
run, or open the controller binding UI once — SteamVR caches the
manifest path. `teleop_actions.json` lives next to the scripts.

## File locations

```
arctos/
├── arctos_arm.py             # arm driver (see README.md)
├── run_vr_teleop.sh          # the launcher
├── cleanup.sh                # universal reset
├── vr_controller_publisher.py
├── vr_teleop_rotation.py
├── vr_hand_control.py
├── vr_visualizer.py
├── vr_calibration_gui.py
├── vr_ros_io.py              # shared lib (topics, AXES, calibration, subscribers)
├── teleop_actions.json       # SteamVR action manifest
├── teleop_binding.json       # SteamVR Knuckles binding
└── vr_calibration.json       # 6-DOF transform (auto-saved by the GUI)
```
