# ARCTOS Arm — `arctos_arm.py`

Joints **1–6**. Angles in **degrees**, speeds in **RPM (0–3000)**.

## Setup on a new machine

```bash
cd arctos && ./setup.sh
```

Installs what is missing (`python3-can`, `python3-serial`, `python3-tk` via
apt), adds you to `dialout` for serial access, warns about a stray `slcand`,
then runs `selftest.py` — an offline check, no arm needed. Safe to re-run.
Targets Ubuntu 24.04; tools run on the system `python3`. The CANable is found
automatically (`ARCTOS_COM_PORT` overrides).

`joint_limits.json` belongs to the **arm**, not the computer — it is
committed, so every machine enforces the same measured limits.

## Usage

```python
with ArctosArm() as arm:
    arm.move_joint(1, -15)
    arm.set_joint_angles(0, 10, -20, 0, 0, 0)
```

Context manager opens the SLCAN bus, syncs encoders, and emergency-stops on exit.

## Methods

| | |
|---|---|
| `move_joint(j, deg, rpm=128, acc=5, *, wait=True, on_limit="clamp")` | Relative move, limit-checked first. Returns `Future`. |
| `set_joint_angles(j1..j6, ..., on_limit="clamp", sync=True)` | Absolute targets, all six in parallel; `sync` scales rpms so all finish together. |
| `check_pose({j: deg})` | Joints whose targets fall outside their limits. |
| `confirm_parked()` | Operator asserts the arm is at its park pose → measured limits enforced. |
| `read_io(j)` | Limit-sensor input bits (0x34). |
| `read_encoder(j)` / `sync_all_encoders()` | Fresh angle(s); writes `current_angles`. |
| `query_status(j)` | `STATUS_MOTOR_*` constant. |
| `set_home_params(j, home_dir, home_speed, end_limit=False, mode=HOME_MODE_SWITCH)` | 0x90 config. |
| `home_joint(j)` / `home_all(order=None)` | Run homing. Sequential — never parallel. |
| `zero_here(j)` | Mark current position as zero, no motion. |
| `emergency_stop(j)` / `emergency_stop_all()` | 0xF7. |

## Soft limits

Every move is checked **before** it is sent. `on_limit="clamp"` shortens it
to the limit; `"raise"` refuses with `SoftLimitExceeded`. Which window applies:

- **Measured** (`joint_limits.json`) — once the joint has a known origin this
  session: zeroed (`home.py`, jog.py `z`) or `confirm_parked()`.
- **Travel-from-start** (`TRAVEL_LIMITS`, ±15–30°) — otherwise. Encoders read
  0 at power-on wherever the arm is, so absolute limits mean nothing until an
  origin is established.

A joint outside its window may move back toward it, never further out.
Measurement log and per-joint notes: [JOINT_LIMITS.md](JOINT_LIMITS.md).

## Tools

| | |
|---|---|
| `teleop_gui.py` | Nudge buttons, live positions, limit windows, e-stop, park confirm. |
| `jog.py` | Keyboard jog; record limits (`[` `]`, `s`), zero (`z`), save poses (`S`). |
| `home.py` | Establish a joint's origin — against a sensor, or `--manual` in place. |
| `park.py` | Drive to the park pose (all zeros) before power-down. |
| `poses.py` | List / show / delete poses saved from jog.py (`poses.json`). |
| `demo_move.py` | Coordinated whole-arm demo, offsets from the start pose. |
| `read_pos.py` | Read positions without connect()/e-stop side effects. |
| `check_sensors.py` | Watch limit-sensor inputs live. Moves nothing. |
| `read_params.py` / `set_params.py` | Read / write board EEPROM (current, stall protection). |
| `motion_debug.py` | Why a joint answers but will not move. |
| `selftest.py` | Offline checks — run before committing changes to the arm tools. |

## Gotchas

- **Home on every boot** — encoders reset on power-on. Or park before power-down and confirm the park pose next session.
- **`slcand` holding a port** (`stty -F /dev/ttyACM1` shows `line = 17`) — the arm cannot connect. `sudo killall slcand`, replug.
- **J5/J6 are differentially coupled** — `home_all()` gives wrong zeros; differential kinematics not implemented.
- **CanRSP must be enabled** in firmware — every TX awaits a response.
- **`current_angles`** is only written by encoder reads, never by moves. Call `read_encoder` for guaranteed-fresh.

Protocol reference: `MKS SERVO42&57D_CAN User Manual V1.0.6.pdf`.
