# ARCTOS Arm — `arctos_arm.py`

Joints **1–6**. Angles in **degrees**, speeds in **RPM (0–3000)**.

```python
with ArctosArm() as arm:
    arm.move_joint(1, -15)
    arm.set_joint_angles(0, 10, -20, 0, 0, 0)
```

Context manager opens the SLCAN bus, syncs encoders, and emergency-stops on exit.

## Methods

| | |
|---|---|
| `move_joint(j, deg, rpm=128, acc=5, *, wait=True)` | Relative move. Returns `Future`. |
| `set_joint_angles(j1..j6, ...)` | Absolute targets, all six in parallel. |
| `read_encoder(j)` / `sync_all_encoders()` | Fresh angle(s); writes `current_angles`. |
| `query_status(j)` | `STATUS_MOTOR_*` constant. |
| `set_home_params(j, home_dir, home_speed, end_limit=False, mode=HOME_MODE_SWITCH)` | 0x90 config. |
| `home_joint(j)` / `home_all(order=None)` | Run homing. Sequential — never parallel. |
| `zero_here(j)` | Mark current position as zero, no motion. |
| `emergency_stop(j)` / `emergency_stop_all()` | 0xF7. |

## Gotchas

- **Home on every boot** — encoders reset on power-on.
- **J5/J6 are differentially coupled** — `home_all()` gives wrong zeros; differential kinematics not implemented.
- **CanRSP must be enabled** in firmware — every TX awaits a response.
- **`current_angles`** is only written by encoder reads, never by moves. Call `read_encoder` for guaranteed-fresh.

Protocol reference: `MKS SERVO42&57D_CAN User Manual V1.0.6.pdf`.
