# ARCTOS joint limits — measurement log

Running record of measured joint travel, feeding `joint_limits.json` and the
`TRAVEL_LIMITS` safety guard in `arctos_arm.py`.

**Last updated:** 2026-08-18

---

## How to read this

Every angle here is **relative to the encoder zero of the current power-on
session.** Encoders reset when motor power is cut, so these numbers are only
portable across reboots once the arm homes to a repeatable zero (Phase 4).
If motor power has been cycled since the date above, re-measure before
trusting anything in this file.

A limit is only real if the **arm** stopped it. Three different things end a
jog, and only one of them is a limit:

| Outcome | Means | Valid limit? |
|---|---|---|
| `*** STOP REACHED` | no completion frame — joint could not follow | **yes**, if mechanical |
| `refused: ... --max-travel ceiling` | jog.py's software ceiling | **no** — raise `--max-travel` |
| stall under gravity / load | torque ran out, not a stop | **no** — retest in a lighter pose |

---

## No published limits exist — measurement is the only source

Checked upstream (2026-08-18). The official Arctos URDF
(`arctos_urdf_description/urdf/arctos.urdf`) declares **joint1 through joint6
as `continuous` with no `<limit>` tag at all**; only the gripper jaws (jaw1,
jaw2) carry bounds of 0.0–0.015 m. `arctos_config/config/joint_limits.yaml`
holds velocity/acceleration entries only — all disabled — and no position
limits.

So the reference design never characterised its range of motion. There is
nothing to look up: these numbers have to be measured on this arm.

---

## Hardware: magnetic limit sensors

This arm has non-contact magnetic limit sensors wired into the MKS boards.
That matters:

- Homing trips the sensor **before** anything touches, unlike stall-based
  homing — no risk to printed parts.
- It provides a **repeatable zero**, which is what makes measured limits
  survive a power cycle. Measure *after* homing, or the numbers die with the
  session.
- Board config already suits it: `EndLimit=1`, `mode=0` (use limit switch).
- Possible re-explanation for the one-off `FD 03` on J5 at +4.3°: with real
  sensors present that may have been a genuine trigger, not a floating pin.

**Hazard:** with `EndLimit=1` the firmware *unlocks the shaft* when homing
touches the switch. On the gravity-loaded J2/J3 the arm can drop. Support it
during homing, or set `EndLimit=0` on those two first.

---

## What actually limits each joint

The motors themselves turn freely — J4 was driven ~360° without resistance,
and the wrist has no stop at all. **The limits come from the 3D-printed
structure**, not from the drivetrain. So every joint has a real limit somewhere
inside the motor's free rotation, and finding it means driving until the
printed parts stop the motion.

Two consequences:

- A joint that keeps turning has not "no limit" — it has a limit you have not
  reached yet. Only `--max-travel` refusals and genuine `STOP REACHED` events
  distinguish the two.
- **Approach every limit slowly.** Running a printed arm into its own hard stop
  at 1600 mA is how parts break. Small steps, low rpm, and let the locked-rotor
  protection catch it rather than driving at it. Back the recorded limit off
  from the contact point (jog.py's `--margin` does this on save; it defaults
  to the file's `margin_deg`).

---

## Status

Confirmed with the builder, 2026-08-18. Only **J2 and J3** needed measuring.
Both have magnetic sensors at *both* ends, but the sensors could not be
verified as working, so the limits were set by jogging to the ends by eye, not
read off sensor trips.

| Joint | Name | Limit sensors | Range | Action |
|---|---|---|---|---|
| J1 | base yaw | none | **continuous, full 360° both ways** | none — leave alone |
| J2 | shoulder | 2 (unused) | **0° to +250.93°** | **MEASURED — enforced** |
| J3 | elbow | 2 (unused) | **0° to +125.5°** | **MEASURED — enforced** |
| J4 | forearm roll | none | **−179° to +179°** (chosen cap) | **enforced** |
| J5/J6 | wrist B/C | none | differential, no hard stop | see wrist note |

This also explains why **limit port remap is enabled on every board**: MKS
needs it to expose a *second* limit input (left → En pin, right → Dir pin).
It was configured deliberately for the two-sensor joints, not left on by
mistake.

### Reading a limit off a sensor

The firmware only reports `FD 03` once it has run its own homing cycle
(EndLimit takes effect after one), so in normal jogging you will not see this.
Use `check_sensors.py` to see whether a sensor works at all, and `home.py` to
home against one -- it polls the inputs itself instead.

When the firmware does stop a move on a sensor, jog.py surfaces it as:

```
=== SENSOR LIMIT: axis 2: LIMIT SENSOR tripped ...
```

That is definitive — record it with `[` or `]`. Contrast with:

```
*** STOPPED (no completion): ...
```

which is **not** a confirmed limit: it may be a stall under gravity, fouling,
or a torque shortfall.

## Gear ratios — verify before trusting any angle

Published Arctos raw ratios: **X 13.5 · Y 150 · Z 150 · A 48 · B 67.82 · C 67.82**
([arctosgui README](https://github.com/Arctos-Robotics/arctosgui)). The repo also
lists a halved set for its own `convert.py` pipeline; that halving is a software
artifact of their stack, not a physical ratio.

| Joint | In our code | Published raw | Assessment |
|---|---|---|---|
| J1 | 24.6 | 13.5 | **matches neither raw nor halved.** If the true value is 13.5, a commanded 10° swings the base ~18° — moving *further* than commanded. |
| J2 | 75.0 | 150 | **uses the halved convention** while every neighbour uses raw. If true is 150, a commanded 10° moves the shoulder only 5°. |
| J3 | 150 | 150 | consistent |
| J4 | 48 | 48 | consistent |
| J5/J6 | 67.82 | 67.82 | consistent |

**Verification method.** Mark the joint, command a known angle, measure the
actual rotation. If commanding **C** degrees produces **P** degrees of physical
motion, the true ratio is:

```
G_true = (C × G_code) / P
```

Note that commanded-vs-encoder agreement does **not** validate a ratio — the
gear ratio appears in both conversions and any error cancels exactly. Only
physical measurement settles it.

---

## Per-joint detail

### J1 — base yaw
Not yet measured; sitting at 0.000°. **Verify the 24.6 ratio first** — this is
the joint where a wrong ratio moves *more* than commanded.

### J2 — shoulder  ✅ MEASURED

| | Value |
|---|---|
| zero (folded, park pose) | `0.0°` |
| enforced max | `+250.93°` |

The max is a **deliberate cap, not the mechanical end** — beyond it certain
J2/J3 combinations collide. A per-joint limit cannot express that constraint,
so the cap is set conservatively enough that no J3 position can reach a
collision. See "Unmodelled constraint" below.

Worth noting: 250.93° is almost exactly twice J3's 125.5°, and the published
ratios give **both** Y and Z as 150:1 while our code has J2 at 75. That is
consistent with J2's ratio being half the true value, making its reported
angles double the real ones. Harmless for enforcement — the same conversion
applies in both directions — but J2's degrees are probably not real-world
degrees.

### J3 — elbow  ✅ MEASURED

Measured 2026-08-18 by jogging to each mechanical end by eye, with the zero
set at the folded (park) end using jog.py's `z` key. **Limit sensors were not
used** — they could not be verified as working.

| | Value |
|---|---|
| reference end (zero, park pose) | `0.0°` |
| far end | `+125.5°` |
| **usable range** | **125.5°** |
| enforced limits | `0.0°` to `+122.5°` |

The far end is backed off 3° rather than the usual 2° because the zero comes
from a hand-placed stop, not a sensor — it is only as repeatable as returning
to that end by eye, roughly a degree or two. The zero end has no margin, so
park.py can still reach the origin.

**To re-establish next session:** jog J3 back to the same reference end, press
`z` then `y` (or, if it was parked there, confirm the park pose in
teleop_gui.py). Until then the measured limits are NOT enforced and the joint
falls back to ±15° travel from the connect pose.

Ratio confirmed against the published value, so its degrees are trustworthy.

### J4 — forearm roll
Ratio confirmed against the published value, so its degrees are trustworthy.

| Direction | Reached | Stopped by |
|---|---|---|
| negative | **−179.718°** (23.96 motor revs, 392,601 counts) | software ceiling |
| positive | **~+180°** (reported from the session, not captured in output) | software ceiling |

**Roughly 360° of travel explored and the arm never resisted in either
direction.** Both ends were cut off by jog.py's `--max-travel`, so no
mechanical stop exists.

**Enforced limits: −179° to +179°** — a chosen cap, not a physical limit. It
exists to stop the wiring run to the hand from winding up, since nothing
mechanical will. Zero is the park pose, like J2 and J3.

Open question — and the reason not to simply mark it continuous: wiring runs
through the forearm to the tactile hand. The *practical* limit is however far
that harness tolerates before winding up, and 24 motor revolutions in one
direction is a great deal of accumulated twist. The mechanism will not stop
you; the cable will. Record the harness limit with `[` and `]` rather than
pressing `n`, unless the hand is wireless.

### J5 / J6 — differential wrist
The two motors drive a differential: neither alone corresponds to a wrist axis,
so per-motor limits are meaningless. Use the coupled axes J7 (both motors same
direction) and J8 (opposite). Confirmed by observation that the two always move
together, and reported to have no hard stop.

J7 reported `STOP REACHED` at **+12.03°**. Unresolved whether that is a genuine
mechanical stop, the wrist fouling on the hand or a cable, or a torque limit.
12° is a suspiciously small range — worth investigating before recording.

---

## Open actions

1. Verify the **J1 (24.6)** and **J2 (75.0)** gear ratios physically — both are
   unmeasured joints whose limits would inherit any ratio error.
2. Confirm the cable run to the hand tolerates J4's chosen ±179° cap —
   ~360° of free rotation is confirmed, so nothing mechanical protects it.
3. Diagnose the J7 stop at +12.03° before recording any wrist limit.
4. Get the J2/J3 sensors working (`check_sensors.py`), so `home.py` can give a
   repeatable zero instead of the by-eye one. home.py polls the sensors
   itself rather than using firmware homing, because with `EndLimit=1` the
   firmware **unlocks the shaft** when it touches the switch — a drop hazard
   on the gravity-loaded J2/J3.
