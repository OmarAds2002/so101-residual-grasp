# 08 — MJCF fit (Session 06)

Fitting kp, damping and frictionloss in `src/p4_common/mjcf/so101.xml`
against the trajectories recorded in S05.

Script: `tools/sysid/fit_mjcf.py`. Diagnosis helpers: `plot_one.py`,
`sim_only.py`.

## The bug that had to be found first

The fit's first run gave RMS errors of 790 ticks (69 deg). Not a
parameter problem — the model itself was broken.

`sim_only.py`: command all actuators to 0, gravity on, step 2 s.
shoulder_pan ended at 1.224 rad moving at 8 rad/s with the actuator
saturated at -10 N*m pulling back. Same endpoint with gravity off, and
with kp raised tenfold. **Something drove it there that no controller
setting could resist.**

Cause: **12 contacts at the zero pose, every one penetrating 2.6 cm.**
All of them `world` against `shoulder_link`.

The URDF conversion merged the fixed base link into `world`. On the real
arm the shoulder housing and the base overlap — they are bolted
together. ROS never noticed because nothing in ROS runs contact physics,
and P4's own MuJoCo use until now was `mj_forward` for posing, which
computes positions without contacts. The first thing to actually step
the physics found it.

Fix: `<geom contype="0" conaffinity="0"/>` in the defaults. Meshes still
render, masses and inertias still count, nothing collides. Correct for
Phase 1 — the real arm had no link contact during these recordings, and
mechanical limits are already joint ranges. **Phase 4 must turn
collision back on selectively for the jaws and objects.**

Masses were fine (79-104 g) and `qfrc_bias` on pan was ~0, so neither
inertia nor gravity was involved.

## Cost function

Per run, per moving joint:

w_p*(pos_err/pos_sd)^2 + w_v*(vel_err/vel_sd)^2 + w_l*(load_err/load_sd)^2


all weights 1. Each term divided by that quantity's own spread, so none
dominates through its units. Load converted to N*m with the S05 scale
factor, 74.3 units per N*m.

**Velocity error is in there deliberately.** Position error reacts
slowly — a small sustained velocity mismatch takes a long time to appear
as a position gap. Velocity error is largest exactly at reversals and
transients, which is where friction and kp show themselves. It weights
the informative parts without a threshold.

**Runs are averaged, not pooled.** Pooled, the 2400-sample runs would
outvote the 320-sample `fit_pan_ramp_fast`, which is the only run
probing the top of the speed band.

**Joints fitted separately.** `fit_pan_*` moves only pan, `fit_lift_*`
only lift, so each gives its own parameters independently.

## Dead time: 65 ms stands

Recordings are shifted rather than fitting dead time as a fourth
parameter. Swept 45 / 55 / 65 / 75 / 85 ms. Pan's cost: 1.08, 1.12,
1.16, 1.46, 1.26. Flat, no minimum. **The shift is not absorbing a
timing error.**

Residual: the validation overlay shows the sim very slightly ahead in
phase after ~8 s, consistent with dead time being a little longer under
continuous commands than the ~65 ms measured on single idle steps. Too
small for the sweep to resolve. Would appear in Phase 5 as the policy
leading slightly. Noted, not chased.

## Results

Bounds enforced during the search, not checked after: kp 5..2000,
damping < STALL/vel_p90, friction < gravity torque. Three starting
points per joint.

### shoulder_lift — kp identified

| start | cost | kp | damping | friction |
|---|---|---|---|---|
| 0 | 0.9511 | 47.8 | 1.5409 | 0.6551 |
| 1 | 0.9596 | 45.1 | 1.0011 | 0.9090 |
| 2 | 0.9659 | 45.9 | 0.8184 | 1.0000 |

**kp agrees to 6% across all three.** Taken as identified.

### shoulder_pan — kp NOT identified

| start | cost | kp | damping | friction |
|---|---|---|---|---|
| 0 | 1.1645 | 161.5 | 2.1438 | 0.4805 |
| 1 | 1.4319 | 1526.4 | 1.7083 | 1.0000 |
| 2 | 1.5650 | 2000.0 | 0.0227 | 0.1359 |

kp spans a factor of twelve and two starts hit the upper bound. Start 0
is genuinely better on cost, but multiple basins mean it cannot be
confirmed as the global best.

Pan's damping sits at 1.64 N*m at p90 speed — 86% of stall. Inside the
bound, but at the edge of plausible.

### Why lift is identified and pan is not

Pan is gravity-neutral: vertical axis, no weight to hold. The controller
fights only friction and damping, which is exactly the inseparable pair.

Lift carries the arm's weight. Gravity is a known torque that varies
with angle, and droop at a given angle depends on **kp alone**. That
independent handle is what makes kp identifiable.

**A known external force makes a parameter identifiable.** Same
principle as the phone torque calibration.

## The ridge — damping and friction are not separated

On lift, costs 0.9511 / 0.9596 / 0.9659 — within 1.5% — while damping
varies 1.54 -> 0.82 and friction moves 0.66 -> 1.00 in lockstep. As one
rises the other falls.

**The data pins their combined effect, not the split.** This is the
identifiability problem that opened Phase 1, arriving where it was
predicted to.

| quantity | value | confidence |
|---|---|---|
| kp | ~46 N*m/rad (lift), 161 (pan) | good / weak |
| damping + friction combined | reproduces motion | good |
| damping alone | 0.8 - 2.1 | **not separated** |
| friction alone | 0.48 - 1.0 | **not separated** |

**Accepted for now.** The sim reproduces real motion across the whole
operating band, which is what the residual policy experiences. Revisit
if Phase 5 shows a gap pointing here.

To break the ridge later: record very slow motion, where friction
dominates and damping nearly vanishes. ~20 min on the bench.

## Validation — parameters generalise

Never fitted against:

| run | cost | RMS |
|---|---|---|
| val_pan_chirp | 0.8876 | 15.0 ticks |
| val_multi | 1.1245 | 9.0 ticks |
| val_elbow_sine | 0.5703 | 7.3 ticks |

Validation costs are close to the fit costs (1.16 and 0.95), so the
parameters describe the robot rather than the recordings.

**val_elbow_sine is the strongest result**: 7.3 ticks using pan's
parameters on a joint that was never fitted for.

`val_pan_chirp` overlay: sim tracks real through 15 s and 14
oscillations of rising frequency and falling amplitude, with no drift.

## What went into the MJCF

```xml
<option timestep="0.001"/>          <!-- 200 Hz control -> decimation 5 -->
<default>
  <joint damping="1.54" frictionloss="0.655" armature="0.01"/>
  <position kp="47.8" kv="0" forcerange="-1.9 1.9"/>
  <geom contype="0" conaffinity="0"/>
</default>
<!-- shoulder_pan overrides: damping 2.14, frictionloss 0.48, kp 161.5 -->
```

Three changes beyond the fitted values:

- **forcerange ±1.9, was ±10.** The sim could produce five times the
  real torque. A policy trained against that learns moves the arm cannot
  make — a Phase 4 trap.
- **kv = 0, was 1.** The actuator's own damping duplicates the joint's,
  making them inseparable. The fit assumed kv played no part; this makes
  that true.
- **timestep 0.001, was 0.002.** Clears the S04 owed item: 0.002 with
  200 Hz control gave decimation 2.5, not a whole number.

## Still unverified

- **armature 0.01 is still a placeholder.** Never fitted.
- **Four of six joints carry lift's values untested.** Only pan and lift
  were fitted; elbow was validated but never fitted; wrist_flex,
  wrist_roll and gripper have no measurement at all.
- The damping/friction split, above.
- Lever arm 0.35 m by hand, ±2 cm; the load-to-torque scale and
  everything derived from it scale inversely with it.