# 07 — Servo characterisation (Session 05)

Joint: shoulder_pan (ID 1) unless stated.
Pose: sysid park pose — `tools/bringup/park.py`. All results depend on it.
Raw: profile_probe_raw.csv, torque_sweep_raw.csv, coast_down_raw.csv,
     velocity_sweep_raw.csv, sweep_checks_raw.csv, ring_capture_raw.csv,
     backlash_raw.csv

## Sampling — one joint, read-only

| min | p50 | p90 | p99 | p99.9 | max |
|---|---|---|---|---|---|
| 0.250 | 0.272 | 0.304 | 0.351 | 0.368 | 3.257 |

p99 0.351 ms -> ~2845 Hz. 3.5x the 800 Hz control figure, which included
a write and six servos. **A measured ceiling carries the conditions it
was measured under.**

max is a lone outlier ~10x p50. Log per-sample timestamps; never assume
a fixed interval.

## Registers — read from the arm, identical on all six

| Addr | Name | Value |
|---|---|---|
| 16 | max_torque_limit | 1000 |
| 21 | P coefficient | 32 |
| 22 | D coefficient | 32 |
| 23 | I coefficient | 0 |
| 24 | min_startup_force | 16 |
| 26/27 | CW/CCW dead zone | 1 |
| 33 | mode | 0 (position) |
| 41 | acceleration | 0 |
| 46 | goal_speed | 0 |
| 48 | torque_limit | 1000 |

Factory defaults, untouched. One parameter set for the sim, not six.
Community reference claims min_startup_force default 0; the arm says 16.

### CORRECTION to S03
S03 recorded "0 means maximum". True for goal_speed (0 = unlimited).
**False for acceleration** — there 0 = instant, no ramp. No trajectory
generator is configured on any joint.

## Mechanism: torque saturation

Steps 40–400 ticks: peak speed scales with the square root of distance.
Log-log slope **0.55**, no bend, no linear PD regime above the deadband.

Torque limit (reg 48) at a fixed 300-tick step:

| limit | peak ticks/s | travelled | short by |
|---|---|---|---|
| 1000 | 1321.9 | 299.0 | 1.0 |
| 500 | 1315.1 | 299.0 | 1.0 |
| 250 | 879.9 | 298.0 | 2.0 |

250 is slower; a software planner cannot know the torque limit.
**Saturation confirmed.**

Open, not chased: 1000 and 500 agree to 0.5%, so demand never exceeds
~50% here. Firmware duty cap or back-EMF. Irrelevant at 200 Hz where
per-cycle steps are a few ticks and saturation never occurs.

## Dead time: ~65 ms

Flat before motion on every step, independent of step size. Constant, so
firmware loop rate plus position filtering, not deadband. **13 control
periods at 200 Hz.** This is what the delay-injecting stub reproduces and
what the residual policy must tolerate.

## Table drag — contamination found and removed

Original pose rested outboard links on the table. Shortfall 7–8 ticks.
In the park pose, clear: **1–2 ticks**. All friction numbers from before
the park pose are void. Every sysid run now records its pose and what was
touching what.

## Friction and damping — the result

Constant-velocity sweep. Position mode only; goal position walked forward
at a fixed rate from our own 200 Hz loop, the same code path the real
controller uses. Six speeds, both directions.

|load| = 0.289 * speed_ticks_per_s + 22.9 (run 2)
|load| = 0.295 * speed_ticks_per_s + 20.9 (run 1)


- intercept ~21 load units = **Coulomb friction**
- slope ~0.29 load units per tick/s = **viscous damping**
- forward and reverse overlay -> **symmetric**
- **two independent runs agree to 2%** — this is the confirmation

### A wrong turn, recorded so it isn't repeated
Lag (target − measured) rises steeply with speed: 7, 13, 33, 76, 161, 308
ticks. Claude argued load was tracking lag, not speed, and that the fit
was an artefact of the servo chasing its own error.

Tested on the existing CSV, residuals of both fits:

| fit | slope | intercept | max residual |
|---|---|---|---|
| load vs speed | 0.2894 | 22.89 | **4.17** |
| load vs lag | 1.4067 | 80.47 | 36.62 |

Speed wins by ~9x and has the sensible intercept. **The claim was wrong.**
Lag and speed both rise, so either could explain load; only the residuals
separate them. Reasoning from "this could explain it" to "this does
explain it" skips the test.

The lag is real but is a consequence: the servo is saturated during the
sweep, so load reflects the speed it settles at against friction and
damping. Which is exactly what makes the measurement valid — constant
speed, no acceleration, torque supplied = torque resisting.

### load is a valid torque proxy
load/current ratio across speeds: 56, 80, 68, 75, 68, 61. No trend. The
low-speed scatter is current quantisation (reads 1–2 units). Load tracks
current, current is proportional to torque.

## Backlash + deadband: 2.6 ticks

Same target approached from both directions, 8 repeats, 5 positions.

| target | from below | from above | gap | spread |
|---|---|---|---|---|
| 1696 | 1695.00 | 1697.00 | 2.00 | 0.61 |
| 1896 | 1896.50 | 1898.00 | 1.50 | 0.66 |
| 2096 | 2096.00 | 2100.00 | 4.00 | 0.33 |
| 2296 | 2296.00 | 2300.00 | 4.00 | 0.33 |
| 2496 | 2497.00 | 2498.50 | 1.50 | 1.14 |

Mean **2.6 ticks (0.23°)**. Gaps are 2–6x the spread, so real. Sign is
consistent: from above always settles high, from below always low.
Varies with position (4.0 at centre, 1.5 at the extremes) — normal, gear
engagement changes around the revolution.

Includes the 1-tick deadband each way as well as gear slack. What it
measures is total direction-dependent error, which is what the sim needs.

## Settling: stick-slip hunting, not PD ringing

10 repeats averaged per step, aligned on the command.

Step 300 settles by **undershooting**, then oscillating BELOW the target:
−2, −8, +1, −5, 0, −3, settle. Four cycles, period ~150 ms, amplitude
decaying 8 -> 5 -> 3 ticks. Same at 500 and 800.

Never crosses above the target, so not a PD overshoot. Consistent with
stick-slip: joint stops short, friction holds it, demand builds past the
breakaway force, it lurches and sticks. Fits min_startup_force = 16 and
I = 0.

**P and D cannot be fitted from the settling ring.** The frequency is
friction against rising demand, not P against inertia. Fit kp, damping
and frictionloss together against recorded trajectories instead.

## Dead ends — do not retry

### shoulder_lift free-swing
Torque off, it falls until it hits the table or its own structure.
Collision ends the motion, not physics. Not fixable by repositioning.

### shoulder_pan hand-flick coast-down
Four flicks gave decelerations 4,700–7,900 ticks/s², and the fastest
flick decayed slowest — consistent with neither friction nor damping.
A hand flick is an uncontrolled input: release timing unknown, transient
sits where the measurement is, no repeatability. Superseded by the sweep.

### Motor mode (reg 33 = 1)
Needs EEPROM writes to regs 9, 11, 33. The write to reg 11 fails with
COMM_RX_TIMEOUT (-6) every time and the bus desyncs — subsequent reads
return garbage from the wrong byte offsets (mode 252, min_angle 64001,
position 388 with the arm at 2096). Reproduced across a power cycle.
Recovery is a power cycle; nothing is damaged, the writes fail rather
than landing wrong.

Same class as S04's "a read is not a packet". The vendor Python SDK has
no reassembly; one timeout loses the stream.

**Not needed.** Position mode gives the same measurement with no EEPROM
write, and exercises the real control path.

## Still owed before these go in the MJCF

- **Load-to-torque calibration.** load is an uncalibrated 0–1000 reading.
  MuJoCo wants N·m. Hang a known mass at a known lever arm, hold
  position, read load.
- **Fit and validate.** kp, damping, frictionloss tuned together against
  recorded real trajectories, then checked against a trajectory never
  fitted to.

## Load-to-torque calibration

Arm straight and horizontal, shoulder_lift at 2000, elbow 1875,
wrist_flex 2020, wrist_roll 2074. Phone (Galaxy S23 FE, 209 g, Samsung
spec) gripped at a measured lever of 0.35 m -> known torque 0.7176 N*m.
Lever measured by hand, +/- ~2 cm. All N*m figures scale inversely
with it.

The arm's own weight is identical with and without the phone, so it
cancels in the difference. No link masses needed.

| trial | empty | with phone | delta |
|---|---|---|---|
| 0 | 80 | 128 | 48 |
| 1 | 72 | 128 | 56 |
| 2 | 72 | 128 | 56 |

mean delta 53.3, sd 3.77 (one quantisation step)
Every individual reading had sd 0.00 -- the load register is steady
while holding.

**SCALE: 74.3 load units per N*m.  1 unit = 0.0135 N*m.**

## The velocity-sweep slope is NOT viscous damping

Converting the sweep fit gives damping = 2.53 N*m*s/rad. At 1 rad/s that
demands 2.5 N*m; the STS3215 stalls near 1.9 N*m. The arm could not move
at 1 rad/s, and it moved far faster today. **The number is impossible.**

Cause: the sweep ran the servo SATURATED -- lag reached 308 ticks at the
top speed. Under saturation the load register reports the motor at full
effort, which rises with speed for reasons unrelated to viscous friction.
The slope describes the saturated regime, not a damping coefficient.

The intercept (0.31 N*m, ~16% of stall) is plausible for a 345:1
gearbox and is kept as a provisional Coulomb friction figure.

**A unit conversion is a sanity check.** The residual test could not
catch this; dividing by the scale factor did, in one line.