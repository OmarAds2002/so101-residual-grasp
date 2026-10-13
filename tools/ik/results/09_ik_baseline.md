# 09 — IK baseline: the number the residual has to beat

**Date:** 2026-10-12
**Phase:** 2 — baseline IK controller
**Code:** `tools/ik/ik_baseline.cpp`, `tools/ik/analyse_baseline.py`
**Data:** `tools/ik/results/ik_8cm_10cms.csv`, `ik_8cm_25cms.csv`

---

## The number

Gripper-space tracking error, plain per-cycle IK, no policy.

| | 10 cm/s | 25 cm/s |
|---|---|---|
| **p99 while moving** | **12.6 mm** | **44.3 mm** |
| p50 while moving | 10.8 | 33.5 |
| **steady state after settling** | **~5.5 mm** | **~5.4 mm** |
| non-converged cycles | 0 of 261 | 0 of 465 |

Two numbers, because they answer different questions.

**Dynamic error** is how far behind the arm runs while the motion is in
progress. It scales with speed — 2.5× the speed gives 3.5× the error — which
is what lag looks like: the arm trails by a roughly fixed *time*, so a faster
arm trails by more *distance*.

**Steady-state error** is where the arm ends up once it has stopped. It does
not depend on speed, and it is the number that decides whether a grasp lands.

For Phase 4, the dynamic number is the one with room in it. A learned residual
that anticipates the lag has 12 mm to work with at approach speed. The 5.5 mm
floor is a harder target and may be structural.

---

## Setup

- **Rate:** 200 Hz, `sleep_until` on a steady clock
- **Motion:** 8 cm along base-frame +x, straight line in gripper space,
  smoothstep velocity profile
- **Solver:** KDL `ChainIkSolverPos_LMA`, chain `base_link` → `gripper_link`,
  five joints
- **Dropped freedom:** yaw weighted to zero (see below)
- **Seed:** the measured joint angles from the same cycle
- **On non-convergence:** hold the last good command; a run of ten
  consecutive failures is a fault and stops the run
- **Settle:** the loop keeps commanding the goal for 2 s after the plan's
  clock runs out

Error is computed offline: the measured joint angles from each row are run
through MuJoCo's forward kinematics and compared against that row's setpoint.
MuJoCo, not KDL — the control loop already used KDL, so using KDL again would
compare the thing against itself. The two were cross-checked on three poses in
`09_fk_crosscheck.md`.

---

## Why 5 joints, not 6

The SO-101 has six servos but only five position the gripper; the sixth opens
and closes the jaw. Five joints cannot reach an arbitrary six-number pose, so
one task-space freedom has to be given up.

**Given up: rotation about the gripper's approach axis.** For a top-down grasp
that is spin about the vertical, and a parallel jaw closing on a cube does not
care about it. In the solver this is the weight vector's sixth entry set to
zero.

What it costs: a long object that must be gripped across its length needs a
specific wrist angle, and that may not be reachable at every table position.
Foam cubes are fine. Revisit if the demo ever uses a pen.

This also only holds while the approach is straight down. An angled approach
breaks the coincidence between "spin about the approach axis" and "rotation
about base z", and the fixed weight vector stops being correct.

---

## What the data shows

**The error is lag, not offset.** At t = 0 the error is exactly 0.00 mm. It
then accumulates cycle by cycle — 0.51, 1.01, 1.51, 2.01 mm — while the
measured position does not move at all. The setpoint is advancing and the arm
has not started yet.

That is the ~65 ms dead time measured in Phase 1, arriving again from a
completely different direction. At 10 cm/s, 65 ms of lag is 6.5 mm; measured
p50 is 10.8 mm, the remainder being the servos' own tracking once they do
start.

This ruled out a hypothesis worth recording. The S03 handover flagged
`elbow_flex`'s offset as eyeballed rather than derived — a 75-tick shift,
which at this reach would be over 20 mm of gripper position — and predicted it
would surface in Phase 2 as a small constant tracking error. It did not. A
constant offset would show at t = 0, and t = 0 is clean. Whatever that 75 ticks
is, it is not visible here.

**The steady state is real.** After the motion ends the error falls to ~5.5 mm
and stops. The last rows are identical to two decimal places: commanded
x = 0.3023, measured x = 0.3020. Nothing is moving and the arm is 5.5 mm from
where it was asked to be. Confirmed flat at both 0.5 s and 2 s of settle.

Candidates, none yet tested: servo deadband (Phase 1 measured 2.6 ticks of
backlash plus deadband), rounding to integer ticks at the packing step, or a
small model mismatch. Worth resolving before Phase 4, since it is the floor the
residual would have to beat.

**IK is not the bottleneck.** Cycle work — read, solve, write — came in at
1085–1130 µs mean, 1439 µs max, in a 5000 µs period. Phase 1 measured the bus
alone at p99 1.25 ms, so IK costs a few hundred microseconds at most. The
500-iteration default was never a problem.

**Zero non-convergent cycles** across 726 cycles in the two runs. The hold-last
path did fire in an earlier run with an unreachable goal and behaved correctly:
377 cycles converged, then the trajectory crossed the workspace boundary, the
failure counter hit its limit, and the run stopped with the arm holding its
last good command.

---

## Caveats

**This is not the production loop.** It is a standalone program with
`sleep_until`, not a `ros2_control` controller, and FIFO real-time scheduling
is still denied on this machine. Cycle times were measured and are reported
above; they are tight, but this is not a real-time claim. Phase 3 builds the
real stack.

**One axis, one distance.** Both runs move 8 cm along +x from near the same
start pose. Multi-axis motion couples joints differently and may not behave the
same. Worth adding before trusting this as *the* baseline.

**Reach matters.** A first attempt at 15 cm put the goal at x = 0.372 m, well
outside the workspace, and IK correctly failed. The arm is roughly 0.3 m. The
workspace boundary has not been mapped, and Phase 4 will need that to generate
reachable targets.

---

## Reproduce

```bash
# build (outside conda)
g++ -std=c++20 tools/ik/ik_baseline.cpp -o /tmp/ik_baseline \
  -Isrc/p4_common/include -Isrc/p4_control/include -Isrc/p4_hardware/include \
  -DURDF_PATH='"src/p4_common/urdf/so101_new_calib.urdf"' \
  -DCALIBRATION_PATH='"src/p4_common/config/calibration.yaml"' \
  $(pkg-config --cflags --libs orocos-kdl) \
  -Lbuild/p4_common -lp4_common -Lbuild/p4_control -lp4_control \
  -Lbuild/p4_hardware -lp4_hardware -lyaml-cpp \
  -Wl,-rpath,$PWD/build/p4_common:$PWD/build/p4_control:$PWD/build/p4_hardware

mkdir -p tools/ik/results
/tmp/ik_baseline          # distance, speed and output path are constants at the top

# analyse (inside conda — mujoco lives there)
conda activate base
python tools/ik/analyse_baseline.py tools/ik/results/ik_8cm_10cms.csv
```

Support the arm before running. Servos hold their last goal at full torque when
software dies; killing the program does not stop the arm. Power is the stop.