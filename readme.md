# so101-residual-grasp

A learned residual policy on top of a classical IK controller, driving a real
6-DOF SO-101 arm.

**Status: phase 2 of 8. Work in progress.**

---

## The goal

A camera sees objects on a table. A behaviour tree decides what to do with
them. A learned residual policy runs the grasp. C++ drives the arm at 200 Hz.
When a grasp fails, the tree notices and retries.

The arm is an SO-101: six Feetech STS3215 serial-bus servos on a single
half-duplex line. Five of them position the gripper; the sixth opens and
closes the jaw.

---

## Where it is now

| Phase | | Status |
|---|---|---|
| 0 | Repo, CI, URDF→MJCF, hardware abstraction stub | done |
| 1 | Bring-up, latency, system identification | done |
| 2 | Baseline IK controller, tracking-error baseline | done |
| 3 | C++ stack — driver, ros2_control plugin, RT loop, safety FSM, ONNX | next |
| 4 | Residual RL policy | |
| 5 | Deployment, closing the sim-to-real gap | |
| 6 | Perception — hand-eye calibration, ArUco | |
| 7 | Task layer — BT.CPP v4 with failure recovery | |
| 8 | Video, writeup | |

### What works today

- **The arm is characterised.** Serial round-trip measured over 20,000
  transactions, servo registers read, backlash and deadband measured,
  calibration derived and verified against the simulator.
- **The simulator matches the arm.** A MuJoCo model fitted against recorded
  real trajectories tracks held-out runs to 7–15 ticks RMS. That is the model
  the policy will be trained in.
- **A baseline controller runs on hardware.** Per-cycle inverse kinematics at
  200 Hz, straight-line motion in gripper space, driving the real arm through
  the project's own serial driver.

### The number

Gripper-space tracking error with plain IK and no policy — the number the
learned residual has to beat.

| | 10 cm/s | 25 cm/s |
|---|---|---|
| p99 while moving | 12.6 mm | 44.3 mm |
| steady state after settling | ~5.5 mm | ~5.4 mm |

Zero non-convergent IK cycles across both runs. Full conditions and caveats in
[`tools/ik/results/09_ik_baseline.md`](tools/ik/results/09_ik_baseline.md).

---

## Layout

```
src/p4_common/     joint order, calibration, tick<->radian conversion, URDF, MJCF
src/p4_hardware/   Feetech protocol, serial port, ros2_control plugin
src/p4_control/    IK solver, trajectory generation
tools/bringup/     hardware measurement scripts, with their raw output
tools/sysid/       the MuJoCo parameter fit
tools/ik/          kinematics and control-loop probes
```

Each `tools/` directory has a README listing what each script measures and
where its output went. The `results/` folders hold writeups and raw CSVs:
measurements of this specific arm, on specific days, which cannot be
regenerated.

---

## Build

Requires ROS 2 Jazzy.

```bash
colcon build
colcon test
```

Build C++ outside conda. Conda ships `libyaml-cpp.so.0.9` and Jazzy uses
`0.8`; building with conda active bakes the wrong one in and it fails at load
rather than at build. The Python tools need conda, since MuJoCo and the
Feetech SDK live there.

---

## License

Apache-2.0.