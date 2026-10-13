# Bring-up scripts: what each measures and what it produced (paths relative to this directory)

| Script | Measures / does | Output |
|---|---|---|
| ping.py | Pings servo IDs 1-6 with the vendor SDK (adapted SDK example) | terminal only; see results/01_bus_scan.md |
| sync_read_timeout.py | Serial round-trip timing: sync read and write, all six servos | results/latency_raw.csv; results/02_latency.md |
| sync_read_1joint.py | Same timing test, one servo only | results/latency__sysid_raw.csv; matches the one-joint Sampling table in results/07_servo_characterisation.md |
| joint_mapping.py | Nudges each servo ID to find which physical joint it drives | terminal only; result (ID to joint map) in results/03_calibration.md, which does not name the script |
| find_offsets.py | Tick offsets from hand-moved mechanical stops. Superseded for elbow_flex and wrist_roll by find_offsets_2joints.py | terminal only; method and results (signs, offsets, spans) in results/03_calibration.md, which does not name the script |
| find_offsets_2joints.py | Re-measures elbow_flex and wrist_roll stops (fixes the labelling trap in find_offsets.py) | terminal only; no recorded output found (calibration.yaml says these two joints were not re-measured) |
| goto_zero.py | Drives each joint to 0 rad via the calibration, to compare with MuJoCo | none; zero-pose result in results/03_calibration.md and the TRACKING ERROR note in src/p4_common/config/calibration.yaml |
| goto_half.py | Drives each joint to +0.5 rad to confirm calibration signs against MuJoCo | none; the +0.5 rad tracking result is the TRACKING ERROR note in src/p4_common/config/calibration.yaml |
| gripper.py | Open, close, move or read the gripper (ID 6) | none |
| torque_off.py | Disables torque on one or all joints (the arm drops) | none |
| park.py | Moves to the sysid pose that every sysid run starts from | none; pose is logged with results (07) |
| packets_capture.py | Captures raw bus bytes with the vendor SDK as reference for the C++ packet builder | results/04_packet_capture.md |
| dump_registers.py | Dumps the configuration registers of all six servos | terminal only; table in the Registers section of results/07_servo_characterisation.md |
| motor_mode_check.py | Does motor mode work at all (writes mode and angle-limit registers on ID 1) | terminal only |
| profile_probe.py | Does the STS3215 run an internal motion profile (step sizes vs peak speed) | results/profile_probe_raw.csv |
| profile_analyze.py | Re-analyses profile_probe_raw.csv in position space (replaces its peak-speed check); offline | results/profile_probe.png |
| slope_fit.py | Peak speed vs step size, log-log, to find the regime; offline | results/slope_fit.png |
| torque_sweep.py | Trajectory generator or torque saturation: same step at three torque limits | results/torque_sweep_raw.csv, torque_sweep.png |
| coast_down.py | Damping and friction from a hand flick with torque off (shoulder_pan) | results/coast_down_raw.csv, coast_down.png |
| velocity_sweep.py | Friction and damping from load vs commanded constant speed | results/velocity_sweep_raw.csv, velocity_sweep.png |
| sweep_checks.py | Checks velocity_sweep: load vs current linearity, tracking-error pollution | results/sweep_checks_raw.csv, sweep_checks.png |
| sweep_check_analysis.py | Offline analysis of sweep_checks_raw.csv | terminal only |
| ring_capture.py | Settling ring after a step, averaged, to fit inner P and D | results/ring_capture_raw.csv, ring_capture.png |
| backlash.py | Backlash: approach the same target from both directions | results/backlash_raw.csv, backlash.png |
| torque_calibration.py | Load units to N*m using a known mass at a horizontal arm | terminal only; result (74.3 load units per N*m) in results/07_servo_characterisation.md |
| record_trajectories.py | Records command, position and load for the MJCF fit (fit and validate sets) | results/trajectories/*.csv (results/trajectories_v1/ also exists: older timestamps, not referenced in any writeup) |
| mujoco_viewer.py | Plays one joint through angles in the MuJoCo viewer (sim only, no hardware) | none |
