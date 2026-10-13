# Phase 2 IK baseline and kinematics/serial checks (run from the repo root; paths are repo-relative)

| File | Does | Output / writeup |
|---|---|---|
| ik_baseline.cpp | Runs the plain per-cycle IK controller on the real arm and logs setpoint and measured angles. Run twice by editing kDistanceM, kSpeedMps and kOutPath at the top (committed values: 8 cm at 25 cm/s) | results/ik_8cm_10cms.csv and results/ik_8cm_25cms.csv; results/09_ik_baseline.md |
| analyse_baseline.py | Tracking error from an ik_baseline CSV, using MuJoCo FK (independent of the KDL used in the loop) | terminal only; results/09_ik_baseline.md |
| mujoco_fk.py | Prints gripper_link pose from 5 joint angles using MuJoCo FK | terminal only; results/09_fk_crosscheck.md |
| chain_probe.cpp | Prints the KDL chain and gripper_link pose from 5 joint angles (URDF, KDL FK) | terminal only; results/09_fk_crosscheck.md |
| ping_test.cpp | Pings IDs 1-6 through the C++ serial layer (SerialPort, BuildPing, ResponseReader) | terminal only; recorded in results/09_fk_crosscheck.md (Serial-layer verification) |
| read_test.cpp | Reads all six positions through the C++ FeetechBus, compared with calibration offsets | terminal only; recorded in results/09_fk_crosscheck.md (Serial-layer verification) |
