#!/usr/bin/env python3
"""Compute the Phase 2 baseline tracking error from an ik_baseline CSV.

Runs the measured joint angles through forward kinematics and compares the
resulting gripper pose against the setpoint that was commanded on the same
cycle. The difference is the tracking error: how far behind the arm was.

MuJoCo does the FK here, not KDL. The C++ loop already used KDL, so using
KDL again would compare a thing against itself. The two were cross-checked
on three poses (tools/ik/results/09_fk_crosscheck.md), so MuJoCo is an
independent path to the same quantity.

Needs conda: conda activate base
Usage: python tools/ik/analyse_baseline.py tools/ik/results/<run>.csv
"""

import csv
import sys

import mujoco
import numpy as np

MJCF_PATH = "src/p4_common/mjcf/so101.xml"

# The IK chain, base to tip. The gripper's jaw is not part of it.
IK_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]

TIP_BODY = "gripper_link"

# Rows whose setpoint has stopped moving: the arm is settling onto a
# stationary goal and the error decays to nothing. Including them drags
# every percentile down and flatters the baseline.
MOVING_EPS_M = 1e-6


def load_rows(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 1

    path = sys.argv[1]
    rows = load_rows(path)
    if not rows:
        print(f"{path}: no rows")
        return 1

    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)

    # Index qpos by joint name. The MJCF's internal order need not match
    # the canonical joint_order, and assuming it would be the joint
    # ordering trap.
    qpos_adr = [model.joint(name).qposadr[0] for name in IK_JOINTS]

    errors_mm = []
    moving_errors_mm = []
    cycle_us = []
    non_converged = 0

    prev_sp = None

    for row in rows:
        setpoint = np.array(
            [float(row["sp_x"]), float(row["sp_y"]), float(row["sp_z"])]
        )

        for i, adr in enumerate(qpos_adr):
            data.qpos[adr] = float(row[f"meas_{i}"])
        mujoco.mj_forward(model, data)

        reached = np.array(data.body(TIP_BODY).xpos)
        err_mm = float(np.linalg.norm(reached - setpoint)) * 1000.0

        errors_mm.append(err_mm)
        cycle_us.append(float(row["cycle_us"]))
        if int(row["converged"]) == 0:
            non_converged += 1

        if prev_sp is not None and np.linalg.norm(setpoint - prev_sp) > MOVING_EPS_M:
            moving_errors_mm.append(err_mm)
        prev_sp = setpoint

    errors_mm = np.array(errors_mm)
    moving_errors_mm = np.array(moving_errors_mm)
    cycle_us = np.array(cycle_us)

    def report(label, a):
        if a.size == 0:
            print(f"{label}: no samples")
            return
        print(
            f"{label:16s} n={a.size:5d}  "
            f"p50={np.percentile(a, 50):7.2f}  "
            f"p90={np.percentile(a, 90):7.2f}  "
            f"p99={np.percentile(a, 99):7.2f}  "
            f"max={a.max():7.2f}"
        )

    print(f"\n{path}")
    print(f"rows                {len(rows)}")
    print(
        f"non-converged       {non_converged} "
        f"({100.0 * non_converged / len(rows):.1f}%)"
    )
    print(
        f"cycle_us            mean={cycle_us.mean():.0f}  "
        f"p99={np.percentile(cycle_us, 99):.0f}  max={cycle_us.max():.0f}"
    )
    print("\ngripper-space tracking error, mm")
    report("whole run", errors_mm)
    report("while moving", moving_errors_mm)
    print()

    return 0


if __name__ == "__main__":
    sys.exit(main())
