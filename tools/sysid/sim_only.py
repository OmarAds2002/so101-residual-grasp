#!/usr/bin/env python3
"""Does the sim behave at all, with no recording involved?

Three tests, each simpler than the last failure:
  1. hold every ctrl at 0. The arm should sit still.
  2. step pan to +0.4 rad. It should go there and stop.
  3. same, with gravity off. Separates gravity from control.

If test 1 drifts, nothing downstream can work.
"""

import numpy as np
import mujoco
from fit_mjcf import MJCF, TIMESTEP


def run(label, target=0.0, gravity=True, kp=20.0, n_s=2.0):
    m = mujoco.MjModel.from_xml_path(str(MJCF))
    m.opt.timestep = TIMESTEP
    if not gravity:
        m.opt.gravity[:] = 0
    m.actuator_gainprm[:, 0] = kp
    m.actuator_biasprm[:, 1] = -kp
    d = mujoco.MjData(m)

    d.qpos[:] = 0
    d.ctrl[:] = 0
    d.ctrl[0] = target
    mujoco.mj_forward(m, d)

    for _ in range(int(n_s / TIMESTEP)):
        mujoco.mj_step(m, d)

    print(f"{label}")
    print(f"  ctrl[0] = {target:+.3f}")
    print(f"  qpos    = {np.array2string(d.qpos, precision=3, suppress_small=True)}")
    print(f"  qvel    = {np.array2string(d.qvel, precision=3, suppress_small=True)}")
    print(
        f"  force   = {np.array2string(d.actuator_force, precision=3, suppress_small=True)}"
    )
    print()


run("1. all ctrl 0, gravity ON  -- should stay near zero")
run("2. pan -> +0.4, gravity ON", target=0.4)
run("3. pan -> +0.4, gravity OFF", target=0.4, gravity=False)
run("4. pan -> +0.4, gravity OFF, kp=200", target=0.4, gravity=False, kp=200.0)
