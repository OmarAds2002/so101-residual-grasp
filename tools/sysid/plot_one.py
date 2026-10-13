#!/usr/bin/env python3
"""Plot sim against real for ONE run. Diagnosis before tuning.

The fit reported RMS of 790 ticks (69 deg). That is not a parameter
problem, it is a structural one. The shape of the divergence says which.

  drifts away steadily      -> integration or a missing constraint
  right shape, wrong phase  -> the dead-time shift
  right shape, wrong size   -> scaling, units, or sign
  flat while real moves     -> commands are not reaching the actuator
  sim falls / goes wild     -> the non-moving joints are not held
"""

import sys
import numpy as np
import matplotlib.pyplot as plt
from fit_mjcf import load_run, simulate, TRAJ, MJCF, TIMESTEP, TICK2RAD, JOINTS
import mujoco

fname = sys.argv[1] if len(sys.argv) > 1 else "fit_pan_sine.csv"
params = (
    float(sys.argv[2]) if len(sys.argv) > 2 else 20.0,
    float(sys.argv[3]) if len(sys.argv) > 3 else 0.1,
    float(sys.argv[4]) if len(sys.argv) > 4 else 0.05,
)

model = mujoco.MjModel.from_xml_path(str(MJCF))
model.opt.timestep = TIMESTEP
data = mujoco.MjData(model)

run = load_run(TRAJ / fname)
pos, tau = simulate(model, data, run, params)

print(f"{fname}   kp={params[0]} damping={params[1]} friction={params[2]}")
for s in sorted(run):
    r = run[s]
    print(f"  joint {JOINTS[s]}")
    print(f"    real  range {r['meas'].min():+.3f} .. {r['meas'].max():+.3f} rad")
    print(f"    sim   range {pos[s].min():+.3f} .. {pos[s].max():+.3f} rad")
    print(f"    cmd   range {r['cmd'].min():+.3f} .. {r['cmd'].max():+.3f} rad")
    print(
        f"    rms error   {np.sqrt(np.mean((pos[s]-r['meas'])**2))/TICK2RAD:.1f} ticks"
    )

n = len(run)
fig, ax = plt.subplots(n, 1, figsize=(11, 4 * n), squeeze=False)
for k, s in enumerate(sorted(run)):
    r = run[s]
    a = ax[k][0]
    a.plot(r["t"], r["cmd"], "k--", lw=0.8, label="commanded")
    a.plot(r["t"], r["meas"], lw=1.2, label="real measured")
    a.plot(r["t"], pos[s], lw=1.2, label="sim measured")
    a.set_title(JOINTS[s])
    a.set_ylabel("rad")
    a.grid(alpha=0.3)
    a.legend()
ax[-1][0].set_xlabel("time (s)")
plt.tight_layout()
plt.savefig("diagnose.png", dpi=110)
plt.show()
