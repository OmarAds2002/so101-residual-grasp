#!/usr/bin/env python3
"""Re-analyse profile_probe_raw.csv without touching the arm.

The original peak-speed check differentiated quantised position at the
full sample rate, which measures the 1-tick quantisation and nothing else.
This works in position instead.

Discriminator:
  no profile -> normalised curves overlay; t50 roughly equal for all steps
  profile    -> t50 scales with step size; curves fan out
"""

import csv
import numpy as np
import matplotlib.pyplot as plt

IN_CSV   = "results/profile_probe_raw.csv"
VEL_WIN_MS = 10.0          # slope window; long enough to clear quantisation

# --- load, grouped by step --------------------------------------------
runs = {}
with open(IN_CSV) as f:
    for row in csv.DictReader(f):
        step = int(row["step_ticks"])
        runs.setdefault(step, []).append(
            (float(row["t_ms"]), float(row["ticks"])))

steps = sorted(runs)

def frac_crossing(t, frac_arr, level):
    """First time the response crosses `level` (0..1). None if it never does."""
    hit = np.where(frac_arr >= level)[0]
    return None if len(hit) == 0 else t[hit[0]]

print(f"{'step':>6} {'dt_ms':>7} {'t50_ms':>8} {'t90_ms':>8} "
      f"{'peak_ticks/s':>13} {'ss_err':>7}")

fig, ax = plt.subplots(2, 1, figsize=(9, 8))

for step in steps:
    arr = np.array(sorted(runs[step]))
    t, p = arr[:, 0], arr[:, 1]

    dt_ms = np.median(np.diff(t))
    p0 = np.median(p[:10])
    settled = np.median(p[-200:])
    frac = (p - p0) / step

    t50 = frac_crossing(t, frac, 0.5)
    t90 = frac_crossing(t, frac, 0.9)

    # slope over a window wide enough that motion >> 1 tick
    w = max(1, int(round(VEL_WIN_MS / dt_ms)))
    if len(p) > 2 * w:
        v = (p[2*w:] - p[:-2*w]) / (t[2*w:] - t[:-2*w]) * 1000.0
        peak = np.max(np.abs(v))
    else:
        peak = float("nan")

    ss_err = (p0 + step) - settled

    print(f"{step:6d} {dt_ms:7.3f} "
          f"{(t50 if t50 else float('nan')):8.2f} "
          f"{(t90 if t90 else float('nan')):8.2f} "
          f"{peak:13.1f} {ss_err:7.1f}")

    ax[0].plot(t, np.log10(p), label=f"step {step}")
    ax[1].plot(t, frac, label=f"step {step}")

ax[0].set_ylabel("position (ticks)")
ax[0].set_title("raw step responses")
ax[0].legend(); ax[0].grid(alpha=0.3)

ax[1].axhline(0.5, color="k", lw=0.5)
ax[1].axhline(1.0, color="k", lw=0.5)
ax[1].set_xlabel("time since command (ms)")
ax[1].set_ylabel("fraction of commanded step")
ax[1].set_title("normalised — overlay means no profile, fan-out means profile")
ax[1].set_xlim(0, 400)
ax[1].legend(); ax[1].grid(alpha=0.3)

plt.tight_layout()
plt.savefig("results/profile_probe.png", dpi=120)
print("\nwrote results/profile_probe.png")
plt.show()