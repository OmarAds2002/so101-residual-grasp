#!/usr/bin/env python3
"""Peak speed vs step size, log-log, to find the regime.

v = C * step^n  ->  log v = log C + n * log step
So the slope of a log-log plot IS the exponent.

  slope ~ 1.0  -> linear regime; the PD controller is visible
  slope ~ 0.5  -> constant-acceleration regime (profile or saturation)
  a bend       -> the transition between them. That bend is the prize.
"""

import csv
import numpy as np
import matplotlib.pyplot as plt

IN_CSV     = "results/profile_probe_raw.csv"
VEL_WIN_MS = 10.0
DEADBAND   = 8.0        # measured: runs stop ~8 ticks short

runs = {}
with open(IN_CSV) as f:
    for row in csv.DictReader(f):
        step = int(row["step_ticks"])
        runs.setdefault(step, []).append(
            (float(row["t_ms"]), float(row["ticks"])))

steps, peaks, moved = [], [], []

print(f"{'step':>6} {'moved':>7} {'peak':>10}")
for step in sorted(runs):
    arr = np.array(sorted(runs[step]))
    t, p = arr[:, 0], arr[:, 1]

    p0 = np.median(p[:10])
    settled = np.median(p[-200:])
    actual = settled - p0          # what it really travelled

    dt_ms = np.median(np.diff(t))
    w = max(1, int(round(VEL_WIN_MS / dt_ms)))
    v = (p[2*w:] - p[:-2*w]) / (t[2*w:] - t[:-2*w]) * 1000.0
    peak = np.max(np.abs(v))

    print(f"{step:6d} {actual:7.1f} {peak:10.1f}")
    steps.append(step); peaks.append(peak); moved.append(actual)

steps = np.array(steps, float)
peaks = np.array(peaks, float)
moved = np.array(moved, float)

# fit against DISTANCE ACTUALLY TRAVELLED, not the commanded step.
# deadband means those differ, and the physics follows the real distance.
lx, ly = np.log10(moved), np.log10(peaks)

n_all, c_all = np.polyfit(lx, ly, 1)
print(f"\nslope over all points        : {n_all:.3f}")

lo = moved <= 100
hi = moved >= 100
if lo.sum() >= 2:
    n_lo = np.polyfit(lx[lo], ly[lo], 1)[0]
    print(f"slope, small steps (<=100)   : {n_lo:.3f}")
if hi.sum() >= 2:
    n_hi = np.polyfit(lx[hi], ly[hi], 1)[0]
    print(f"slope, large steps (>=100)   : {n_hi:.3f}")

plt.figure(figsize=(7, 6))
plt.loglog(moved, peaks, "o-", label="measured")

ref = np.array([moved.min(), moved.max()])
plt.loglog(ref, peaks[0] * (ref / moved[0])**1.0, "--",
           label="slope 1.0 (linear / PD visible)")
plt.loglog(ref, peaks[0] * (ref / moved[0])**0.5, ":",
           label="slope 0.5 (constant acceleration)")

plt.xlabel("distance actually travelled (ticks)")
plt.ylabel("peak speed (ticks/s)")
plt.title(f"regime check — fitted slope {n_all:.2f}")
plt.grid(True, which="both", alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("results/slope_fit.png", dpi=120)
plt.show()