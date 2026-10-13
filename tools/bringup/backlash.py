#!/usr/bin/env python3
"""Measure backlash: slack in the gear train.

Approach the same target from both directions and compare where the
joint actually settles. The gap between the two is the backlash, plus
whatever the deadband contributes.

Deadband (1 tick each way) and the torque floor also produce a
direction-dependent stopping point, so what this measures is the TOTAL
direction-dependent error -- which is what the sim needs anyway.

Several targets across the range, because backlash varies with gear
engagement. Several repeats, because a single pair is one sample.

SAFETY: driven under power. Clear the swing. Run park.py first.
"""

import time, csv
import numpy as np
import matplotlib.pyplot as plt
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

SCS_ID = 1
DEVICENAME, BAUDRATE, PROTOCOL_END = "/dev/ttyACM0", 1000000, 0

CENTRE = 2096
TARGETS = [CENTRE - 400, CENTRE - 200, CENTRE, CENTRE + 200, CENTRE + 400]
RUN_UP = 300  # how far away to start the approach
N_REPEATS = 8
SETTLE_S = 2.0
OUT_CSV = "results/backlash_raw.csv"

p = PortHandler(DEVICENAME)
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE):
    raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)


def read_pos():
    try:
        v, comm, _ = h.read2ByteTxRx(p, SCS_ID, ADDR_PRESENT_POSITION)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None


def goto(t):
    h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, int(t))


input("Swing clear? park.py run? [enter] ")
h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows = []
for target in TARGETS:
    for rep in range(N_REPEATS):
        for direction, name in [(+1, "from_below"), (-1, "from_above")]:
            goto(target - direction * RUN_UP)
            time.sleep(SETTLE_S)
            goto(target)
            time.sleep(SETTLE_S)

            # median of several reads -- one read is one sample of noise
            vals = [v for _ in range(20) if (v := read_pos()) is not None]
            if vals:
                rows.append([target, rep, name, float(np.median(vals))])
    print(f"target {target}: done")

goto(CENTRE)
time.sleep(SETTLE_S)
p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["target", "rep", "direction", "settled"])
    w.writerows(rows)
print(f"\nwrote {len(rows)} measurements")

print(f"\n{'target':>7} {'below':>9} {'above':>9} {'gap':>8} {'sd':>7}")
gaps = []
for target in TARGETS:
    b = np.array([r[3] for r in rows if r[0] == target and r[2] == "from_below"])
    a_ = np.array([r[3] for r in rows if r[0] == target and r[2] == "from_above"])
    if len(b) < 3 or len(a_) < 3:
        continue
    gap = np.median(a_) - np.median(b)
    sd = np.sqrt(b.std() ** 2 + a_.std() ** 2)
    print(
        f"{target:>7} {np.median(b):>9.2f} {np.median(a_):>9.2f} "
        f"{gap:>8.2f} {sd:>7.2f}"
    )
    gaps.append(gap)

print(
    f"\nmean gap over the range: {np.mean(gaps):.2f} ticks "
    f"({np.mean(gaps) * 360/4096:.3f} deg)"
)
print("gap >> spread -> real backlash. gap ~ spread -> below the noise.")

plt.figure(figsize=(8, 5))
for target in TARGETS:
    for name, mk in [("from_below", "o"), ("from_above", "s")]:
        v = [r[3] - target for r in rows if r[0] == target and r[2] == name]
        plt.plot(
            [target] * len(v),
            v,
            mk,
            color="C0" if name == "from_below" else "C1",
            label=name if target == TARGETS[0] else None,
        )
plt.axhline(0, color="k", lw=0.5)
plt.xlabel("target (ticks)")
plt.ylabel("settled − target (ticks)")
plt.title("separation between the two colours = backlash")
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("results/backlash.png", dpi=120)
plt.show()
