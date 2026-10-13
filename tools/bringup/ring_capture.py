#!/usr/bin/env python3
"""Capture the settling ring after a step, to fit the inner P and D.

The ring is the only window where P and D are both live:
  - during transit the motor saturates, so P's value is invisible
  - at rest nothing moves, so D does nothing
  - while settling, the joint moves slowly enough not to saturate and
    fast enough that D matters

  how fast it wobbles -> P
  how fast it dies    -> D

Repeats each step N times and averages, aligned on the command. The ring
is the same every run; the 1-tick quantisation is not, so averaging
shrinks the noise and leaves the ring.

SAFETY: driven under power across up to 800 ticks. Clear the swing.
Power is the stop. Run park.py first; pose changes the answer.
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

START = 2096
STEPS = [300, 500, 800]
N_REPEATS = 10
CAPTURE_S = 2.0
APPROACH = 60  # always arrive at START from the same side
SETTLE_S = 1.5
GRID_MS = 0.5  # resample onto this grid before averaging
OUT_CSV = "results/ring_capture_raw.csv"

p = PortHandler(DEVICENAME)
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE):
    raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)
print("port open at", BAUDRATE)

reader = GroupSyncRead(p, h, ADDR_PRESENT_POSITION, 2)
if not reader.addParam(SCS_ID):
    raise SystemExit(f"addParam failed for id {SCS_ID}")


def read_pos():
    try:
        if reader.txRxPacket() != COMM_SUCCESS:
            return None
        if not reader.isAvailable(SCS_ID, ADDR_PRESENT_POSITION, 2):
            return None
        return reader.getData(SCS_ID, ADDR_PRESENT_POSITION, 2)
    except (IndexError, TypeError):
        return None


def goto(t):
    h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, int(t))


input("Swing clear of shoulder_pan? park.py already run? [enter] ")
h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows, fails = [], 0
for step in STEPS:
    for rep in range(N_REPEATS):
        goto(START - APPROACH)
        time.sleep(SETTLE_S)
        goto(START)
        time.sleep(SETTLE_S)

        goto(START + step)
        t0 = time.perf_counter()
        n = 0
        while (t := time.perf_counter()) - t0 < CAPTURE_S:
            v = read_pos()
            if v is None:
                fails += 1
                continue
            rows.append([step, rep, (t - t0) * 1000.0, v])
            n += 1
        print(f"step {step:4d} rep {rep:2d}: {n} samples")

goto(START)
time.sleep(SETTLE_S)
p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["step", "rep", "t_ms", "ticks"])
    w.writerows(rows)
print(f"\nwrote {len(rows)} samples to {OUT_CSV}, {fails} read failures")

# --- average the repeats, then look at the tail -----------------------
grid = np.arange(0, CAPTURE_S * 1000.0, GRID_MS)

fig, ax = plt.subplots(len(STEPS), 1, figsize=(10, 3.2 * len(STEPS)), squeeze=False)

print(f"\n{'step':>6} {'settled':>9} {'overshoot':>10} {'ring_pp':>9}")
for k, step in enumerate(STEPS):
    stack = []
    for rep in range(N_REPEATS):
        a = np.array([[r[2], r[3]] for r in rows if r[0] == step and r[1] == rep])
        if len(a) < 100:
            continue
        p0 = np.median(a[:20, 1])
        stack.append(np.interp(grid, a[:, 0], a[:, 1] - p0))

    if not stack:
        continue
    mean = np.mean(np.vstack(stack), axis=0)

    settled = np.median(mean[grid > 1500])
    # arrival: first time within 2% of the step
    arrive_idx = np.argmax(np.abs(mean - settled) < 0.02 * step)
    t_arrive = grid[arrive_idx]
    tail = mean[(grid > t_arrive + 50) & (grid < t_arrive + 600)]
    ring_pp = tail.max() - tail.min()
    undershoot = settled - mean[arrive_idx:].min()

    print(
        f"{step:>6} {settled:>9.2f} {t_arrive:>9.1f} "
        f"{ring_pp:>9.2f} {undershoot:>11.2f}"
    )

    axk = ax[k][0]
    axk.plot(grid, mean - settled, lw=1.0, label=f"mean of {len(stack)} reps")
    axk.axhline(0, color="k", lw=0.5)
    axk.set_xlim(300, 1400)
    lim = max(3.0, ring_pp)
    axk.set_ylim(-lim, lim)
    axk.set_ylabel("ticks from settled")
    axk.set_title(f"step {step} — settling tail")
    axk.grid(alpha=0.3)
    axk.legend()

ax[-1][0].set_xlabel("time since command (ms)")
plt.tight_layout()
plt.savefig("results/ring_capture.png", dpi=120)
plt.show()
