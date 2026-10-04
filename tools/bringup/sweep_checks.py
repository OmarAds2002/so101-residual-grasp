#!/usr/bin/env python3
"""Two checks on the velocity-sweep result.

1. Is present_load (reg 60) linear in torque? Cross-check against
   present_current (reg 69, x6.5 mA). If they track proportionally,
   load is a usable torque proxy. If not, the 0.2949/20.9 fit absorbed
   the nonlinearity.

2. Is tracking error polluting the damping slope? We walk the goal
   position at a fixed rate; the servo chases it. Part of the measured
   load is the servo fighting its own position error, not friction.
   If (target - measured) is constant across speeds, the slope is
   friction. If it grows with speed, some of the slope is tracking.

SAFETY: driven under power across ~1200 ticks. Clear the swing.
Run park.py first.
"""

import time, csv
import numpy as np
import matplotlib.pyplot as plt
from scservo_sdk import *

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD     = 60
ADDR_PRESENT_CURRENT  = 69

SCS_ID = 1
DEVICENAME, BAUDRATE, PROTOCOL_END = '/dev/ttyACM0', 1000000, 0

CENTRE, HALF, MARGIN = 2096, 600, 150
DT      = 0.005
SPEEDS  = [100, 200, 400, 700, 1100, 1600]
OUT_CSV = "results/sweep_checks_raw.csv"

p = PortHandler(DEVICENAME)
if not p.openPort():            raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE): raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)

def safe2(addr):
    try:
        v, comm, _ = h.read2ByteTxRx(p, SCS_ID, addr)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None

def signed_load():
    v = safe2(ADDR_PRESENT_LOAD)
    if v is None: return None
    return -(v & 0x3FF) if (v & 0x400) else (v & 0x3FF)

def goto(t):
    h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, int(t))

input("Swing clear? park.py run? [enter] ")
h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows, fails = [], 0
for s in SPEEDS:
    for direction in (+1, -1):
        start = CENTRE - direction * HALF
        goto(start); time.sleep(2.5)

        target = float(start)
        t0 = time.perf_counter()
        while abs(target - start) < 2 * HALF:
            target += direction * s * DT
            goto(target)

            pv = safe2(ADDR_PRESENT_POSITION)
            lv = signed_load()
            cv = safe2(ADDR_PRESENT_CURRENT)
            if pv is None or lv is None or cv is None:
                fails += 1
            elif abs(pv - CENTRE) < HALF - MARGIN:
                rows.append([s, direction,
                             (time.perf_counter() - t0) * 1000.0,
                             target, pv, lv, cv])

            nxt = t0 + (abs(target - start) / s)
            while time.perf_counter() < nxt:
                pass
        print(f"speed {s:5d} dir {direction:+d}: done")

goto(CENTRE); time.sleep(2.0)
p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["cmd_speed", "dir", "t_ms", "target", "ticks",
                "load", "current"])
    w.writerows(rows)
print(f"\nwrote {len(rows)} samples, {fails} read failures")

a = np.array(rows, dtype=float)
speed, dirn, tgt, pos = a[:, 0], a[:, 1], a[:, 3], a[:, 4]
load, curr = np.abs(a[:, 5]), a[:, 6]
lag = np.abs(tgt - pos)

print(f"\n{'speed':>6} {'|load|':>8} {'current':>9} "
      f"{'load/cur':>9} {'lag':>8}")
L, C, G, S = [], [], [], []
for s in SPEEDS:
    m = speed == s
    if m.sum() < 30: continue
    ml, mc, mg = np.median(load[m]), np.median(curr[m]), np.median(lag[m])
    ratio = ml / mc if mc else float("nan")
    print(f"{s:>6} {ml:>8.1f} {mc:>9.1f} {ratio:>9.3f} {mg:>8.1f}")
    L.append(ml); C.append(mc); G.append(mg); S.append(s)

print("\nCHECK 1 — load/current ratio roughly CONSTANT -> load is linear "
      "in torque.")
print("CHECK 2 — lag roughly CONSTANT across speeds -> slope is friction,"
      " not tracking.")

fig, ax = plt.subplots(1, 2, figsize=(12, 5))
ax[0].plot(C, L, "o-")
ax[0].set_xlabel("present_current"); ax[0].set_ylabel("|present_load|")
ax[0].set_title("linear = load is a torque proxy"); ax[0].grid(alpha=0.3)
ax[1].plot(S, G, "o-")
ax[1].set_xlabel("commanded speed (ticks/s)")
ax[1].set_ylabel("|target - measured| (ticks)")
ax[1].set_title("flat = friction;  rising = tracking error")
ax[1].grid(alpha=0.3)
plt.tight_layout(); plt.savefig("results/sweep_checks.png", dpi=120)
plt.show()