#!/usr/bin/env python3
"""Friction and damping, via a commanded constant velocity.

Position mode only -- no mode switch, no EEPROM write. The script walks
the goal position forward in small steps at a fixed wall-clock rate, so
the joint tracks a ramp at a speed WE set. Read position and load while
it does.

At constant speed there is no acceleration, so inertia drops out and the
load reading is the resisting torque.

  load vs speed:  intercept at zero = Coulomb friction
                  slope             = viscous damping

This also exercises exactly the code path the real 200 Hz controller
uses, which makes the number relevant where it will be used.

SAFETY: driven under power across ~1200 ticks. Clear the swing.
Power is the stop. Run park.py first; pose changes the answer.
"""

import time, csv
import numpy as np
import matplotlib.pyplot as plt
from scservo_sdk import *

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD     = 60

SCS_ID = 1
DEVICENAME, BAUDRATE, PROTOCOL_END = '/dev/ttyACM0', 1000000, 0

CENTRE   = 2096
HALF     = 600          # sweep CENTRE-HALF .. CENTRE+HALF
MARGIN   = 150          # drop samples within this of either end
DT       = 0.005        # 5 ms command period = 200 Hz, same as the real loop
SPEEDS   = [100, 200, 400, 700, 1100, 1600]      # ticks/s, commanded
OUT_CSV  = "results/velocity_sweep_raw.csv"

p = PortHandler(DEVICENAME)
if not p.openPort():            raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE): raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)
print("port open at", BAUDRATE)

def safe2(addr):
    """read2 that survives a short reply instead of raising."""
    try:
        v, comm, _ = h.read2ByteTxRx(p, SCS_ID, addr)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None

def load():
    v = safe2(ADDR_PRESENT_LOAD)
    if v is None:
        return None
    mag = v & 0x3FF
    return -mag if (v & 0x400) else mag

def goto(t):
    h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, int(t))

input("Swing clear of shoulder_pan? park.py already run? [enter] ")
h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows, fails = [], 0
for s in SPEEDS:
    for direction in (+1, -1):
        start = CENTRE - direction * HALF
        goto(start); time.sleep(2.5)

        target = float(start)
        t0 = time.perf_counter()
        n = 0
        while abs(target - start) < 2 * HALF:
            target += direction * s * DT
            goto(target)

            pv, lv = safe2(ADDR_PRESENT_POSITION), load()
            if pv is None or lv is None:
                fails += 1
            elif abs(pv - CENTRE) < HALF - MARGIN:
                rows.append([s, direction,
                             (time.perf_counter() - t0) * 1000.0, pv, lv])
                n += 1

            # hold the commanded rate
            nxt = t0 + (abs(target - start) / s)
            while time.perf_counter() < nxt:
                pass

        print(f"speed {s:5d} dir {direction:+d}: {n} samples")

goto(CENTRE); time.sleep(2.0)
p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["cmd_speed", "dir", "t_ms", "ticks", "load"])
    w.writerows(rows)
print(f"\nwrote {len(rows)} samples to {OUT_CSV}, {fails} read failures")

# --- fit --------------------------------------------------------------
print(f"\n{'cmd':>6} {'dir':>4} {'actual':>10} {'|load|':>8} {'n':>6}")
xs, ys, cols = [], [], []
for s in SPEEDS:
    for d in (+1, -1):
        a = np.array([[r[2], r[3], r[4]] for r in rows
                      if r[0] == s and r[1] == d])
        if len(a) < 30:
            continue
        actual = abs(np.polyfit(a[:, 0] / 1000.0, a[:, 1], 1)[0])
        lmag = np.median(np.abs(a[:, 2]))
        print(f"{s:>6} {d:>+4} {actual:>10.1f} {lmag:>8.1f} {len(a):>6}")
        xs.append(actual); ys.append(lmag); cols.append(d)

if len(xs) < 3:
    raise SystemExit("not enough points to fit -- check the sample counts")

xs, ys, cols = np.array(xs), np.array(ys), np.array(cols)
slope, intercept = np.polyfit(xs, ys, 1)
print(f"\nCoulomb friction (intercept): {intercept:.1f} load units")
print(f"viscous damping   (slope)   : {slope:.5f} load units per tick/s")

for d, name in [(+1, "forward"), (-1, "reverse")]:
    m = cols == d
    if m.sum() >= 2:
        sl, ic = np.polyfit(xs[m], ys[m], 1)
        print(f"  {name:<8} intercept {ic:7.1f}   slope {sl:.5f}")

plt.figure(figsize=(8, 5))
for d, mk, name in [(+1, "o", "forward"), (-1, "s", "reverse")]:
    m = cols == d
    plt.plot(xs[m], ys[m], mk, label=name)
ref = np.array([0, xs.max()])
plt.plot(ref, slope * ref + intercept, "-",
         label=f"fit: {slope:.4f}v + {intercept:.1f}")
plt.xlabel("actual speed (ticks/s)")
plt.ylabel("|present_load|")
plt.title("intercept = Coulomb friction, slope = viscous damping")
plt.grid(alpha=0.3); plt.legend()
plt.tight_layout(); plt.savefig("results/velocity_sweep.png", dpi=120)
plt.show()