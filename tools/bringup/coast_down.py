#!/usr/bin/env python3
"""Coast-down test on shoulder_pan: damping and friction, no controller.

Torque off, flick the joint by hand, record position until it stops.
Vertical axis means no gravity term in the model -- just inertia,
damping and friction.

  speed decays as a curve with a long tail -> viscous damping
  speed decays as a straight line, stops   -> Coulomb friction
  line that steepens at the end            -> both. Fit splits them.

Run several flicks of different strengths. Friction looks the same at
every speed; damping does not.

SAFETY: torque is OFF the whole time. The arm will not hold itself.
Keep shoulder_lift and elbow supported or parked low.
"""

import time, csv
import numpy as np
import matplotlib.pyplot as plt
from scservo_sdk import *

ADDR_TORQUE_ENABLE    = 40
ADDR_PRESENT_POSITION = 56

SCS_ID = 1
DEVICENAME, BAUDRATE, PROTOCOL_END = '/dev/ttyACM0', 1000000, 0

N_FLICKS   = 4
CAPTURE_S  = 3.0
MOVE_TICKS = 3          # motion that counts as "started"
STILL_S    = 0.4        # no motion this long = stopped
OUT_CSV    = "results/coast_down_raw.csv"

p = PortHandler(DEVICENAME)
if not p.openPort():            raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE): raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)

reader = GroupSyncRead(p, h, ADDR_PRESENT_POSITION, 2)
if not reader.addParam(SCS_ID):
    raise SystemExit(f"addParam failed for id {SCS_ID}")

def read_pos():
    if reader.txRxPacket() != COMM_SUCCESS: return None
    if not reader.isAvailable(SCS_ID, ADDR_PRESENT_POSITION, 2): return None
    return reader.getData(SCS_ID, ADDR_PRESENT_POSITION, 2)

input("Supporting the arm? Torque about to go OFF on shoulder_pan. [enter] ")
h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 0)

rows = []
for k in range(N_FLICKS):
    input(f"\nflick {k}: hold still, press enter, THEN flick the arm. [enter] ")

    rest = read_pos()
    print("  waiting for motion...")
    while abs((cur := read_pos() or rest) - rest) < MOVE_TICKS:
        pass

    t0 = time.perf_counter()
    last_move, last_pos = t0, cur
    print("  recording...")
    while (t := time.perf_counter()) - t0 < CAPTURE_S:
        v = read_pos()
        if v is None:
            continue
        rows.append([k, (t - t0) * 1000.0, v])
        if abs(v - last_pos) >= 1:
            last_move, last_pos = t, v
        elif t - last_move > STILL_S:
            break
    print(f"  stopped after {(t-t0)*1000:.0f} ms, "
          f"travelled {abs(v - rest)} ticks")

p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f); w.writerow(["flick", "t_ms", "ticks"]); w.writerows(rows)
print(f"\nwrote {len(rows)} samples to {OUT_CSV}")

# --- speed vs time ---------------------------------------------------
plt.figure(figsize=(9, 5))
for k in range(N_FLICKS):
    a = np.array([[r[1], r[2]] for r in rows if r[0] == k])
    if len(a) < 50:
        continue
    t, pos = a[:, 0], a[:, 1]
    w = max(1, int(round(10.0 / np.median(np.diff(t)))))   # 10 ms window
    v = np.abs((pos[2*w:] - pos[:-2*w]) / (t[2*w:] - t[:-2*w])) * 1000.0
    plt.plot(t[w:-w], v, label=f"flick {k}")

plt.xlabel("time since motion started (ms)")
plt.ylabel("|speed| (ticks/s)")
plt.title("straight line to zero = friction;  curve with a tail = damping")
plt.grid(alpha=0.3); plt.legend()
plt.tight_layout(); plt.savefig("results/coast_down.png", dpi=120)
plt.show()