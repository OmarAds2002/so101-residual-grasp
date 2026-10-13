#!/usr/bin/env python3
"""Trajectory generator, or torque saturation?

Same joint, same step, three torque limits. Register 48 is RAM, so no
EEPROM unlock and it resets on power cycle.

  acceleration drops with the limit -> SATURATION. The controller is
                                       asking for more than it can get.
  curves unchanged                  -> GENERATOR. Software planning.

Run from the sysid park pose (python park.py). Links must be clear of
the table -- dragging adds load and contaminates the comparison.
"""

import time, csv
import numpy as np
import matplotlib.pyplot as plt
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_LIMIT = 48
ADDR_PRESENT_POSITION = 56

SCS_ID = 1  # shoulder_pan: vertical axis, gravity-neutral
DEVICENAME, BAUDRATE, PROTOCOL_END = "/dev/ttyACM0", 1000000, 0

START_TICKS = 2096
STEP = 300
LIMITS = [1000, 500, 250]
APPROACH = 60  # always arrive at the start from one side
CAPTURE_S = 1.0
SETTLE_S = 1.5
OUT_CSV = "results/torque_sweep_raw.csv"

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
    if reader.txRxPacket() != COMM_SUCCESS:
        return None
    if not reader.isAvailable(SCS_ID, ADDR_PRESENT_POSITION, 2):
        return None
    return reader.getData(SCS_ID, ADDR_PRESENT_POSITION, 2)


def goto(t):
    h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, t)


h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows = []
for limit in LIMITS:
    h.write2ByteTxRx(p, SCS_ID, ADDR_TORQUE_LIMIT, limit)

    goto(START_TICKS - APPROACH)
    time.sleep(SETTLE_S)
    goto(START_TICKS)
    time.sleep(SETTLE_S)

    p0 = read_pos()
    print(f"\nlimit {limit:4d}: resting at {p0}, commanding {START_TICKS+STEP}")

    goto(START_TICKS + STEP)
    t0 = time.perf_counter()
    n = 0
    while (t := time.perf_counter()) - t0 < CAPTURE_S:
        v = read_pos()
        if v is not None:
            rows.append([limit, (t - t0) * 1000.0, v])
            n += 1
    print(f"            {n} samples")

h.write2ByteTxRx(p, SCS_ID, ADDR_TORQUE_LIMIT, 1000)  # restore
goto(START_TICKS)
time.sleep(SETTLE_S)
p.closePort()

with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["limit", "t_ms", "ticks"])
    w.writerows(rows)
print(f"\nwrote {len(rows)} samples to {OUT_CSV}")

# --- compare ---------------------------------------------------------
print(f"\n{'limit':>6} {'peak':>10} {'travelled':>10} {'short_by':>9}")
plt.figure(figsize=(8, 5))
for limit in LIMITS:
    a = np.array([[r[1], r[2]] for r in rows if r[0] == limit])
    t, pos = a[:, 0], a[:, 1]
    p0 = np.median(pos[:10])
    settled = np.median(pos[-200:])

    w = max(1, int(round(10.0 / np.median(np.diff(t)))))
    v = np.abs((pos[2 * w :] - pos[: -2 * w]) / (t[2 * w :] - t[: -2 * w])) * 1000.0

    print(
        f"{limit:>6} {v.max():>10.1f} {settled-p0:>10.1f} " f"{STEP-(settled-p0):>9.1f}"
    )
    plt.plot(t, pos - p0, label=f"torque limit {limit}")

plt.xlabel("time since command (ms)")
plt.ylabel("distance travelled (ticks)")
plt.title("split = torque saturation,  overlay = trajectory generator")
plt.xlim(0, 800)
plt.grid(alpha=0.3)
plt.legend()
plt.tight_layout()
plt.savefig("results/torque_sweep.png", dpi=120)
plt.show()
