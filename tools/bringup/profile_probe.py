#!/usr/bin/env python3
"""Find out whether the STS3215 runs an internal motion profile.

Commands step changes of three different sizes on ONE joint and records
position against time as fast as the bus allows.

Discriminator:
  no profile -> peak speed scales with step size
  profile    -> peak speed is capped, identical for every step size

Also prints the goal-speed / goal-acceleration registers, which say what
is configured. The motion says what actually happens. Two sources.

SAFETY: shoulder_pan rotates about a vertical axis, so gravity does not
pull it. Support nothing, but keep hands clear. Power is the stop.
"""

import time
import csv
import numpy as np
from scservo_sdk import *

# --- control table (STS/SMS) -------------------------------------------
ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_ACC = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED = 46
ADDR_PRESENT_POSITION = 56
ADDR_TORQUE_LIMIT = 48

# --- settings ----------------------------------------------------------
SCS_ID = 1  # shoulder_pan
BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0"
PROTOCOL_END = 0

START_TICKS = 2096  # shoulder_pan offset = 0 rad
STEP_TICKS = [40, 60, 90, 130, 190, 280, 400]  # ~0.15, 0.31, 0.61 rad
CAPTURE_S = 1.0  # record this long after each command
SETTLE_S = 1.5  # wait before each step

OUT_CSV = "results/profile_probe_raw.csv"

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort():
    raise SystemExit("failed to open port")
if not portHandler.setBaudRate(BAUDRATE):
    raise SystemExit("failed to set baudrate")
print("port open at", BAUDRATE)

# --- what is configured ------------------------------------------------
acc, r, e = packetHandler.read1ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_ACC)
spd, r, e = packetHandler.read2ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_SPEED)
print(f"goal_acc register   = {acc}")
print(f"goal_speed register = {spd}")
print("(0 means MAXIMUM on Feetech, not stopped)")

# --- one sample source, reused -----------------------------------------
reader = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 2)
if not reader.addParam(SCS_ID):
    raise SystemExit(f"addParam failed for id {SCS_ID}")


def read_pos():
    """Returns ticks, or None if the transaction failed."""
    if reader.txRxPacket() != COMM_SUCCESS:
        return None
    if not reader.isAvailable(SCS_ID, ADDR_PRESENT_POSITION, 2):
        return None
    return reader.getData(SCS_ID, ADDR_PRESENT_POSITION, 2)


def goto(ticks):
    packetHandler.write2ByteTxRx(portHandler, SCS_ID, ADDR_GOAL_POSITION, ticks)


# --- run ---------------------------------------------------------------
packetHandler.write1ByteTxRx(portHandler, SCS_ID, ADDR_TORQUE_ENABLE, 1)

rows = []  # step_ticks, t_rel_ms, ticks
failures = 0

for step in STEP_TICKS:
    goto(START_TICKS)
    time.sleep(SETTLE_S)

    p0 = read_pos()
    print(f"\nstep {step:4d}: resting at {p0}, commanding {START_TICKS + step}")

    goto(START_TICKS + step)
    t0 = time.perf_counter()  # clock starts when the write returns

    while True:
        t = time.perf_counter()
        if t - t0 > CAPTURE_S:
            break
        p = read_pos()
        if p is None:
            failures += 1
            continue
        rows.append([step, (t - t0) * 1000.0, p])

    settled = [r[2] for r in rows if r[0] == step][-1]
    print(
        f"           ended at {settled}  "
        f"({len([r for r in rows if r[0] == step])} samples)"
    )

goto(START_TICKS)
time.sleep(SETTLE_S)
portHandler.closePort()

# --- output ------------------------------------------------------------
with open(OUT_CSV, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["step_ticks", "t_ms", "ticks"])
    w.writerows(rows)

print(f"\nwrote {len(rows)} samples to {OUT_CSV}, {failures} failures")

# --- the discriminator -------------------------------------------------
print("\npeak speed per step size (ticks/s, from position differences):")
for step in STEP_TICKS:
    sel = np.array([[r[1], r[2]] for r in rows if r[0] == step])
    t_s = sel[:, 0] / 1000.0
    v = np.gradient(sel[:, 1], t_s)
    # median of the top 20 to shrug off single-sample noise
    peak = np.median(np.sort(np.abs(v))[-20:])
    print(f"  step {step:4d} ticks  ->  peak {peak:8.1f} ticks/s")

print("\nscales with step size -> no profile, you are seeing the controller")
print("roughly equal across all three -> a profile is running")
