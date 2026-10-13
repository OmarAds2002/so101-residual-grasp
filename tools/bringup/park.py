#!/usr/bin/env python3
"""Park the arm in the sysid pose: outboard links clear of the table.

Every system-identification run starts here, so friction, load and
reach are the same every time. Pose is logged with the results.

Approaches each target from the same side so the ~8 tick deadband
leaves the arm in a repeatable place rather than wherever it drifted.

SAFETY: support the arm before running. Power is the stop.
"""

import time
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56

DEVICENAME, BAUDRATE, PROTOCOL_END = "/dev/ttyACM0", 1000000, 0

# offsets from calibration.yaml = 0 rad for each joint
OFFSETS = {1: 2096, 2: 2154, 3: 1875, 4: 2020, 5: 2074, 6: 2266}

# sysid pose, as ticks away from each joint's zero.
# tune SHOULDER_LIFT / ELBOW until the wrist hangs in free air.
POSE = {
    1: 0,  # shoulder_pan  — stays at zero, this is the joint under test
    2: -400,  # shoulder_lift — raises the arm
    3: +500,  # elbow_flex    — folds the forearm up
    4: 0,
    5: 0,
    6: 0,
}

APPROACH = 60  # always arrive from this side
SETTLE_S = 1.5

p = PortHandler(DEVICENAME)
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE):
    raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)

input("Arm supported, hands clear, space clear? [enter] ")
for i in OFFSETS:
    h.write1ByteTxRx(p, i, ADDR_TORQUE_ENABLE, 1)

targets = {i: OFFSETS[i] + POSE[i] for i in OFFSETS}

# base first, then outward — the arm sweeps through less of the table
for i in sorted(targets):
    h.write2ByteTxRx(p, i, ADDR_GOAL_POSITION, targets[i] - APPROACH)
time.sleep(SETTLE_S)
for i in sorted(targets):
    h.write2ByteTxRx(p, i, ADDR_GOAL_POSITION, targets[i])
time.sleep(SETTLE_S)

print(f"{'id':>3} {'target':>8} {'actual':>8} {'err':>6}")
for i in sorted(targets):
    v, comm, err = h.read2ByteTxRx(p, i, ADDR_PRESENT_POSITION)
    if comm == COMM_SUCCESS:
        print(f"{i:>3} {targets[i]:>8} {v:>8} {targets[i]-v:>6}")
    else:
        print(f"{i:>3} {targets[i]:>8} {'ERR':>8}")

p.closePort()
print("\nTorque left ON. Arm is holding. Power is the stop.")
