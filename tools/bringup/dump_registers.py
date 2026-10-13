#!/usr/bin/env python3
"""Dump the configuration registers of all six servos.

The published control table is a community compilation, not Feetech's
own document. This reads what is actually on the bus. Two sources.
"""

from scservo_sdk import *

DEVICENAME, BAUDRATE, PROTOCOL_END = "/dev/ttyACM0", 1000000, 0
IDS = [1, 2, 3, 4, 5, 6]

# addr, name, size
REGS = [
    (16, "max_torque_limit", 2),
    (21, "P_coefficient", 1),
    (22, "D_coefficient", 1),
    (23, "I_coefficient", 1),
    (24, "min_startup_force", 2),
    (26, "cw_dead_zone", 1),
    (27, "ccw_dead_zone", 1),
    (33, "mode", 1),
    (41, "acceleration", 1),
    (46, "goal_speed", 2),
    (48, "torque_limit", 2),
]

p = PortHandler(DEVICENAME)
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE):
    raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)

print(f"{'register':>20} " + " ".join(f"{i:>6}" for i in IDS))
for addr, name, size in REGS:
    vals = []
    for i in IDS:
        read = h.read1ByteTxRx if size == 1 else h.read2ByteTxRx
        v, comm, err = read(p, i, addr)
        vals.append(str(v) if comm == COMM_SUCCESS else "ERR")
    print(f"{name:>20} " + " ".join(f"{v:>6}" for v in vals))

p.closePort()
