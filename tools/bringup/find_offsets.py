#!/usr/bin/env python3
"""Find tick offsets by reading at each joint's mechanical stops.

You move the joint by hand. The script reads and does the arithmetic.
Torque is OFF the whole time — support the arm.
"""

import math
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_PRESENT_POSITION = 56

BAUDRATE = 1000000
DEVICENAME = "/dev/ttyACM0"
PROTOCOL_END = 0

TICKS_PER_REV = 4096


def rad_to_ticks(r):
    return r * TICKS_PER_REV / (2 * math.pi)


# joint name, servo id, urdf lower, urdf upper
JOINTS = [
    ("shoulder_pan", 1, -1.91986, 1.91986),
    ("shoulder_lift", 2, -1.74533, 1.74533),
    ("elbow_flex", 3, -1.69, 1.69),
    ("wrist_flex", 4, -1.65806, 1.65806),
    ("wrist_roll", 5, -2.74385, 2.84121),
    ("gripper", 6, -0.174533, 1.74533),
]

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

# torque off everywhere so you can move it by hand
for name, sid, lo, hi in JOINTS:
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 0)
print("Torque is OFF on all six. Support the arm.\n")


def read_pos(sid):
    pos, comm, err = packetHandler.read2ByteTxRx(
        portHandler, sid, ADDR_PRESENT_POSITION
    )
    if comm != COMM_SUCCESS:
        raise SystemExit(f"read failed on id {sid}")
    return pos


for name, sid, lo, hi in JOINTS:
    print(f"--- {name} (id {sid})")
    print(f"    URDF range {lo:+.4f} to {hi:+.4f} rad")

    input("    move it gently to the LOWER stop, then Enter ")
    t_lo = read_pos(sid)
    print(f"    lower stop = {t_lo} ticks")

    input("    move it gently to the UPPER stop, then Enter ")
    t_hi = read_pos(sid)
    print(f"    upper stop = {t_hi} ticks")

    measured = abs(t_hi - t_lo)
    expected = rad_to_ticks(hi - lo)
    print(f"    measured span {measured} ticks, URDF says {expected:.0f}")
    print(f"    difference    {measured - expected:+.0f} ticks")

    sign = 1 if t_hi > t_lo else -1
    offset_from_lo = t_lo - sign * rad_to_ticks(lo)
    offset_from_hi = t_hi - sign * rad_to_ticks(hi)
    print(f"    sign = {sign:+d}")
    print(f"    offset from lower stop: {offset_from_lo:.0f}")
    print(f"    offset from upper stop: {offset_from_hi:.0f}")
    print(f"    (these two should agree)\n")

portHandler.closePort()
print("Torque still off. Power down or re-enable before moving on.")
