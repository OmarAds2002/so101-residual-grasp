#!/usr/bin/env python3
"""Drive the arm to zero radians using the measured calibration,
then hold. Compare against MuJoCo with all qpos at zero.

Runs one joint at a time so nothing swings through anything.
"""

import math
import time
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_ACC      = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_SPEED    = 46
ADDR_PRESENT_POS   = 56

BAUDRATE     = 1000000
DEVICENAME   = '/dev/ttyACM0'
PROTOCOL_END = 0

SLOW_SPEED = 300
SLOW_ACC   = 10

TICKS_PER_REV = 4096
def rad_to_ticks(r):
    return r * TICKS_PER_REV / (2 * math.pi)

# name, id, offset, sign
CAL = [
    ("shoulder_pan",  1, 2096, -1),
    ("shoulder_lift", 2, 2154,  1),
    ("elbow_flex",    3, 1875, -1),
    ("wrist_flex",    4, 2020,  1),
    ("wrist_roll",    5, 2074, -1),
    ("gripper",       6, 2266,  1),
]

TARGET_RAD = 0.0          # change to 0.5 for the second pass

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

print(f"Driving every joint to {TARGET_RAD:+.2f} rad, one at a time.")
print("Hand near the power switch.\n")

for name, sid, offset, sign in CAL:
    ticks = round(offset + sign * rad_to_ticks(TARGET_RAD))

    if not 0 <= ticks <= 4095:
        print(f"{name}: {ticks} ticks is outside 0-4095 — SKIPPING")
        continue

    print(f"--- {name} (id {sid})")
    print(f"    {TARGET_RAD:+.2f} rad -> {ticks} ticks")
    input("    press Enter to move it ")

    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_GOAL_ACC, SLOW_ACC)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_GOAL_SPEED, SLOW_SPEED)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_GOAL_POSITION, ticks)

    time.sleep(2.0)
    pos, _, _ = packetHandler.read2ByteTxRx(portHandler, sid, ADDR_PRESENT_POS)
    print(f"    commanded {ticks}, reached {pos}, error {pos - ticks:+d}\n")

portHandler.closePort()
print("All six at target. Torque is ON — they are holding.")
print("Compare to MuJoCo with all joints at zero.")