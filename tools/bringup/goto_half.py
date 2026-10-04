#!/usr/bin/env python3
"""Drive each joint to +0.5 rad using the measured calibration.

Zero looks the same whether a sign is right or wrong. +0.5 does not.
Compare each joint against MuJoCo with that joint at +0.5 and the
rest at 0. Same direction -> sign confirmed. Opposite -> sign wrong.

One joint at a time, and each returns to zero before the next moves.
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

# name, id, offset, sign, urdf lower, urdf upper
CAL = [
    ("shoulder_pan",  1, 2096, -1, -1.91986,  1.91986),
    ("shoulder_lift", 2, 2154,  1, -1.74533,  1.74533),
    ("elbow_flex",    3, 1875, -1, -1.69,     1.69),
    ("wrist_flex",    4, 2020,  1, -1.65806,  1.65806),
    ("wrist_roll",    5, 2074, -1, -2.74385,  2.84121),
    ("gripper",       6, 2266,  1, -0.174533, 1.74533),
]

TARGET_RAD = 0.5

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)


def to_ticks(offset, sign, rad):
    return round(offset + sign * rad_to_ticks(rad))


def move(sid, ticks):
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 1)
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_GOAL_ACC, SLOW_ACC)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_GOAL_SPEED, SLOW_SPEED)
    packetHandler.write2ByteTxRx(portHandler, sid, ADDR_GOAL_POSITION, ticks)
    time.sleep(2.0)
    pos, _, _ = packetHandler.read2ByteTxRx(portHandler, sid, ADDR_PRESENT_POS)
    return pos


print("Everything goes to zero first, then one joint at a time to +0.5 rad.")
print("Hand near the power switch.\n")

input("press Enter to send all six to zero ")
for name, sid, offset, sign, lo, hi in CAL:
    move(sid, to_ticks(offset, sign, 0.0))
print("at zero.\n")

results = {}

for name, sid, offset, sign, lo, hi in CAL:
    if not lo <= TARGET_RAD <= hi:
        print(f"{name}: {TARGET_RAD} rad is outside the URDF limits — SKIPPING\n")
        continue

    zero_ticks   = to_ticks(offset, sign, 0.0)
    target_ticks = to_ticks(offset, sign, TARGET_RAD)

    if not 0 <= target_ticks <= 4095:
        print(f"{name}: {target_ticks} ticks is outside 0-4095 — SKIPPING\n")
        continue

    print(f"--- {name} (id {sid}), sign {sign:+d}")
    print(f"    0.0 rad = {zero_ticks} ticks, "
          f"{TARGET_RAD:+.1f} rad = {target_ticks} ticks "
          f"({target_ticks - zero_ticks:+d})")
    input(f"    set {name} to +0.5 in MuJoCo, then Enter to move the arm ")

    reached = move(sid, target_ticks)
    print(f"    commanded {target_ticks}, reached {reached}, "
          f"error {reached - target_ticks:+d}")

    answer = input("    same direction as MuJoCo? (y/n) ").strip().lower()
    results[name] = (sign, answer.startswith("y"))

    input("    Enter to return it to zero ")
    move(sid, zero_ticks)
    print()

portHandler.closePort()

print("=== sign check ===")
all_good = True
for name, (sign, matched) in results.items():
    verdict = "confirmed" if matched else "WRONG — flip it"
    print(f"  {name:15s} sign {sign:+d}  {verdict}")
    if not matched:
        all_good = False

if all_good:
    print("\nAll signs confirmed by motion.")
else:
    print("\nAt least one sign is wrong. Flip it in calibration.yaml and rerun.")