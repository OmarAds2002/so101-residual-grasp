#!/usr/bin/env python3
"""Find out which physical joint each servo ID drives.

Nudges one servo a small amount, waits, returns it. You watch and
type what moved. Nothing is assumed about which ID is which joint.
"""

import time
from scservo_sdk import *

ADDR_TORQUE_ENABLE     = 40
ADDR_GOAL_ACC          = 41
ADDR_GOAL_POSITION     = 42
ADDR_GOAL_SPEED        = 46
ADDR_PRESENT_POSITION  = 56

SCS_IDS      = [1, 2, 3, 4, 5, 6]
BAUDRATE     = 1000000
DEVICENAME   = '/dev/ttyACM0'
PROTOCOL_END = 0

NUDGE_TICKS  = 100      # ~9 degrees
SLOW_SPEED   = 200      # ticks/sec-ish, deliberately slow
SLOW_ACC     = 10

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort():
    raise SystemExit("failed to open port")
if not portHandler.setBaudRate(BAUDRATE):
    raise SystemExit("failed to set baudrate")


def read_pos(scs_id):
    pos, comm, err = packetHandler.read2ByteTxRx(
        portHandler, scs_id, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS:
        raise SystemExit(f"read failed on id {scs_id}")
    return pos


def go_to(scs_id, ticks):
    packetHandler.write1ByteTxRx(portHandler, scs_id, ADDR_GOAL_ACC, SLOW_ACC)
    packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_GOAL_SPEED, SLOW_SPEED)
    packetHandler.write2ByteTxRx(portHandler, scs_id, ADDR_GOAL_POSITION, ticks)


print("Each servo will move about 9 degrees and come back.")
print("Watch the arm. Hand near the power switch.\n")

results = {}

for scs_id in SCS_IDS:
    start = read_pos(scs_id)
    print(f"--- ID {scs_id}: currently at {start} ticks")
    input("    press Enter to nudge it ")

    target = start + NUDGE_TICKS
    if target > 4000:                 # stay away from the ends
        target = start - NUDGE_TICKS

    go_to(scs_id, target)
    time.sleep(1.5)
    end = read_pos(scs_id)
    print(f"    moved {start} -> {end}  (ticks went "
          f"{'UP' if end > start else 'DOWN'})")

    joint = input("    which joint moved? ")
    direction = input("    which way did it physically go? (describe) ")

    go_to(scs_id, start)
    time.sleep(1.5)
    print(f"    returned to {read_pos(scs_id)}\n")

    results[scs_id] = {
        "joint": joint.strip(),
        "tick_direction": "up" if end > start else "down",
        "physical": direction.strip(),
        "start_ticks": start,
    }

portHandler.closePort()

print("\n=== ID -> joint map ===")
for scs_id, r in results.items():
    print(f"ID {scs_id}: {r['joint']}   "
          f"ticks {r['tick_direction']} -> {r['physical']}   "
          f"(rest {r['start_ticks']})")