#!/usr/bin/env python3
"""Open and close the gripper (ID 6).

  python gripper.py open
  python gripper.py close
  python gripper.py 2400        # go to a specific tick value
  python gripper.py read        # just report position and load

Gripper offset (0 rad) is 2266. URDF limits are -0.1745 to +1.7453 rad,
so in ticks roughly 2152 to 3404. Which direction opens is not recorded
anywhere, so START WITH SMALL MOVES and watch.

The gripper's lower stop is rubber pads squeezing, not a hard stop --
S03 flagged its offset as the least trustworthy of the six. Closing too
far just squeezes harder.
"""

import sys, time
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60

SCS_ID = 6
OFFSET = 2266  # 0 rad, from calibration.yaml

OPEN_TICKS = OFFSET + 350  # tune these once you see which way it goes
CLOSE_TICKS = OFFSET + 60

p = PortHandler("/dev/ttyACM0")
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(1000000):
    raise SystemExit("failed baud")
h = PacketHandler(0)


def read_pos():
    try:
        v, comm, _ = h.read2ByteTxRx(p, SCS_ID, ADDR_PRESENT_POSITION)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None


def read_load():
    try:
        v, comm, _ = h.read2ByteTxRx(p, SCS_ID, ADDR_PRESENT_LOAD)
        if comm != COMM_SUCCESS:
            return None
        mag = v & 0x3FF
        return -mag if (v & 0x400) else mag
    except (IndexError, TypeError):
        return None


arg = sys.argv[1] if len(sys.argv) > 1 else "read"

if arg == "read":
    print(f"position {read_pos()}   load {read_load()}")
    p.closePort()
    raise SystemExit

target = {"open": OPEN_TICKS, "close": CLOSE_TICKS}.get(arg)
if target is None:
    target = int(arg)

if not 2100 <= target <= 3450:
    raise SystemExit(f"{target} is outside the gripper's range, refusing")

h.write1ByteTxRx(p, SCS_ID, ADDR_TORQUE_ENABLE, 1)
print(f"from {read_pos()} -> {target}")
h.write2ByteTxRx(p, SCS_ID, ADDR_GOAL_POSITION, target)

for _ in range(12):
    time.sleep(0.25)
    print(f"  pos {read_pos()}   load {read_load()}")

p.closePort()
