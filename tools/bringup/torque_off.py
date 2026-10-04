#!/usr/bin/env python3
"""Disable torque on one joint, or all six.

  python torque_off.py        -> all six
  python torque_off.py 2      -> shoulder_lift only

SAFETY: the arm FALLS the moment this runs. Hold it first.
Torque off is not a stop, it is a drop.
"""

import sys
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
IDS = [int(a) for a in sys.argv[1:]] or [1, 2, 3, 4, 5, 6]

p = PortHandler('/dev/ttyACM0')
if not p.openPort():        raise SystemExit("failed to open port")
if not p.setBaudRate(1000000): raise SystemExit("failed baud")
h = PacketHandler(0)

input(f"Arm supported by hand? Releasing {IDS}. [enter] ")
for i in IDS:
    comm, err = h.write1ByteTxRx(p, i, ADDR_TORQUE_ENABLE, 0)
    print(f"id {i}: {'off' if comm == COMM_SUCCESS else 'FAILED'}")

p.closePort()