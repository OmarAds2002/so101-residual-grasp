#!/usr/bin/env python3
"""Re-measure elbow_flex and wrist_roll stops.

Fixes the labelling trap from last time. You are NOT asked which end is
"lower" — you just push it to one end, then the other. The script works
out which is which from the tick readings and the URDF range.
"""

import math
from scservo_sdk import *

ADDR_TORQUE_ENABLE    = 40
ADDR_PRESENT_POSITION = 56

BAUDRATE     = 1000000
DEVICENAME   = '/dev/ttyACM0'
PROTOCOL_END = 0

TICKS_PER_REV = 4096
def rad_to_ticks(r):
    return r * TICKS_PER_REV / (2 * math.pi)

# name, id, urdf lower, urdf upper, current offset in the file
JOINTS = [
    ("elbow_flex", 3, -1.69,    1.69,    1875),
    ("wrist_roll", 5, -2.74385, 2.84121, 2074),
]

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)
portHandler.openPort()
portHandler.setBaudRate(BAUDRATE)

for name, sid, lo, hi, _ in JOINTS:
    packetHandler.write1ByteTxRx(portHandler, sid, ADDR_TORQUE_ENABLE, 0)
print("Torque is OFF on elbow_flex and wrist_roll. Support the arm.\n")

def read_pos(sid):
    pos, comm, err = packetHandler.read2ByteTxRx(
        portHandler, sid, ADDR_PRESENT_POSITION)
    if comm != COMM_SUCCESS:
        raise SystemExit(f"read failed on id {sid}")
    return pos

for name, sid, lo, hi, old_offset in JOINTS:
    print(f"=== {name} (id {sid})")
    print(f"    URDF range {lo:+.4f} to {hi:+.4f} rad "
          f"= {rad_to_ticks(hi - lo):.0f} ticks of travel")

    input("    push it gently to ONE end, then Enter ")
    t_a = read_pos(sid)
    print(f"    end A = {t_a} ticks")

    input("    now push it gently to the OTHER end, then Enter ")
    t_b = read_pos(sid)
    print(f"    end B = {t_b} ticks")

    # The script decides which end is which, not you.
    # sign = +1 means increasing ticks goes the URDF's positive way.
    # If we assume sign +1, then the URDF's lower limit is at the
    # SMALLER tick value.
    t_min, t_max = min(t_a, t_b), max(t_a, t_b)

    measured = t_max - t_min
    expected = rad_to_ticks(hi - lo)
    print(f"\n    measured span {measured} ticks, URDF says {expected:.0f}, "
          f"diff {measured - expected:+.0f}")

    for sign in (+1, -1):
        if sign == +1:
            off_lo = t_min - rad_to_ticks(lo)
            off_hi = t_max - rad_to_ticks(hi)
        else:
            off_lo = t_max + rad_to_ticks(lo)
            off_hi = t_min + rad_to_ticks(hi)
        mid = (off_lo + off_hi) / 2
        print(f"    if sign = {sign:+d}: offset {off_lo:.0f} / {off_hi:.0f} "
              f"-> midpoint {mid:.0f}   (file says {old_offset})")

    print("    -> take the sign whose midpoint is near the file's value.\n")

portHandler.closePort()
print("Torque still off on those two. Power down or re-enable.")