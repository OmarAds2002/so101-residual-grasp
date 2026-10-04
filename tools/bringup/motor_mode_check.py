#!/usr/bin/env python3
"""Does motor mode work at all? Smallest possible test.

Prints every read result including failures, so a silent swallow
cannot hide the problem.
"""

import time
from scservo_sdk import *

SCS_ID = 1
p = PortHandler('/dev/ttyACM0')
p.openPort(); p.setBaudRate(1000000)
h = PacketHandler(0)

def raw_read(addr, size, name):
    try:
        fn = h.read1ByteTxRx if size == 1 else h.read2ByteTxRx
        v, comm, err = fn(p, SCS_ID, addr)
        status = "ok" if comm == COMM_SUCCESS else f"comm={comm}"
        print(f"  {name:<20} {v:>6}   {status}")
        return v if comm == COMM_SUCCESS else None
    except Exception as e:
        print(f"  {name:<20} {'--':>6}   EXCEPTION {type(e).__name__}")
        return None

print("\n--- before any change ---")
for a, s, n in [(33, 1, "mode"), (9, 2, "min_angle"),
                (11, 2, "max_angle"), (56, 2, "position"),
                (60, 2, "load"), (40, 1, "torque_enable")]:
    raw_read(a, s, n)

print("\n--- switching to motor mode ---")
h.write1ByteTxRx(p, SCS_ID, 40, 0)
time.sleep(0.1)
h.write1ByteTxRx(p, SCS_ID, 55, 0)
time.sleep(0.1)
print("  min_angle ->", h.write2ByteTxRx(p, SCS_ID, 9, 0))
time.sleep(0.1)
print("  max_angle ->", h.write2ByteTxRx(p, SCS_ID, 11, 0))
time.sleep(0.1)
print("  mode      ->", h.write1ByteTxRx(p, SCS_ID, 33, 1))
time.sleep(0.1)
h.write1ByteTxRx(p, SCS_ID, 55, 1)
time.sleep(0.5)
h.write1ByteTxRx(p, SCS_ID, 40, 1)
time.sleep(0.5)

print("\n--- after the change, did it take? ---")
for a, s, n in [(33, 1, "mode"), (9, 2, "min_angle"),
                (11, 2, "max_angle"), (56, 2, "position"),
                (60, 2, "load"), (40, 1, "torque_enable")]:
    raw_read(a, s, n)

print("\n--- spin at 300 for 1.5 s ---")
input("  swing clear? [enter] ")
h.write2ByteTxRx(p, SCS_ID, 42, 300)
t0 = time.perf_counter()
while time.perf_counter() - t0 < 1.5:
    pv = raw_read(56, 2, "position")
    time.sleep(0.2)
h.write2ByteTxRx(p, SCS_ID, 42, 0)

print("\n--- restoring position mode ---")
h.write1ByteTxRx(p, SCS_ID, 40, 0); time.sleep(0.1)
h.write1ByteTxRx(p, SCS_ID, 55, 0); time.sleep(0.1)
h.write2ByteTxRx(p, SCS_ID, 9, 0);  time.sleep(0.1)
h.write2ByteTxRx(p, SCS_ID, 11, 4095); time.sleep(0.1)
h.write1ByteTxRx(p, SCS_ID, 33, 0); time.sleep(0.1)
h.write1ByteTxRx(p, SCS_ID, 55, 1); time.sleep(0.3)
for a, s, n in [(33, 1, "mode"), (9, 2, "min_angle"), (11, 2, "max_angle")]:
    raw_read(a, s, n)

p.closePort()