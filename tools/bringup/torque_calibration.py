#!/usr/bin/env python3
"""Convert present_load units into N*m, using a known mass.

The arm is held horizontal and straight. Read load with and without a
known mass in the gripper. The arm's OWN weight is identical in both,
so it cancels in the difference -- no need to know any link mass.

    delta_load  <->  mass * g * lever

giving load units per N*m, which converts the friction and damping
numbers from the velocity sweep into units MuJoCo accepts.

Horizontal matters: gravity torque is mass*g*lever*cos(angle), and at
horizontal cos = 1. Ten degrees off costs 1.5%, thirty costs 13%.

SAFETY: the arm holds a load at full extension. Support it before any
torque-off. Do not leave it loaded. Power is the stop.
"""

import time
import numpy as np
from scservo_sdk import *

ADDR_TORQUE_ENABLE    = 40
ADDR_GOAL_POSITION    = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD     = 60
ADDR_PRESENT_CURRENT  = 69

LIFT_ID = 2

# the horizontal pose, as set by hand this session
POSE = {2: 2000, 3: 1875, 4: 2020, 5: 2074}

MASS_KG = 0.209          # Galaxy S23 FE, Samsung spec
G       = 9.81
LEVER_M = 0.35            # <-- MEASURE THIS. metres. pivot to grip point.

N_READS  = 40            # per trial
N_TRIALS = 3             # repeat the whole pair this many times
SETTLE_S = 2.0

if LEVER_M <= 0:
    raise SystemExit("set LEVER_M to the measured distance in metres")

p = PortHandler('/dev/ttyACM0')
if not p.openPort():        raise SystemExit("failed to open port")
if not p.setBaudRate(1000000): raise SystemExit("failed baud")
h = PacketHandler(0)

def safe2(sid, addr):
    try:
        v, comm, _ = h.read2ByteTxRx(p, sid, addr)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None

def signed_load(sid):
    v = safe2(sid, ADDR_PRESENT_LOAD)
    if v is None: return None
    return -(v & 0x3FF) if (v & 0x400) else (v & 0x3FF)

def set_pose():
    for i, t in POSE.items():
        h.write1ByteTxRx(p, i, ADDR_TORQUE_ENABLE, 1)
        h.write2ByteTxRx(p, i, ADDR_GOAL_POSITION, t)
    time.sleep(SETTLE_S)

def sample(tag):
    set_pose()
    loads, currs = [], []
    for _ in range(N_READS):
        l, c = signed_load(LIFT_ID), safe2(LIFT_ID, ADDR_PRESENT_CURRENT)
        if l is not None and c is not None:
            loads.append(abs(l)); currs.append(c)
        time.sleep(0.02)
    pos = safe2(LIFT_ID, ADDR_PRESENT_POSITION)
    ml, sd = np.median(loads), np.std(loads)
    print(f"  {tag:<12} load {ml:7.1f} +/- {sd:5.2f}   "
          f"current {np.median(currs):5.1f}   pos {pos}   n={len(loads)}")
    return ml

print(f"\nmass {MASS_KG*1000:.0f} g, lever {LEVER_M*100:.1f} cm")
known_torque = MASS_KG * G * LEVER_M
print(f"known torque from the phone: {known_torque:.4f} N*m\n")

empties, loadeds = [], []
for trial in range(N_TRIALS):
    print(f"trial {trial}:")
    input("  gripper EMPTY, arm horizontal. [enter] ")
    empties.append(sample("empty"))
    input("  phone GRIPPED, arm horizontal. [enter] ")
    loadeds.append(sample("with phone"))

p.closePort()

e, l = np.array(empties), np.array(loadeds)
delta = l - e
print(f"\nempty      {e}")
print(f"with phone {l}")
print(f"delta      {delta}   mean {delta.mean():.1f}  sd {delta.std():.2f}")

scale = delta.mean() / known_torque
print(f"\nSCALE: {scale:.1f} load units per N*m")
print(f"       1 load unit = {1/scale:.5f} N*m")

print("\nconverting the velocity-sweep fit:")
print(f"  Coulomb friction  22.9 units  -> {22.9/scale:.4f} N*m")
print(f"  viscous damping  0.289 units per tick/s")
tick_per_rad = 4096 / (2*np.pi)
print(f"       -> {0.289 * tick_per_rad / scale:.5f} N*m*s/rad")
print("\n(frictionloss and damping for the MJCF)")