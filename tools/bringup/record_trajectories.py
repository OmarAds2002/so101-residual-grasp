#!/usr/bin/env python3
"""Record real trajectories for the MJCF fit.

Records, per sample: the command we sent, the measured position, and
the load. The sim will later be driven with the SAME commands and its
measured positions compared against these.

End-effector position is NOT recorded -- there is no sensor for it. It
would be computed from joint angles, so it carries nothing the angles
do not already have.

Speed band 50-500 ticks/s: roughly what tabletop grasping needs at
200 Hz control, and below where lag runs away (7 ticks at 100, 161 at
1100). Saturated motion teaches the fit nothing about kp, because
during saturation kp has no effect.

SET SPLIT:
  fit        -- single-joint moves, 5 trajectories
  validate   -- different IN KIND, 3 trajectories, never fitted against
A good match on fitted data only shows the optimiser worked. Three
parameters admit several combinations that match the same recording;
only held-out data distinguishes them.

SAFETY: driven under power. Clear the swing. Support before torque-off.
Power is the stop.
"""

import time, csv, sys
import numpy as np
from scservo_sdk import *

ADDR_TORQUE_ENABLE = 40
ADDR_GOAL_POSITION = 42
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_LOAD = 60

DEVICENAME, BAUDRATE, PROTOCOL_END = "/dev/ttyACM0", 1000000, 0
DT = 0.005  # 200 Hz, the real control period

OFFSETS = {1: 2096, 2: 2154, 3: 1875, 4: 2020, 5: 2074, 6: 2266}
HOME = {1: 2096, 2: 2000, 3: 1875, 4: 2020, 5: 2074, 6: 2266}

OUT_DIR = "results/trajectories"

p = PortHandler(DEVICENAME)
if not p.openPort():
    raise SystemExit("failed to open port")
if not p.setBaudRate(BAUDRATE):
    raise SystemExit("failed baud")
h = PacketHandler(PROTOCOL_END)


def safe2(sid, addr):
    try:
        v, comm, _ = h.read2ByteTxRx(p, sid, addr)
        return v if comm == COMM_SUCCESS else None
    except (IndexError, TypeError):
        return None


def signed_load(sid):
    v = safe2(sid, ADDR_PRESENT_LOAD)
    if v is None:
        return None
    return -(v & 0x3FF) if (v & 0x400) else (v & 0x3FF)


def goto(sid, t):
    h.write2ByteTxRx(p, sid, ADDR_GOAL_POSITION, int(round(t)))


def go_home():
    for i, t in HOME.items():
        h.write1ByteTxRx(p, i, ADDR_TORQUE_ENABLE, 1)
        goto(i, t)
    time.sleep(2.5)


# --- trajectory shapes, as functions of time -------------------------


def ramp(joint, amp, speed):
    """Out and back at a constant rate. Friction shows at the reversal."""
    dur = amp / speed

    def f(t):
        if t < dur:
            d = speed * t
        elif t < 2 * dur:
            d = amp - speed * (t - dur)
        else:
            return None
        return {joint: HOME[joint] + d}

    return f


def sine(joint, amp, period):
    """Smooth, continuously varying speed. Covers the band in one run."""

    def f(t):
        if t > 3 * period:
            return None
        return {joint: HOME[joint] + amp * np.sin(2 * np.pi * t / period)}

    return f


def chirp(joint, amp, t_end):
    """Slow to fast, amplitude falling to hold speed inside the band."""

    def f(t):
        if t > t_end:
            return None
        freq = 0.2 + 0.5 * (t / t_end)  # Hz, 0.2 -> 0.7
        a = amp * (0.2 / freq)  # shrink as freq rises
        return {joint: HOME[joint] + a * np.sin(2 * np.pi * freq * t)}

    return f


def multi_sine(joints, amp, period):
    """Several joints at once -- coupled inertia and gravity."""

    def f(t):
        if t > 3 * period:
            return None
        return {
            j: HOME[j] + amp * np.sin(2 * np.pi * t / period + k)
            for k, j in enumerate(joints)
        }

    return f


def hold_steps(joint, steps, hold_s):
    """Step, hold, step. Settling and stick-slip."""

    def f(t):
        k = int(t / hold_s)
        if k >= len(steps):
            return None
        return {joint: HOME[joint] + steps[k]}

    return f


# --- the set ---------------------------------------------------------

FIT = [
    ("fit_pan_ramp_slow", ramp(1, 400, 100)),
    ("fit_pan_ramp_med", ramp(1, 400, 300)),
    ("fit_pan_sine", sine(1, 300, 4.0)),
    ("fit_lift_sine", sine(2, 250, 4.0)),
    ("fit_lift_steps", hold_steps(2, [0, 40, 0, -40, 0], 2.5)),
    ("fit_pan_ramp_fast", ramp(1, 400, 500)),
]

VALIDATE = [
    ("val_pan_chirp", chirp(1, 300, 15.0)),
    ("val_elbow_sine", sine(3, 200, 4.5)),
    ("val_multi", multi_sine([1, 2, 3], 120, 7.0)),
]


def record(name, traj):
    go_home()
    rows = []
    t0 = time.perf_counter()
    nxt = t0
    fails = 0
    while True:
        t = time.perf_counter() - t0
        cmd = traj(t)
        if cmd is None:
            break
        for sid, target in cmd.items():
            goto(sid, target)
        for sid in cmd:
            pv, lv = safe2(sid, ADDR_PRESENT_POSITION), signed_load(sid)
            if pv is None or lv is None:
                fails += 1
            else:
                rows.append([t * 1000.0, sid, cmd[sid], pv, lv])
        nxt += DT
        while time.perf_counter() < nxt:
            pass

    path = f"{OUT_DIR}/{name}.csv"
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ms", "servo_id", "commanded", "measured", "load"])
        w.writerows(rows)
    lag = np.abs(np.array([r[2] - r[3] for r in rows]))
    print(
        f"  {name:<22} {len(rows):6d} rows  {fails:3d} fails  "
        f"max lag {lag.max():5.0f}"
    )


import os

os.makedirs(OUT_DIR, exist_ok=True)

input("Swing clear? arm unloaded (phone out)? [enter] ")
for i in OFFSETS:
    h.write1ByteTxRx(p, i, ADDR_TORQUE_ENABLE, 1)

print("\nFIT SET — these are what the parameters get tuned against")
for name, traj in FIT:
    record(name, traj)

print("\nVALIDATION SET — never fit against these")
for name, traj in VALIDATE:
    record(name, traj)

go_home()
p.closePort()
print(f"\nwrote to {OUT_DIR}/")
print("Watch max lag. Anything over ~50 ticks means that run went into")
print("saturation and teaches the fit nothing about kp.")
