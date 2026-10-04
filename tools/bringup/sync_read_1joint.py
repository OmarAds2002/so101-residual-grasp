#!/usr/bin/env python3
"""Measure serial round-trip timing on the SO-101 bus.

Times two operations, each the way the RT loop will actually do it:
  - sync read  of six present positions  (request out, six replies back)
  - sync write of six goal positions     (write only, no reply)

Dumps raw samples to CSV. Percentiles are computed from the CSV afterwards,
so a different statistic never means rerunning the measurement.
"""

import time
import csv
import numpy as np
from scservo_sdk import *

# --- control table addresses -------------------------------------------
ADDR_STS_GOAL_POSITION    = 42
ADDR_STS_PRESENT_POSITION = 56

# --- settings ----------------------------------------------------------
SCS_IDS      = [1]
BAUDRATE     = 1000000
DEVICENAME   = '/dev/ttyACM0'
PROTOCOL_END = 0          # STS/SMS

N_WARMUP     = 100        # thrown away
N_SAMPLES    = 10000

portHandler   = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_END)

if not portHandler.openPort():
    raise SystemExit("failed to open port")
if not portHandler.setBaudRate(BAUDRATE):
    raise SystemExit("failed to set baudrate")
print("port open at", BAUDRATE)


# =======================================================================
# sync read
# =======================================================================
groupSyncRead = GroupSyncRead(portHandler, packetHandler,
                              ADDR_STS_PRESENT_POSITION, 2)

for scs_id in SCS_IDS:
    if not groupSyncRead.addParam(scs_id):
        raise SystemExit(f"addParam failed for id {scs_id}")

read_samples  = []
read_failures = 0

for n in range(N_WARMUP + N_SAMPLES):
    t0 = time.perf_counter()
    comm_result = groupSyncRead.txRxPacket()
    t1 = time.perf_counter()

    if n < N_WARMUP:
        continue

    if comm_result != COMM_SUCCESS:
        read_failures += 1
        continue                      # a failed transaction is not a sample

    read_samples.append((t1 - t0) * 1000.0)

print(f"sync read: {len(read_samples)} samples, {read_failures} failures")



portHandler.closePort()


# =======================================================================
# output
# =======================================================================
with open("results/latency__sysid_raw.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["operation", "ms"])
    for v in read_samples:
        w.writerow(["sync_read", v])

def report(name, s, failures):
    a = np.array(s)
    print(f"\n{name}   ({len(a)} samples, {failures} failures)")
    print(f"  min    {a.min():8.3f} ms")
    print(f"  p50    {np.percentile(a, 50):8.3f} ms")
    print(f"  p90    {np.percentile(a, 90):8.3f} ms")
    print(f"  p99    {np.percentile(a, 99):8.3f} ms")
    print(f"  p99.9  {np.percentile(a, 99.9):8.3f} ms")
    print(f"  max    {a.max():8.3f} ms")

report("sync read ", read_samples,  read_failures)

rt = np.percentile(read_samples, 99)
print(f"\nread p99: {rt:.3f} ms  ->  {1000.0/rt:.0f} Hz ceiling")