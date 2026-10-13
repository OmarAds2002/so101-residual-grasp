"""Capture raw Feetech bus bytes using the vendor SDK.

The C++ packet builder gets tested against these bytes. Two independent
sources: the vendor's implementation on one side, ours on the other.

Nothing moves -- goals are set to the positions just read.
"""

import os
from scservo_sdk import *

PORT = "/dev/ttyACM0"
BAUD = 1000000
IDS = [1, 2, 3, 4, 5, 6]

ADDR_PRESENT_POSITION = 56
ADDR_GOAL_POSITION = 42

# Path built from this script's location, not the shell's working directory.
HERE = os.path.dirname(os.path.abspath(__file__))
OUT_PATH = os.path.join(HERE, "results", "04_packet_capture.md")

port = PortHandler(PORT)
packet = PacketHandler(0)  # 0 = STS/SMS byte order

if not port.openPort():
    raise SystemExit("could not open " + PORT)
if not port.setBaudRate(BAUD):
    raise SystemExit("could not set baud rate")

# --- byte logging ---------------------------------------------------------
# Replace the port's read and write with versions that record, then call the
# originals. Every transaction below goes through these, so nothing else in
# the script needs to know about logging.

log = []  # list of (label, direction, bytes)
label = "setup"  # which transaction we're inside right now

original_write = port.writePort
original_read = port.readPort


def logged_write(data):
    log.append((label, "TX", bytes(data)))
    return original_write(data)


def logged_read(length):
    data = original_read(length)
    if data:
        log.append((label, "RX", bytes(data)))
    return data


port.writePort = logged_write
port.readPort = logged_read
# -------------------------------------------------------------------------

# 1. Ping ID 1. Simplest packet there is.
label = "ping_id1"
model, result, error = packet.ping(port, 1)
if result != COMM_SUCCESS:
    print("ping failed:", packet.getTxRxResult(result))
else:
    print("ping ok, model", model)

# 2. Sync read all six present positions.
label = "sync_read_positions"
sync_read = GroupSyncRead(port, packet, ADDR_PRESENT_POSITION, 2)
for servo_id in IDS:
    if not sync_read.addParam(servo_id):
        raise SystemExit("addParam failed for id %d" % servo_id)

result = sync_read.txRxPacket()
if result != COMM_SUCCESS:
    print("sync read failed:", packet.getTxRxResult(result))

positions = {}
for servo_id in IDS:
    positions[servo_id] = sync_read.getData(servo_id, ADDR_PRESENT_POSITION, 2)
print("positions:", positions)

# 3. Sync write each servo to where it already is. Nothing moves.
label = "sync_write_goals"
sync_write = GroupSyncWrite(port, packet, ADDR_GOAL_POSITION, 2)
for servo_id in IDS:
    goal = positions[servo_id]
    param = [SCS_LOBYTE(goal), SCS_HIBYTE(goal)]
    if not sync_write.addParam(servo_id, param):
        raise SystemExit("addParam failed for id %d" % servo_id)

result = sync_write.txPacket()
if result != COMM_SUCCESS:
    print("sync write failed:", packet.getTxRxResult(result))

port.closePort()

# --- write the capture ----------------------------------------------------
lines = [
    "# Feetech packet capture",
    "",
    "Raw bytes on /dev/ttyACM0 at %d baud, servo IDs %s." % (BAUD, IDS),
    "Captured from the vendor Python SDK. TX = host to bus, RX = bus to host.",
    "Present position address %d, goal position address %d, 2 bytes each."
    % (ADDR_PRESENT_POSITION, ADDR_GOAL_POSITION),
    "",
    "Positions read: %s" % positions,
    "",
]

current = None
for entry_label, direction, data in log:
    if entry_label != current:
        current = entry_label
        lines.append("")
        lines.append("## %s" % entry_label)
        lines.append("")
    lines.append("%s  %s" % (direction, data.hex(" ")))

os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)
with open(OUT_PATH, "w") as f:
    f.write("\n".join(lines) + "\n")

print("wrote", OUT_PATH)
