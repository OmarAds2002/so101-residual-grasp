# Bus scan — first contact with the arm

Date: 2026-09-20

## Setup
- Arm: SO-ARM101 Pro, clamped, external power connected
- Adapter chip: WCH CH343 (idVendor=1a86, idProduct=55d3)
- Presents as CDC-ACM, so the port is /dev/ttyACM0, not /dev/ttyUSB*
- No `latency_timer` file exists for ACM devices — the FTDI trick does not apply here
- User must be in the `dialout` group; needs a full logout/reboot to take effect

## Serial settings that work
- Port: /dev/ttyACM0
- Baud: 1000000
- protocol_end = 0  (STS/SMS family)
- SDK: feetech-servo-sdk (pip), `sms_sts` side

## Ping results
All six servos answer.

| ID | Model number |
|----|--------------|
| 1  | 777 |
| 2  | 777 |
| 3  | 777 |
| 4  | 777 |
| 5  | 777 |
| 6  | 777 |

Model 777 = STS3215.

## What this does NOT tell me
Which physical joint each ID drives. IDs answering 1-6 does not mean
ID 1 is shoulder_pan. That needs motion and comes next.

## Open
- Latency distribution — not measured yet
- ID -> joint map — not measured yet
- sign and offset per joint — not measured yet
