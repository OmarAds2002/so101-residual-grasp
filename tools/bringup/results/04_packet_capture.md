# Feetech packet capture

Raw bytes on /dev/ttyACM0 at 1000000 baud, servo IDs [1, 2, 3, 4, 5, 6].
Captured from the vendor Python SDK. TX = host to bus, RX = bus to host.
Present position address 56, goal position address 42, 2 bytes each.

Positions read: {1: 1885, 2: 2323, 3: 2107, 4: 2740, 5: 3008, 6: 2041}


## ping_id1

TX  ff ff 01 02 01 fb
RX  ff ff 01 02 00 fc
TX  ff ff 01 04 02 03 02 f3
RX  ff ff 01 04 00 09
RX  03 ee

## sync_read_positions

TX  ff ff fe 0a 82 38 02 01 02 03 04 05 06 26
RX  ff ff 01 04 00 5d
RX  07 96
RX  ff ff 02 04 00 13
RX  09 dd
RX  ff ff 03 04 00 3b
RX  08 b5
RX  ff ff 04 04 00 b4
RX  0a 39
RX  ff ff 05 04 00 c0
RX  0b 2b
RX  ff ff 06 04 00 f9
RX  07 f5

## sync_write_goals

TX  ff ff fe 16 83 2a 02 01 5d 07 02 13 09 03 3b 08 04 b4 0a 05 c0 0b 06 f9 07 db
