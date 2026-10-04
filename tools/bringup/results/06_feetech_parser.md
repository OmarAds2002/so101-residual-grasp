# 06 — Feetech wire protocol

Decoded from real bytes captured off `/dev/ttyACM0` with the vendor Python SDK.
Raw capture: `04_packet_capture.md`. Implementation:
`src/p4_hardware/include/p4_hardware/feetech_protocol.hpp`.

---

## Packet layout

Every packet, both directions:

```
ff ff  <id>  <length>  <instruction / error>  <params...>  <checksum>
```

- **Header** is `ff ff`. Not included in the checksum.
- **ID** is the servo, 1–253. `fe` (254) is broadcast — every servo listens,
  none replies.
- **Length** counts the instruction byte, the parameters, and the checksum.
  It does **not** count the header or the ID.
- Byte 4 is the **instruction** in a command and the **error byte** in a reply.

## Checksum

Sum every byte from the ID through the last parameter. Keep the low 8 bits.
Invert.

```
ping: 01 + 02 + 01 = 0x04  ->  ~0x04 = 0xfb
```

The low-8-bits part is load-bearing. Summing into an `int` and inverting gives a
32-bit result with leading ones. Accumulate into a `uint8_t` so it wraps, or mask
with `& 0xFF` before inverting.

Catches accidental bit flips on the wire. Weak against deliberate tampering,
which does not matter for a servo on a desk.

## Instruction codes

| Code | Instruction |
|---|---|
| `0x01` | ping |
| `0x02` | read |
| `0x03` | write |
| `0x82` | sync read |
| `0x83` | sync write |

## Register addresses (STS3215)

| Address | Register |
|---|---|
| 40 (`0x28`) | torque enable |
| 42 (`0x2a`) | goal position |
| 56 (`0x38`) | present position |
| 58 | present speed |

## Multi-byte values are little-endian, low byte first

```
01 5c 07   ->  id 1, goal 0x075c = 1884
```

Split: low = `v & 0xFF`, high = `v >> 8`. Recombine: `low | (high << 8)`.

Reversing the order turns 1884 into 23559 — off the end of the tick range, so a
joint would drive to a limit. This is what `protocol_end = 0` selects in the
vendor SDK (STS/SMS family); the SCS family is the other way round.

---

## The three packets in use

**Ping** — 6 bytes, fixed.

```
ff ff 01 02 01 fb
```

**Sync read** — one packet out, one reply per servo back.

```
 0  1  2  3  4  5  6 |  7  8  9 10 11 12 | 13
ff ff fe 0a 82 38 02 | 01 02 03 04 05 06 | 26
                  ^  bytes per servo
               ^  address (56, present position)
```

Length `0a` = 10 = instruction + address + bytes-per-servo + 6 IDs + checksum.

**Sync write** — no reply at all, which is why it measured 0.246 ms against the
read's 0.869 ms. No turnaround to pay for on a half-duplex line.

```
 0  1  2  3  4  5  6 |  7  8  9 | ... | 22 23 24 | 25
ff ff fe 16 83 2a 02 | id lo hi | ... | id lo hi | dd
```

Length `16` = 22 = instruction + address + bytes-per-servo + (6 x 3) + checksum.

## Reply

```
ff ff 01 04 00 5c 07 97
         ^  ^  ^^^^^  ^ checksum
         |  |  payload
         |  error byte
         length
```

`payload_size = length - 2` (length counts the error byte, the payload, and the
checksum).

**The error byte is status, not a problem.** Zero means healthy. Non-zero has
bits set for overload, overheating, voltage, angle-limit. Several can be set at
once — `0x05` is two faults, not fault five.

Extract it and hand it up. It is a second, independent signal of the same trouble
that tracking error catches, and the architecture wants both. Decoding the
individual bits can wait for the Phase 3 safety FSM.

---

## Replies fragment. Always.

Every single reply in the capture arrived across two `read()` calls:

```
RX  ff ff 01 04 00 5c
RX  07 97
```

So **a read is not a packet.** Anything that assumes one read gives one complete
reply is wrong, and would have looked fine against a hand-written fixture.

Hence `ResponseReader`:

- fixed `std::array` buffer plus a `count_`, no allocation, safe on the RT path
- `Append(span)` adds newly-read bytes
- `TryParseOne()` returns `Ok` / `Incomplete` / `BadChecksum` / `BufferOverflow`
- consumed bytes are removed from the front; the rest shuffle down

Design points worth not relitigating:

- **`Incomplete` is not an error.** More bytes are coming. Different from
  `BadChecksum`, which means something is actually broken, and the caller does
  something different for each.
- **No waiting inside the parser.** It answers about the bytes that exist right
  now. Blocking belongs in the port layer above it.
- **A bad reply is consumed anyway.** Otherwise it blocks the buffer forever and
  every later call fails on the same bytes.
- **Overflow clears the whole buffer and reports.** A full buffer means framing
  failed across far more bytes than the longest valid packet, so everything in
  there is untrustworthy. Salvaging is guessing. Don't repair, report — same rule
  as `RadToTicks` not clamping.
- **Scan for `ff ff`, don't assume index 0.** `ff` can appear inside a packet as
  data, so the header match is a candidate and the checksum is what confirms it.

---

## Why the capture exists

The parser is tested against bytes the **vendor SDK** put on the wire. Two sides
from genuinely different sources: their implementation and ours. Neither derives
from the other.

Writing the capture in C++ would have meant testing our packet builder against
our packet builder — the tautology trap with a serial cable attached.

It also means the tests need no arm, no port, and no Python. They run in CI
forever.

**Getting at the bytes:** the SDK exposes no raw-byte hook. Monkey-patch
`portHandler.writePort` and `portHandler.readPort` — save the originals, install
functions that log and then call them. Install the patch *before* any
transaction or it is not in the log.

**Retyping capture bytes into a test defeats the point.** Doing that produced
four wrong replies and a failing test that looked like a parser bug. Copy from
the file. Better: have the test read the capture file the way the calibration
test reads the URDF.