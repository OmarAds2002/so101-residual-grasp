# Serial latency — the number every TBD hangs off

Date: 2026-09-20
Port /dev/ttyACM0, 1000000 baud, six STS3215, CH343 adapter.
10,000 samples each after 100 warm-up. Nothing moving (sync write
commands each servo to the position it already holds).
Raw samples in latency_raw.csv.

## Sync read — 6 present positions, 2 bytes each
| min | p50 | p90 | p99 | p99.9 | max |
|---|---|---|---|---|---|
| 0.847 | 0.869 | 0.904 | 0.945 | 0.961 | 1.027 |
0 failures in 10,000.

## Sync write — 6 goal positions, 2 bytes each (no reply)
| min | p50 | p90 | p99 | p99.9 | max |
|---|---|---|---|---|---|
| 0.074 | 0.246 | 0.291 | 0.306 | 0.447 | 1.594 |
0 failures in 10,000.

## Conclusions
- read + write at p99 = 1.25 ms  ->  ~800 Hz bus ceiling
- worst case both peaks same cycle = 2.62 ms, still inside a 5 ms period
- **200 Hz is safe. The bus is not the constraint I assumed it was.**
- read is 3.5x the write: one packet out, six replies back on a
  half-duplex line. The turnaround cost is visible in the data.
- write tail (p99.9 0.447, max 1.594) is ~5x its median with no
  matching read tail -> scheduler or USB stack, not the servos.
  Noted, not chased.

## Caveat
Measured in a tight Python loop with an idle machine. The real loop
runs IK and ONNX between transactions. Bus cost won't change;
scheduling jitter might. This is a ceiling on the bus, not a promise
about the system.

## Updates to the architecture
- Control rate TBD: resolved as far as the bus goes. Pick 200 Hz.
- Sim control decimation: set from 200 Hz.
- Delay buffers: size from ~1.25 ms, not a guess.
- Jitter injection: use the measured distribution, not a made-up one.
