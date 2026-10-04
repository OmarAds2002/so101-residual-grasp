# Calibration — ID map, signs, offsets

Date: 2026-09-20

## ID -> joint (base to tip, matches canonical joint_order)
1 shoulder_pan, 2 shoulder_lift, 3 elbow_flex,
4 wrist_flex, 5 wrist_roll, 6 gripper

The IDs happen to line up with the array index. Do NOT rely on that
in code — the calibration file names the ID explicitly so a replaced
servo is a config edit, not a rebuild.

## Method
Torque off, joint moved by hand to both mechanical stops, ticks read
at each end. sign comes from which way ticks moved between the stops.
offset is the midpoint of the estimate from each stop.

## Cross-check that caught a real error
A first pass compared real-arm motion against MuJoCo by eye and got
all six signs as +1. The stop-measurement says four are -1. Driving
the arm to the calibrated zero pose and comparing to MuJoCo at
qpos=0 confirmed the stop-measurement. The by-eye pass was wrong,
most likely camera angle vs bench viewpoint.

Two genuinely different sources disagreeing is the only reason this
was catchable.

## Span check (measured vs URDF)
| joint | measured | URDF | diff |
|---|---|---|---|
| shoulder_pan  | 2728 | 2503 | +225 |
| shoulder_lift | 2394 | 2276 | +118 |
| elbow_flex    | 2200 | 2203 |   -3 |
| wrist_flex    | 2325 | 2162 | +163 |
| wrist_roll    | 3804 | 3641 | +163 |
| gripper       | 1476 | 1252 | +224 |

Five of six read WIDER than the URDF, never narrower. Either the URDF
limits are software margin inside the real stops, or hand pressure
over-travelled the plastic. Unresolved. elbow_flex agreeing to 3 ticks
says the method itself is sound.

## Zero-pose check
Drove all six to their offset tick value (= 0 rad by definition) and
compared to MuJoCo with all qpos at 0.

Five joints matched. elbow_flex sat slightly negative.

The URDF carries a comment on elbow_flex and no other joint:
"5-degree calibration offset applied to joint limits". 5 deg = 57
ticks. The limits were shifted without shifting the zero, so an
offset derived from those limits inherits the shift.

## Final values
See p4_ws/config/calibration.yaml

## Still open
- elbow_flex offset being tuned against the 5-degree note
- gripper offset least trustworthy: its "lower stop" is rubber pads,
  not a hard stop
- whether the wide spans are software margin or over-travel
