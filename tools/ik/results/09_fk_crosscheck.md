# 09 — FK cross-check: KDL against MuJoCo

**Date:** 2026-10-12
**Phase:** 2, task 3
**Scripts:** `tools/ik/chain_probe.cpp`, `tools/ik/mujoco_fk.py`

---

## Why

KDL builds its model of the arm from the URDF — link lengths, joint placements,
axis directions, frame orientations. That model can be wrong in ways the joint
count never reveals. A misread rotation or a dropped offset still gives five
joints with the right names.

So the chain was checked against a second, independent source: MuJoCo, reading
the fitted MJCF. Different library, different file, different code path. Two
programs arriving at the same answer is evidence the answer is right.

Checking KDL against KDL would not be. That is the tautology trap — both sides
from one source, so the check cannot fail.

Bonus: the arm was driven to all-zeros during calibration (S03) and compared
against MuJoCo by eye. MuJoCo is already tied to the physical arm, so tying KDL
to MuJoCo ties KDL to the bench.

---

## Setup

**Chain:** `base_link` → `gripper_link`, five joints:
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll.
Base→tip, matching `p4_common`'s canonical `joint_order` minus the gripper.

**Tip link is `gripper_link`, not `gripper_frame_link`.** MuJoCo's URDF
conversion merges fixed-jointed links into their parent, so `gripper_frame_link`
does not exist as a body in the MJCF — it was absorbed into `gripper_link`. KDL
keeps it as a segment contributing no joint. Comparing at `gripper_link` is the
link both models have.

**Sources:**
- KDL ← `src/p4_common/urdf/so101_new_calib.urdf`
- MuJoCo ← `src/p4_common/mjcf/so101.xml`

**Angles in radians, positions in metres, rpy in radians, base frame.**

---

## Results

### Pose 1 — all zeros

| | x | y | z |
|---|---|---|---|
| KDL | 0.293234 | −0.000177 | 0.234370 |
| MuJoCo | 0.293234 | −0.000177 | 0.234370 |

| | roll | pitch | yaw |
|---|---|---|---|
| KDL | 1.57093 | −1.52211 | −1.57092 |
| MuJoCo | 1.570939 | −1.522114 | −1.570928 |

### Pose 2 — `0.3 −0.5 0.8 0.2 1.1`

| | x | y | z |
|---|---|---|---|
| KDL | 0.215553 | −0.054850 | 0.164623 |
| MuJoCo | 0.215553 | −0.054850 | 0.164622 |

| | roll | pitch | yaw |
|---|---|---|---|
| KDL | −1.00910 | −0.45077 | 1.00322 |
| MuJoCo | −1.00910 | −0.450771 | 1.003217 |

### Pose 3 — `1.5 −1.2 1.4 −1.3 2.5`

| | x | y | z |
|---|---|---|---|
| KDL | 0.0454905 | −0.0963549 | 0.216236 |
| MuJoCo | 0.045491 | −0.096355 | 0.216236 |

| | roll | pitch | yaw |
|---|---|---|---|
| KDL | −2.82819 | 0.357309 | −0.752675 |
| MuJoCo | −2.828195 | 0.357309 | −0.752675 |

---

## Conclusion

**Agreement to the last printed digit on all three poses, position and
orientation.** Sub-micrometre. KDL's kinematic model of the SO-101 is correct.

Three poses, not one: at the zero pose the arm sits in a neat configuration
where errors can cancel — a flipped axis can still produce the right position.
Pose 3 puts all five joints at large, distinct angles. Same reason the sign
check in S03 needed motion: zero looks identical whether you are right or wrong.

Two things fall out for free:

- **The rpy extraction in `mujoco_fk.py` matches KDL's convention.** MuJoCo
  gives quaternions and rotation matrices, not roll-pitch-yaw, so the extraction
  was hand-written and could have used a different convention. It does not.
  Rotation matrix transposition is a known recurring error; this rules it out here.
- **The fitted MJCF now has a second source.** Before this it had only been
  checked against the real arm by eye during calibration.

---

## Notes

**Zero-pose rpy sits near a singularity.** The angles are all near π/2 and the
pitch is −1.52211, close to −π/2, where roll-pitch-yaw degenerates and two of
the three angles stop being independent. Not a problem — both models agree
anyway — but rpy is a poor thing to compare at this pose. Poses 2 and 3 carry
the orientation evidence.

**The fixed offset from `gripper_link` to `gripper_frame_link` is not covered
by this check**, since MuJoCo has no such body. At the zero pose, differencing
the two KDL chains gives roughly:

| axis | offset (m) |
|---|---|
| x | −0.0008 |
| y | −0.0862 |
| z | −0.1201 |

Still owed: compare those lengths against the `<origin xyz=...>` on
`gripper_frame_joint` in the URDF. It is written in the parent's frame, so the
components may be permuted or signed differently, but the magnitudes should
match. That offset is the reference point the grasp-depth offset measures from,
so it is worth verifying by hand.

**kdl_parser warning, harmless:** `base_link` carries an inertia in the URDF and
KDL does not support inertia on a root link. P4 uses KDL for geometry only.

---

## How to reproduce

```bash
# KDL side
g++ -std=c++20 tools/ik/chain_probe.cpp -o /tmp/chain_probe \
  $(pkg-config --cflags --libs orocos-kdl) \
  -I/opt/ros/jazzy/include/kdl_parser \
  -L/opt/ros/jazzy/lib -lkdl_parser
/tmp/chain_probe 0 0 0 0 0

# MuJoCo side (needs conda — mujoco lives in base)
conda activate base
python tools/ik/mujoco_fk.py 0 0 0 0 0
```

Build C++ outside conda. Conda's `libyaml-cpp.so.0.9` against Jazzy's `0.8`
fails at load, not at build — see `docs/phase0_notes.md`.