# 05 — Stub backend and full stack bring-up

Date: 2026-09-27
Phase 0, items 6 and 7.

---

## What was proved

The full `ros2_control` stack comes up with no arm attached, and a command sent on
a topic comes back out on `/joint_states` having passed through hand-written C++.

```
/position_controller/commands
  -> ForwardCommandController
  -> commands_[] (our memory, via exported CommandInterface)
  -> StubSystem::read()          echoes command into state
  -> positions_[] (our memory, via exported StateInterface)
  -> JointStateBroadcaster
  -> /joint_states
```

Commanded `shoulder_pan = 0.3`, read back `shoulder_pan = 0.3`.

---

## Configuration

- Plugin: `p4_hardware/StubSystem`, class `p4_hardware::StubSystem`
- Description: `p4_stub.urdf.xacro`, which includes the vendor
  `so101_new_calib.urdf` and adds a `<ros2_control>` block. **The vendor URDF is
  not edited.**
- Calibration path passed as a hardware `<param>`, resolved with
  `$(find p4_common)/config/calibration.yaml`
- Controllers: `joint_state_broadcaster`, and `forward_command_controller` named
  `position_controller`
- `update_rate: 200` — the number the Session 03 latency measurement earned

---

## Verified state

```
Hardware Component 1
	name: P4Stub
	plugin name: p4_hardware/StubSystem
	state: id=3 label=active
	read/write rate: 200 Hz
	command interfaces
		shoulder_pan/position [available] [claimed]
		shoulder_lift/position [available] [claimed]
		elbow_flex/position [available] [claimed]
		wrist_flex/position [available] [claimed]
		wrist_roll/position [available] [claimed]
		gripper/position [available] [claimed]

position_controller     forward_command_controller/...  active
joint_state_broadcaster joint_state_broadcaster/...     active
```

---

## Design decisions made here

| Decision | Reason |
|---|---|
| `read()` echoes the last command | Returning zeros makes tracking error — a first-class safety check — fire constantly against a robot that does not exist. Echoing makes the stub a perfect, instant servo, which is what proving the plumbing needs. Imperfection is the *delay-injecting* stub's job, later. |
| `write()` does nothing | `read()` already did the echo. No bus to write to. |
| `on_init` validates `info.joints` against `joint_order` by position | A URDF with five joints, or a misspelled name, fails at startup with a clear message instead of a wrong index at cycle 400. This is the G1 joint-ordering bug caught at the door. |
| `on_init` loads the calibration even though the stub never converts to ticks | Same code path as the serial backend, so a broken calibration file fails here — with no arm plugged in and no servo to blame. |
| Loader throws, `on_init` catches and returns `ERROR` | `controller_manager` expects a return code, not an exception. The translation belongs at this boundary. |

---

## Known, not chased

- **FIFO RT scheduling denied** — *"Could not enable FIFO RT scheduling policy:
  Operation not permitted"*. Needs realtime privileges for the user. Fine for a stub;
  must be fixed before any timing claim is made about the real loop.
- **`velocity` and `effort` publish as `nan`** — only position interfaces are
  exported. Open question for the serial backend: use the servo's reported speed
  (address 58, from a hidden inner loop with unknown filtering) or difference
  positions across cycles. Not decided.
- **KDL warning about inertia on `base_link`** — from the vendor URDF, cosmetic.

---

## Commands

```bash
conda deactivate
cd ~/p4_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select p4_common p4_hardware
source install/setup.bash
ros2 launch p4_hardware stub.launch.py
```

Second terminal:

```bash
source ~/p4_ws/install/setup.bash
ros2 control list_hardware_components
ros2 control list_controllers

ros2 topic pub --once /position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]}"
ros2 topic echo /joint_states --once
```

**Order matters** — publish first, then echo. Echoing first shows the startup zeros
and proves nothing.