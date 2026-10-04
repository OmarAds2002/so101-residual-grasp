# Phase 0 — traps worth not rediscovering

Things that cost time once. Read before debugging a build or a launch.

---

## Build environment

**Build P4 outside conda.** Conda ships `libyaml-cpp.so.0.9`; ROS Jazzy and the
system ship `0.8`. Building with conda active bakes `0.9` into the binary, and at
runtime the loader only searches the system path, where `0.9` does not exist. The
failure looks like a pluginlib problem:

```
Failed to load library libp4_hardware.so ...
dlopen error: libyaml-cpp.so.0.9: cannot open shared object file
```

That message names the plugin macro and the XML. Both were fine. Read past the
first sentence to the `dlopen error`.

`conda deactivate` does **not** fix an already-built binary — the library name is
compiled in. Rebuild clean:

```bash
conda deactivate
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build
```

Stop it recurring:

```bash
conda config --set auto_activate_base false
```

Same shape as the `dialout` group problem during bring-up: the environment looked
one way, the process saw another.

**Diagnosing this class of failure:** `ldd <lib>.so | grep <name>` prints
`not found` next to whatever the loader cannot resolve. One command, definite answer.

---

## Build errors vs load errors

Three different failures, three different fixes. Getting the category right is most
of the work.

| Message | Layer | Fix |
|---|---|---|
| "No such file or directory" on an `#include` | compiler | include path |
| "Undefined reference" | linker | `target_link_libraries` |
| "dlopen error: lib... not found" | runtime loader | library on the runtime path |

A build succeeding says nothing about whether the thing will load.

---

## CMake

- **A library target can only be declared once.** `add_library(p4_common ...)` inside
  `if(BUILD_TESTING)` *and* above it gives "another target with the same name already
  exists".
- **The library block must sit above `if(BUILD_TESTING)`.** Inside it, a build with
  testing off produces no library and `install(TARGETS ...)` fails.
- **`SHARED` is required for anything pluginlib loads**, and for anything a shared
  library links. A static `.a` built without `-fPIC` cannot be folded into a `.so`:
  *"relocation ... can not be used when making a shared object; recompile with -fPIC"*.
  Making the dependency `SHARED` is the cleaner fix than setting
  `POSITION_INDEPENDENT_CODE`.
- **Install what runtime reads.** Tests use build-time source paths
  (`CALIBRATION_PATH`, `URDF_PATH` via `target_compile_definitions`). Nodes use the
  installed share directory via `$(find <pkg>)`. Both `config/` and `urdf/` need
  `install(DIRECTORY ...)` or the launch fails with a missing file.
- Use `yaml-cpp::yaml-cpp`, not bare `yaml-cpp` (removed in 0.10.0).

---

## pluginlib

Three names, all different, all easy to swap:

| Field | Form | Example |
|---|---|---|
| `path` in plugins XML | library name, no `lib`/`.so` | `p4_hardware` |
| `name` in plugins XML | **slash** | `p4_hardware/StubSystem` |
| `type` in plugins XML | **colons** | `p4_hardware::StubSystem` |

Plus two things that fail silently if missing:

- `PLUGINLIB_EXPORT_CLASS(...)` at the bottom of the `.cpp`, outside the namespace
- `<hardware_interface plugin="${prefix}/p4_hardware_plugins.xml"/>` in
  `package.xml`'s `<export>` block

---

## Checking the API instead of guessing

`on_init(const HardwareInfo &)` is deprecated in Jazzy. The current form is
`on_init(const HardwareComponentInterfaceParams &)`, where the struct holds
`hardware_info` and a weak pointer to the executor.

Found in ninety seconds by reading
`/opt/ros/jazzy/include/hardware_interface/hardware_interface/hardware_component_interface.hpp`.
The installed headers are the authority for the installed distro — they cannot be
out of date the way a search result can.

Pulling `info` out once keeps the rest of the function unchanged:

```cpp
const hardware_interface::HardwareInfo & info = params.hardware_info;
```

---

## `/joint_states` is alphabetical

`joint_state_broadcaster` publishes names sorted alphabetically, not in
`joint_order`. Harmless in the message itself — each name is paired with its value —
but anything reading `position[0]` and assuming `shoulder_pan` gets `elbow_flex`.

**Match by name, never by index, when consuming `/joint_states`.** Live version of
the G1 joint-ordering bug.

---

## The seam, restated

Radians and memory addresses above `SystemInterface`. Ticks and servo register
addresses below it.

Two things share the word "address" and nothing else:

- **Servo register address** — `56` present position, `42` goal position. Feetech
  control table. Inside the serial backend only.
- **Memory address** — where `positions_[0]` lives in RAM. What
  `export_state_interfaces` reports.

The stub has the second and none of the first. Every backend allocates arrays in
`on_init` and reports their addresses identically; `read()` and `write()` are the
*only* functions that differ between backends.

Exported handles are raw pointers the controller keeps. **Never reallocate the
arrays after exporting** — fixed-size, sized once in `on_init`.