# AGENTS.md — Meister Development Rules (Index)

> Target repository: `sosukigara/2026-Meister`
> Every rule below is grounded in a real mistake made during development on 2026-09-27.
> **This file is the primary index (keep within 200–300 lines). Details live in `docs/`.**
> Code map: [docs/code-map.md](docs/code-map.md). Docs entry point: [docs/README.md](docs/README.md).

---

## 1. Project Purpose and Roles

Robot control software for an autonomous mobile robot based on ROS 2 (Jazzy) + Gazebo.
Simulation (dev) and the real machine (planned) are different stages of the same robot;
simulation serves as the preliminary stage for real-machine development.

- Simulation stage (implemented): mapping with SLAM, autonomous navigation with Nav2,
  teleoperation via Web UI (features/01-06)
- Real machine (planned): 6-wheel rocker-bogie + ESP32 + 4-axis arm + LD-D500 LiDAR.
  Aiming at automatic grasping and automatic control via image recognition (features/07-10)

The MCU layer (ESP32 firmware) is `firmware/main_board/` (PlatformIO + Arduino).
`firmware/foc/` (single-channel winch) and `firmware/foc_drive/` (rover axle roles)
are separate projects (MKS ESP32 FOC V2.0).

Goals: G1 auto-drive / G2 arm retrieval / G3 manual operation ([docs/やりたいこと.md](docs/やりたいこと.md)).

## 2. Tech Stack

| Layer | Technology |
|---|---|
| OS (PC) | Ubuntu |
| Middleware | ROS 2 Jazzy |
| Simulator | Gazebo (ros_gz) |
| Build (ROS) | colcon (workspace root: `ros/`) |
| Build (FW) | PlatformIO |
| Languages | Python 3 / C++17 / YAML |
| Detection | ONNX Runtime + YOLOv8n |
| FW framework | Arduino (both arduino-esp32 2.x/3.x supported) |
| Tests (C++) | Unity (`pio test -e native`) |
| Tests (Python) | pytest |

Details: [docs/coding-standards.md](docs/coding-standards.md) §1-2.

## 3. Verification and Command Summary

Use `firmware/main_board/` as cwd for pio commands, `ros/` as cwd for ros2 commands (R6).

```bash
# Build
./build.sh
cd firmware/main_board && pio run -e esp32dev -e esp32dev_usbuart
# Tests
cd firmware/main_board && pio test -e native
cd ros && python3 -m pytest src/meister_serial_bridge/test/
# Real-machine verification
ros2 run meister_serial_bridge meister_comm_check --port /dev/ttyUSB0
ros2 run meister_serial_bridge meister_hz_measure --port /dev/ttyUSB0 --expect-hz 100
```

Do not commit until the real-machine `comm_check` passes all 3 steps. Run all 4 builds.
Full version: [docs/commands.md](docs/commands.md).

## 4. Absolute Rules (NG List)

1. **One file, one responsibility; consider splitting beyond 300 lines** (R1) → [coding-standards §3](docs/coding-standards.md)
2. **Define each setting in exactly one place** (R2) → [coding-standards §4](docs/coding-standards.md)
3. **Physical constants live in code, in tables with sources** (R3) → [coding-standards §5](docs/coding-standards.md)
4. **Never fill in unmeasured values from memory; isolate and gate them** (R4) → [pitfalls P1](docs/pitfalls.md)
5. **After an edit rejection, verify what was applied via file:line** (R5) → [pitfalls P2](docs/pitfalls.md)
6. **One review pass before every push** (R6) → [pitfalls P3](docs/pitfalls.md)
7. **Consolidate breaking changes into one config location before landing** (R7) → [pitfalls P4](docs/pitfalls.md)
8. **Layer dependencies are one-directional** (R8) → [architecture §2](docs/architecture.md) / [coding-standards §8](docs/coding-standards.md)
9. **Comments explain WHY only** (R9) → [coding-standards §7](docs/coding-standards.md)
10. **Never close unverified work as "no problem"** (R10) → [pitfalls P6](docs/pitfalls.md)
11. **Never rely on static initialization order** (R11) → [pitfalls P5](docs/pitfalls.md)
12. **Split commits into reviewable units** (R12) → [pitfalls P7](docs/pitfalls.md)

Additional NGs:
- Avoid abusing `Any` (add type hints)
- `hal/` must not know its upper layers. Never call into `hal/*` internals from above
- `ros/src/` must not reference `MSTE_*` macros directly
- When changing the protocol axis count, fix C++ and Python together
- Never break what already works in order to move forward

## 5. Pointers to Docs

| File | Contents |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Overall design, module responsibilities, data flow, design patterns |
| [docs/coding-standards.md](docs/coding-standards.md) | Naming, prohibitions, libraries, comments, test placement |
| [docs/commands.md](docs/commands.md) | Run / test / verification commands |
| [docs/pitfalls.md](docs/pitfalls.md) | Traps AI has fallen into before, project-specific constraints |
| [docs/task-template.md](docs/task-template.md) | Spec / task definition template |
| [docs/code-map.md](docs/code-map.md) | File-level code map |
| [docs/README.md](docs/README.md) | Docs entry point (4 layers) |

## 6. Tacit Knowledge (Excerpt — Not Obvious from Code Alone)

- Protocol: `protocol.py` (Python) and `meister_protocol.h` (C++) must match **byte for byte**. Fix both when changing the axis count.
- Timing: PC→ESP32 on arrival, ESP32 applies at 100 Hz, FB_STATE at 100 Hz, STS read-back round-robin at 30–40 ms.
- Unverified values: `MSTE_BUS_RESP_TIMEOUT_US=3000µs`, `MSTE_BUS_INTER_FRAME_GAP_US=200µs`, `reg::*` addresses, `kStepsPerRev=4096` are not datasheet-checked. Do not guess them.
- Web search is blocked for all providers. pip is available.
- GPIO12 is a strapping pin (gripper PWM). Change it when wiring the real machine.
- Specify `upload_port` by-id (prevents misflashing the M5Stack).
- `ROS_LOCALHOST_ONLY=1` in `start_meister.sh` applies to child processes only.

## 7. Review Verdict Vocabulary (R10)

- **Confirmed from a primary source** (can name the source file and line)
- **Measured** (measurement conditions and values exist)
- **Needs confirmation** (not yet obtained. Write `★要確認★` plus where it is isolated)
- **✗ Unverified** (not yet looked at. Never write "no problem")
