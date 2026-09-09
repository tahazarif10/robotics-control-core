# robotics-control-core

[![CI](https://github.com/tahazarif10/robotics-control-core/actions/workflows/ci.yml/badge.svg)](https://github.com/tahazarif10/robotics-control-core/actions/workflows/ci.yml)
[![CodeQL](https://github.com/tahazarif10/robotics-control-core/actions/workflows/codeql.yml/badge.svg)](https://github.com/tahazarif10/robotics-control-core/actions/workflows/codeql.yml)
![C++20](https://img.shields.io/badge/C%2B%2B-20-blue)
![CMake](https://img.shields.io/badge/build-CMake-informational)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

**A deterministic C++20 navigation and control core for a differential-drive mobile robot.**

This repository is intentionally middleware-independent: it demonstrates the robotics algorithms and engineering discipline underneath a future ROS 2 application rather than hiding them inside framework callbacks.

## What it proves

- Modern **C++20** and explicit API contracts
- **A\*** path planning on an occupancy grid
- **PID** heading control with bounded integral/output
- Differential-drive **forward/inverse kinematics**
- **Odometry** integration in SE(2)
- Deterministic waypoint tracking simulation
- **CMake**, CTest, Ninja presets, warnings-as-errors
- Installable CMake package with the reusable `robotics::control` target
- GCC + Clang + MSVC CI
- **ASan + UBSan** regression lane
- Dockerized reproducible build
- CodeQL security scanning and Dependabot-managed GitHub Actions
- Tag-driven Linux/Windows release packaging
- Architecture and engineering documentation

## Demo

The executable plans around obstacles, converts the grid path to metric waypoints, and closes the loop through a differential-drive controller and odometry model.

```text
Start
  ↓
Occupancy Grid ── A* ──> Metric Path
                         ↓
Current Pose ──> Path Follower ──> v, ω
                                 ↓
                        Wheel Kinematics
                                 ↓
                             Odometry
                                 ↓
                               Goal
```

After building:

```bash
./build/dev/robotics_control_demo trajectory.csv planned_path.csv map.csv
python3 scripts/plot_trajectory.py trajectory.csv planned_path.csv map.csv --output trajectory.png
```

Expected success criteria are executable, not descriptive: the demo exits `0` only when the robot reaches the final waypoint within the configured tolerance and fixed simulation budget.

A validated reference run of the checked-in scenario produced:

| Signal | Result |
| --- | ---: |
| Planned cells | 21 |
| A* path cost | 6.24264 m |
| Expanded nodes | 84 |
| Simulation steps @ 20 ms | 502 |
| Final goal error | 0.0985 m |
| Acceptance | Goal reached / exit 0 |

These are regression outputs for the deterministic scenario, not hardware performance claims.

![Deterministic trajectory demo](docs/assets/demo_trajectory.svg)

## Build

Requirements: CMake 3.24+, a C++20 compiler, and Ninja for presets.

```bash
cmake --preset dev
cmake --build --preset dev --parallel
ctest --preset dev
```

Sanitizers:

```bash
cmake --preset asan
cmake --build --preset asan --parallel
ctest --preset asan
```

## Repository layout

```text
include/robotics_control/   public C++ contracts
src/                        algorithm implementations
app/                        deterministic end-to-end demo
tests/                      dependency-free contract tests
scripts/                    visualization tooling
docs/                       architecture and engineering notes
.github/workflows/          multi-compiler CI
```

## Design decisions

- The core has **no ROS 2 dependency** so planner/controller logic can be unit-tested independently and reused later from `rclcpp` nodes.
- Grid A* rejects diagonal corner cutting rather than letting a point robot pass through touching obstacles.
- All control limits are explicit configuration, not magic hidden clamps.
- The simulation uses a fixed step to keep regression behavior deterministic.
- Public names encode SI units where ambiguity is likely.

See [Architecture](docs/ARCHITECTURE.md), [Engineering contract](docs/ENGINEERING.md), [Verification record](docs/VERIFICATION.md), and [Changelog](CHANGELOG.md).

## Consume as a package

After installation, another CMake project can use the exported target directly:

```cmake
find_package(robotics_control_core 0.1 CONFIG REQUIRED)
target_link_libraries(my_robot PRIVATE robotics::control)
```

A standalone smoke consumer lives in [`examples/consumer`](examples/consumer) and is rebuilt against the installed package in CI. This verifies the install/export contract rather than only the in-tree build.

## Roadmap

- **v0.1** — A*, PID, differential-drive kinematics, odometry, deterministic simulation, CI.
- **v0.2** — path smoothing, pure-pursuit controller, benchmark fixtures, JSON map input.
- **v0.3** — covariance-aware localization/filter module and replay fixtures.
- **v1.0** — freeze the core API after integration into `ros2-autonomous-mobile-robot`.

## Safety

This is portfolio/research software, not a certified safety controller. See [SECURITY.md](SECURITY.md).

## License

MIT © 2026 Taha Zarif
