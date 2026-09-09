# Changelog

All notable changes to this project are documented here.

The project follows Semantic Versioning once the public API reaches `v1.0.0`.
Before `v1.0.0`, minor releases may contain API changes when they are documented
here and reflected in the package version.

## [0.2.0] - 2026-09-09

### Added

- Deterministic occupancy-grid obstacle inflation for controller clearance.
- Collision-safe greedy grid-path smoothing with diagonal corner-cut rejection.
- Pure-pursuit controller with continuous polyline projection, interpolated lookahead,
  and explicit speed, curvature, and goal limits.
- End-to-end collision-free pure-pursuit regression on the checked-in demo map.
- Comparative benchmark executable for the PID waypoint baseline and smoothed pure pursuit.
- Separate v0.2 regression lane in CTest.

### Verified

On the checked-in deterministic benchmark fixture, the smoothed pure-pursuit pipeline
reduced the path from 26 to 5 waypoints and improved measured settling steps
(545 → 479), travelled distance (7.158981 m → 6.905655 m), maximum cross-track
error (0.063708 m → 0.052827 m), and final goal error
(0.099468 m → 0.097605 m), while both controllers remained collision-free and
reached the goal.

These numbers are regression evidence for the checked-in fixture, not universal
controller-performance claims.

## [0.1.0] - 2026-09-09

### Added

- Deterministic occupancy-grid A* path planner with diagonal corner-cut rejection.
- PID controller with explicit output and integral bounds.
- Differential-drive forward/inverse kinematics.
- SE(2) differential-drive odometry integration.
- Deterministic waypoint follower and end-to-end simulation demo.
- Dependency-free contract test executable.
- GCC, Clang, MSVC, ASan and UBSan CI configuration.
- Installable CMake package exporting `robotics::control`.
- Standalone package-consumer example using `find_package(robotics_control_core)`.
- Architecture, engineering, contribution and security documentation.
- CodeQL scanning and Dependabot configuration for repository automation.
