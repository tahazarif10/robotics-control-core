# Changelog

All notable changes to this project are documented here.

The project follows Semantic Versioning once the public API reaches `v1.0.0`.
Before `v1.0.0`, minor releases may contain API changes when they are documented
here and reflected in the package version.

## [Unreleased]

### Added

- Deterministic occupancy-grid obstacle inflation for controller clearance.
- Collision-safe greedy grid-path smoothing with diagonal corner-cut rejection.
- Pure-pursuit controller with explicit lookahead, speed, curvature and goal limits.
- End-to-end collision-free pure-pursuit regression on the checked-in demo map.
- Comparative benchmark executable for the PID waypoint baseline and smoothed pure pursuit.

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
