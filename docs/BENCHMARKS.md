# Deterministic controller benchmark

The benchmark is a regression fixture, not a hardware-performance claim. It runs both controllers against the same static obstacle map, fixed 20 ms integration step, differential-drive kinematics, and odometry model.

The v0.2 development slice plans on a one-cell inflated occupancy grid before smoothing. The original occupancy map remains the collision oracle during simulation.

## Reference result — 2026-09-09

| Controller | Waypoints | Steps | Travelled | Max cross-track | Final error | Collision-free | Goal reached |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| PID waypoint baseline | 26 | 545 | 7.158981 m | 0.063708 m | 0.099468 m | yes | yes |
| Pure pursuit + smoothed path | 5 | 479 | 6.905655 m | 0.052827 m | 0.097605 m | yes | yes |

## Interpretation

The new pipeline reduces the global waypoint count from 26 to 5 while preserving collision-free goal completion. On this deterministic reference fixture, the interpolated-lookahead pure-pursuit controller also improves the measured settling steps, travelled distance, maximum cross-track error, and final goal error relative to the PID waypoint baseline.

This is a regression result for the checked-in fixture, not a universal controller-performance claim. The PID follower remains available as the baseline, and future fixtures must preserve the collision-clearance contract rather than trading safety for a visually smoother path.

## Acceptance contract

The benchmark exits non-zero if either controller fails to reach the goal or enters an occupied cell. CI executes the benchmark through CTest so future controller changes cannot silently regress those invariants.
