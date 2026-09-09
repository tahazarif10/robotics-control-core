# Deterministic controller benchmark

The benchmark is a regression fixture, not a hardware-performance claim. It runs both controllers against the same static obstacle map, fixed 20 ms integration step, differential-drive kinematics, and odometry model.

The v0.2 development slice plans on a one-cell inflated occupancy grid before smoothing. The original occupancy map remains the collision oracle during simulation.

## Reference result — 2026-09-09

| Controller | Waypoints | Steps | Travelled | Max cross-track | Final error | Collision-free | Goal reached |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| PID waypoint baseline | 26 | 545 | 7.158981 m | 0.063708 m | 0.099468 m | yes | yes |
| Pure pursuit + smoothed path | 5 | 646 | 7.286272 m | 0.679686 m | 0.109821 m | yes | yes |

## Interpretation

The new pipeline reduces the global waypoint count from 26 to 5 while preserving collision-free goal completion. It does **not** yet outperform the existing PID waypoint baseline on tracking error, travel distance, or settling steps. Pure pursuit therefore remains an experimental v0.2 controller rather than replacing the baseline.

That outcome is intentional evidence: the next tuning work should improve geometric tracking without weakening the collision-clearance contract or hiding regressions behind a visually smoother path.

## Acceptance contract

The benchmark exits non-zero if either controller fails to reach the goal or enters an occupied cell. CI executes the benchmark through CTest so future controller changes cannot silently regress those invariants.
