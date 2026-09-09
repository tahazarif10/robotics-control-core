# Architecture

## Design intent

`robotics-control-core` is deliberately independent of ROS 2. The core owns deterministic planning and control logic; a later ROS 2 repository can wrap these contracts through `rclcpp` without burying the algorithms inside middleware callbacks.

```text
GridMap ──> AStarPlanner ──> metric path
                              │
Pose2D ───────────────────────┼──> PathFollower ──> BodyTwist
                              │                       │
                              │                       v
                              │              DifferentialDriveKinematics
                              │                       │
                              │                       v
                              └────────────── DifferentialDriveOdometry
```

## Modules

| Module | Responsibility | Key invariant |
| --- | --- | --- |
| `GridMap` | Occupancy and metric cell geometry | Out-of-bounds is never traversable |
| `AStarPlanner` | 4/8-connected shortest-path planning | Diagonal corner cutting is rejected |
| `PidController` | Bounded heading feedback | Output and integral are explicitly clamped |
| `DifferentialDriveKinematics` | Body/wheel velocity transforms | Forward/inverse transforms are consistent |
| `DifferentialDriveOdometry` | SE(2) dead reckoning from wheel travel | Heading is normalized every update |
| `PathFollower` | Waypoint tracking | Explicit terminal goal tolerance and bounded angular rate |

## Deliberate boundaries

- No ROS 2 dependency in the algorithmic core.
- No global state, background threads, or hidden clocks.
- Simulation uses a fixed integration step for deterministic regression tests.
- Units are encoded in names (`_m`, `_mps`, `_rps`, `_s`) to reduce unit ambiguity.
- Invalid construction and invalid time steps fail early with typed C++ exceptions.

## Extension path

The next repository, `ros2-autonomous-mobile-robot`, should treat this library as a reusable domain layer and add:

- `rclcpp` nodes and lifecycle ownership;
- Topics / Services / Actions;
- TF2 transforms;
- URDF and Gazebo simulation;
- ROS 2 QoS policy experiments;
- rosbag-based integration regression;
- integration tests around the middleware boundary.
