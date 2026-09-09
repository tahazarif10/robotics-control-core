# Engineering contract

## Quality gates

A change is acceptable only when all applicable gates pass:

1. GCC build with warnings-as-errors.
2. Clang build with warnings-as-errors.
3. MSVC build with `/W4 /WX`.
4. Deterministic unit tests.
5. ASan + UBSan test lane on Linux.
6. Demo reaches the goal inside its fixed step budget.
7. README claims match executable behavior.

## Testing philosophy

Tests target contracts rather than implementation details. Current coverage includes:

- reachable and unreachable A* cases;
- no diagonal corner-cutting;
- metric path cost;
- differential-drive forward/inverse consistency;
- straight and in-place odometry updates;
- PID saturation/integrator reset;
- path-follower terminal behavior;
- endpoint validity and start-equals-goal planning;
- deterministic kinematics round-trip sweeps;
- invalid-control-timestep rejection.

## Release policy

- `v0.1.x`: algorithm/core contract can still evolve.
- `v0.2.x`: add benchmark harness and map fixtures.
- `v1.0.0`: freeze the public core API after the ROS 2 wrapper consumes it successfully.
