# Verification record

This file separates local development evidence from GitHub-hosted verification and
keeps performance claims scoped to deterministic regression fixtures.

## v0.1 local verification — 2026-09-09

| Gate | Result |
| --- | --- |
| GCC 14.2 / Release / warnings-as-errors | PASS |
| Clang 17 / Release / warnings-as-errors | PASS |
| Clang 17 / ASan + UBSan | PASS |
| CTest contract suite | PASS |
| Deterministic navigation demo | PASS |
| CMake install/export | PASS |
| Independent `find_package` consumer | PASS |
| `git diff --check` | PASS |
| Workflow / Dependabot YAML parse | PASS |

The original deterministic demo produced 21 planned cells, a 6.24264 m A* path
cost, 84 expanded nodes, 502 simulation steps at 20 ms, and 0.0985264 m final
goal error.

## v0.1 GitHub-hosted verification

Published snapshot `30d74616b07c4ea9251045c10ea667e835acaba9`:

| Remote gate | Result | Evidence |
| --- | --- | --- |
| GCC / Clang / MSVC CI | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| ASan + UBSan | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| Installed-package consumer | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| CodeQL C/C++ analysis | PASS | [CodeQL run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178311) |
| Dependabot action checks | PASS | [run](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353179604) / [run](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353184519) |

## v0.2 evidence — 2026-09-09

The v0.2 change was developed through PR
[#7](https://github.com/tahazarif10/robotics-control-core/pull/7), with the final
head passing CI and CodeQL before squash merge.

Deterministic benchmark:

| Controller | Waypoints | Steps | Travel | Max cross-track | Final error | Collision-free | Goal |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| PID waypoint baseline | 26 | 545 | 7.158981 m | 0.063708 m | 0.099468 m | yes | yes |
| Pure pursuit + smoothed path | 5 | 479 | 6.905655 m | 0.052827 m | 0.097605 m | yes | yes |

The benchmark is a regression fixture, not a hardware-performance claim.

The squash merge commit
`3c95e79162710d850ecaf326420c6bbd708a5508` is GitHub-verified and the resulting
`main` snapshot completed both hosted quality gates successfully:

| Remote gate | Result | Evidence |
| --- | --- | --- |
| Multi-compiler CI + CTest + sanitizer + package consumer | PASS | [CI](https://github.com/tahazarif10/robotics-control-core/actions/runs/34357410837) |
| CodeQL C/C++ analysis | PASS | [CodeQL](https://github.com/tahazarif10/robotics-control-core/actions/runs/34357411019) |

The release workflow is tag-driven. Package version `0.2.0` on `main` does not by
itself imply that a GitHub Release or tag exists; release packaging is claimed only
after a tag is published and that workflow completes.
