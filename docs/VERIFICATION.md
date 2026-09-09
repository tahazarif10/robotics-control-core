# Verification record

This file separates claims verified in the development environment from checks
verified on GitHub-hosted runners.

## Locally verified — 2026-09-09

The following checks passed against the `v0.1.0` candidate tree:

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

The deterministic demo produced 21 planned cells, a 6.24264 m path cost,
84 expanded A* nodes, 502 simulation steps at 20 ms, and 0.0985264 m final
goal error.

## GitHub-hosted verification — 2026-09-09

The first published `main` snapshot, commit
`30d74616b07c4ea9251045c10ea667e835acaba9`, completed all configured remote
quality gates successfully:

| Remote gate | Result | Evidence |
| --- | --- | --- |
| Multi-compiler CI (GCC, Clang, MSVC) | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| ASan + UBSan lane | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| Installed-package consumer smoke test | PASS | [CI run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178434) |
| CodeQL C/C++ analysis | PASS | [CodeQL run #1](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353178311) |
| Dependabot GitHub Actions update checks | PASS | [run](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353179604) / [run](https://github.com/tahazarif10/robotics-control-core/actions/runs/34353184519) |

The release workflow is tag-driven and remains intentionally unclaimed until a
GitHub tag is published and the packaging workflow executes against that tag.
