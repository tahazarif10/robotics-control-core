# Verification record

This file separates claims verified in the development environment from checks
that are configured for GitHub-hosted runners and become authoritative only
when those workflows execute on the public repository.

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

## Configured remote gates

The repository workflows additionally require:

- GCC and Clang CI on Ubuntu 24.04;
- MSVC CI on Windows Server 2025;
- ASan + UBSan;
- installed-package consumer smoke test;
- CodeQL C/C++ analysis;
- tag-driven Linux and Windows package creation.

These remote lanes must not be described as passing until GitHub Actions has
actually executed them on the published repository.
