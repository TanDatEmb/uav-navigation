# Coding Conventions — uav-navigation

## C++ Style

Primary reference: ROS 2 C++ Style Guide.
Supplement: PX4 Style Guide for safety-critical and autopilot-adjacent code.

| Item | Rule |
|------|------|
| Classes / structs | `PascalCase` |
| Functions / methods | `snake_case()` |
| Member variables | `snake_case_` |
| Local variables | `snake_case` |
| Parameters | `snake_case` |
| Namespaces | `snake_case` |
| File names | `snake_case.cpp`, `snake_case.hpp` |
| `constexpr` constants | `kPascalCase` (PX4 preferred) |
| `#define` constants | `SCREAMING_SNAKE_CASE` |
| Indentation | 4 spaces |
| Braces | attach (K&R) |
| Line length | ≤ 100 characters |

## Comments and Documentation

- All source comments in English.
- Use Doxygen style for public APIs: `/** ... */`.
- No commented-out debug code in final commits.
- Explain **why**, not what, when intent is non-obvious.

## Parameters

- Every new class member must have a matching `declare_parameter` + `get_parameter` call in the same commit.
- Prefer typed ROS 2 parameters with descriptors and bounds.
- No magic numbers in logic; use named constants or YAML parameters.

## Control Flow

- Single setpoint / command publish per control cycle.
- Handle invalid `dt`, missing data, or stale input early with explicit returns or errors.
- Failsafe paths must be obvious and documented.

## Frame Conventions

Frame conventions will be documented in `docs/architecture.md` once finalized. Every transform must have a named, versioned convention.

## Formatting and Linting

- `.clang-format` enforces the style defined above.
- `.clang-tidy` catches common C++ issues.
- `CPPLINT.cfg` enforces Google-style lint checks.

Run formatting before committing:

```bash
./tools/format.sh
```

## Pre-commit Checklist

- [ ] New members have declare + load parameter.
- [ ] `compute_*` functions receive correct input type and unit.
- [ ] No Vietnamese comments.
- [ ] No leftover debug logs.
- [ ] Single publish path per cycle.
- [ ] `clang-format` passes.
