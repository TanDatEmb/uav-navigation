# Authoritative pre-main gate

Command: `python3 tools/runtime/pre_main_gate.py`. It serially performs Release build/provenance, required product package tests, pinned dependency smoke, `px4_ros2_cpp` unit tests with only the one named upstream flaky case excluded, full `tools/runtime/tests` unittest discovery, all architecture/evidence/safety static guards, runtime safety ledger validation, and `git diff --check`. Any required failure returns nonzero.

`px4_ros2_cpp` live-FMU integration tests are explicitly `ENVIRONMENT_GATED / NOT_RUN`; optional pinned upstream examples are explicitly `OPTIONAL_EXAMPLE / NOT_RUN`. Upstream style maintenance targets are not represented as passing.
