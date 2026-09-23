# Static non-authority guard

Command: `python3 tools/audit_observability/check_non_authority.py`.

The script fails if audit symbols appear outside the compile-guarded runtime, adapter and controller hooks, or if any C++ source subscribes to `AuditEvent` or `/navigation/audit_event`. It passes on the current worktree. This is a narrow static guard, not a formal information-flow proof; source review and unchanged product tests remain required. The only product-facing trace entry is `Sink::emit(Event)` returning `void` and `noexcept`, with no branch on its result.

The ON Release tests include `test_audit_event.cpp` for fixed schema/clock and identity separation and `test_audit_sink.cpp` for bounded overflow/drop accounting. The normalizer's nine Python tests cover command-key extension, process restart sequence partitioning, exact one-to-one pairing, clock-proof gating, first-use ordering, and cumulative drops. `python3 -m unittest discover -s tools/audit_observability/tests -v` passed 9/9. Product code has no audit topic subscriber, and the external scan gate runs only when the experiment-specific environment selector is set for a generated session configuration.
