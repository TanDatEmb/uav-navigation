# Static non-authority guard

Command: `python3 tools/audit_observability/check_non_authority.py`.

The script fails if audit symbols appear outside the compile-guarded runtime, adapter and controller hooks, or if any C++ source subscribes to `AuditEvent` or `/navigation/audit_event`. It passes on the current worktree. This is a narrow static guard, not a formal information-flow proof; source review and unchanged product tests remain required. The only product-facing trace entry is `Sink::emit(Event)` returning `void` and `noexcept`, with no branch on its result.
