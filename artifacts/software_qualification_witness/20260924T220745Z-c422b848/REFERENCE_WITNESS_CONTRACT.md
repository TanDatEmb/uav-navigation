# Reference witness contract

Each executable Core reference carries exact runtime/session, localization epoch, goal epoch, request, bundle generation, sample ID, world generation/revision, and world observation stamp. Its producer-owned authorization appears in the lifecycle stream. The adapter's admission or typed rejection is joined by the exact mode activation/localization/goal/request/generation/sample tuple. A later `px4_input_trace` for the same sample is an additional setpoint-update observation, not a substitute for Core/adapter authorization and not proof of firmware consumption.

The evaluator accepts a required reference only with (1) exact Core producer authorization and exact adapter admission/rejection, or (2) a same-sample producer-owned setpoint trace with complete identity. It rejects missing/conflicting world identity and unbound adapter receipts. It never uses nearest timestamp, previous sample, active bundle cache, or an inferred goal. Accepted references lacking a same-sample setpoint trace are counted separately; this count is 147/15,856 in the primary cohort and 3/11,528 in the verification cohort. Their exact Core/adapter lineage is available, but per-sample PX4 setpoint update is not claimed.

No control payload was added to `NavigationCommand` for evidence. The 27-field control contract and adapter admission behavior remain unchanged.
