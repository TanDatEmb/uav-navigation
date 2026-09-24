# Lifecycle identity contract

A transaction is keyed by runtime instance, session, localization epoch, goal epoch, request identity and producer identity. A normal planner transaction uses the declared immutable owner planning cycle. Heading and emergency generations without a planner cycle use `(candidate_source, source, bundle_generation)`. A terminal monitor event uses `(terminal_monitor, captured_bundle_generation, producer_event_sequence)`. A rejected no-execution sample uses `(no_execution, sample_id)`. This preserves independent producer classes without fabricating a planner owner.

Within a transaction, disposition, causal cycle, bundle owner, generation, sample, trace sequence, authorization boundary, source stamp and world identity must not contradict. The reducer deduplicates equal observations and reports distinct contradictory observations. Exact adapter trace identities bind publication; callback time or a recorder cache is never an owner key.

An absent required producer field yields unresolved/`NOT_EVALUABLE`. A later event may neither borrow an earlier bundle owner nor inherit a prior activation. Historical raw evidence lacking these fields remains unresolved.
