# Performance

The branch's changes are offline evaluator parsing and a bounded scenario recorder envelope. No product `src/` byte changes or ROS control callback modifications are present. There is no fresh SITL paired measurement of recorder callback p95/max, evidence queue drops or evaluator runtime overhead; performance non-regression is therefore **unverified**. The previous product-stability and qualification-gate artifacts contain prior measured transport tails and raw evidence writer counts, but are not a before/after measurement for this branch.

No command lease, state freshness or world freshness value changed.
