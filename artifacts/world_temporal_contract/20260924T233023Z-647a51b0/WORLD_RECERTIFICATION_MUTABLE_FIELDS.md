# World recertification mutable-field allowlist

Source comparison of pre/post recertification `CandidateBundle` copy:

| Field group | Mutation permitted? | Source result |
|---|---:|---|
| `world_identity` | Yes | Replaced with newly certified snapshot identity. |
| `valid_until_ns` | Conditionally | Active only: may increase to supplied `now + freshness window`, capped by declared analytic endpoint. Pending receives zero refresh. |
| Everything else | No | Copy preserves trajectory evaluator, role schedule, request/goal/localization identity, bundle generation, pinned world provenance, protected region, certificates, endpoints, activation/valid-from and metadata. |

`valid_from_ns` and `activation_stamp_ns` are never changed by the recertification preparation lambda. No trajectory geometry, generation or request identity mutation is present. Regression tests must compare these fields and sampled trajectory output before/after refresh. The callable evaluator and validator are copied as immutable `std::function` objects; value serialization/replay remains debt, not demonstrated control ambiguity.
