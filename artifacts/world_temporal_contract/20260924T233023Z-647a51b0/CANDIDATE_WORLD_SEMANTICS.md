# Candidate world semantics (AS-IS)

- `pinned_world_identity`: exact immutable WorldModel identity supplied to the solve and retained as provenance after recertification.
- `world_identity`: latest identity on which the executable candidate was certified. Initially equal to the pinned identity; world recertification replaces this field on a copy only after fast-path proof or full immutable validation succeeds.
- `protected_region`: bounded spatial region used to prove a sequence of world changes disjoint. Invalid region, identity/generation mismatch, missing/incomplete history, unknown revision link or touching/intersecting boxes conservatively reports intersection.
- `world_validator`: immutable callable captured with the candidate; validation evaluates the same candidate geometry and certificates against a supplied immutable snapshot outside owner locks. It returns evaluated bundle generation and pinned/validated identities. Exceptions/missing callable fail validation.

The two world identities are intentionally independent: the solve provenance remains fixed while the current certificate can move to a newer revision. A refreshed execution retains bundle generation, request, goal, localization epoch, evaluator, role schedule and protected region; only world identity and, under a bounded rule, `valid_until_ns` change. The callable representation is a replay/serialization debt, but source shows it is immutable candidate data and it does not currently block runtime revalidation. No `WorldCertificateOwner` is warranted by current ownership evidence.
