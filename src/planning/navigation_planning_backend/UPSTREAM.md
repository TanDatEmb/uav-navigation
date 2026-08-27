# Planner backend provenance

This package is independently maintained product code. Its initial port used
the public implementation at reference commit
`2ad3419c127a617c6d7df6925e81a14175a9c096` from the SUPER project as an
algorithmic reference. The reference checkout is not required by the build,
tests or runtime, and no reference implementation is a second product
authority.

The product implementation has its own C++20 layout, configuration contract,
world-model boundary, planner facade, immutable candidate publication and
execution hand-off. Safety decisions are owned by the product world model,
trajectory validator and execution store; optimizer objective terms are not
safety certificates.

The maintained product surface includes:

- `include/navigation_planning_backend/planner_facade.hpp` as the only installed
  planner entry header;
- private planner, corridor, search and trajectory implementation headers and
  sources used only inside the backend target;
- product-owned typed contracts in `navigation_planning` and
  `navigation_execution`.

Reference-specific wrappers, alternate planner interfaces and parity tooling
are not part of the product surface. The source tree retains only the
provenance and license records required for audit in this document,
`THIRD_PARTY_NOTICES.md` and the applicable license files. Future changes must
preserve those notices and update the safety decision ledger when they affect a
gate, budget, fallback, timestamp or validation contract.
