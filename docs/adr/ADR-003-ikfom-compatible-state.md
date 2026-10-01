# ADR-003: upstream IKFoM state

**Status:** accepted and implemented. The production estimator instantiates
`esekfom::esekf` with the FAST-LIO2 state ordering in `ikfom_state.hpp`.
`ManifoldState` is only an interchange/output view. We do not reimplement
manifold math or retain a custom filter. IKFoM and FAST-LIO reference commits
are pinned in `ikfom_vendor/UPSTREAM.md`; source and selective adaptations
preserve license, provenance and patch traceability.
