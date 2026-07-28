# ADR-003: IKFoM-compatible state

**Status:** accepted. The estimator state follows the FAST-LIO/IKFoM-compatible
manifold model in one `ManifoldState` definition. We do not reimplement manifold
math or introduce a custom 15-state. IKFoM and FAST-LIO reference commits are
pinned in `ikfom_vendor/UPSTREAM.md`; any source import must preserve license
and patch traceability.
