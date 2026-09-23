# Authority ownership decision

**SINGLE_WRITER_VIABLE_WITH_BOUNDARY_EXCEPTIONS.** Source and model tests support a sole Core writer for mission gate acceptance, execution trajectory cutover, stopping lifecycle and planning context. The current product distributes these across NavigationRuntimeNode/ExecutionEpisode/ExecutionTimelineStore, MissionController and NavigationMode mirrors. Directly mirrored adapter planner-recovery and mission-controller state creates avoidable cross-owner transitions.

Independent boundary owners remain: WorldModel publishes immutable latest-world snapshots; PX4 adapter receives/authenticates ControlReference, enforces its receive lease and local state/frame gates, observes VehicleStatus and owns Hold transfer. These are physical/external observations, not competing writers to Core mission/execution policy. Planning/mapping workers compute from immutable requests and return immutable results; Core admits them. Core does not run A*, CIRI, MINCO, world export, disk trace writing or large serialization.

The shadow package is **not** itself the proposed production Core and has no product authority. Its three domains demonstrate information conservation and event-order semantics. A production Core state cutover needs an explicit adapter API and rollback gate; see `MIGRATION_CUT_PLAN.md`.
