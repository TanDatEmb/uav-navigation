# Boundary debt outside this cut

1. World source-time staleness from the repaired checkpoint remains an isolated boundary debt. This cut preserves certificate recertification/revocation, UNKNOWN policy and command lease; it does not redesign world temporal policy. A separate source-time fault needs its own focused runtime evidence.
2. PX4 Hold callback/VehicleStatus ordering remains a distinct adapter protocol concern. This cut does not change Hold request, retry, takeover or adapter-local admission.
3. External PX4 checkout was dirty at the pinned baseline. The exact checkout SHA and binary hash are recorded in `BASE_PROVENANCE.md`; focused SITL is diagnostic and no flight qualification claim follows.

No third architecture cut is authorized by this artifact. It can be considered only after the current cut reaches its merge-readiness gates.
