# H1 — normal Hold

Expected: stationary stream remains active, one serialized Hold request is in flight, and only `VehicleStatus.nav_state=AUTO_LOITER` confirms handover. The executor callback success path now waits for that status. Deterministic helper tests cover callback/status ordering. The H2 mapping-only World-stale SITL also exercised one failNavigation→stationary stream→observed AUTO_LOITER handover. A separate non-fault normal/terminal Hold run has not yet been captured.
