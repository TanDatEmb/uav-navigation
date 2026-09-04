# E10 evidence

Status: **BLOCKED / not a valid StoppedRecovery E10 witness**.

The run captured MAIN-to-BACKUP ownership, but no `StoppedRecovery` state or
PlanFromRest failure injection. The run ended after the certified BACKUP
endpoint was outside waypoint acceptance and the external-mode scenario handed
over; it cannot identify whether the StoppedRecovery authority is the failure
count, the timeout, or both.

The earlier ungated diagnostic run is not substituted: it charged three
initial PlanFromRest failures before any StoppedRecovery witness and is kept
only as separate raw evidence.
