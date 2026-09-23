#!/usr/bin/env python3
"""Small proposal model; exhaustive interleavings are design evidence only."""
from collections import deque
from dataclasses import dataclass, replace

@dataclass(frozen=True)
class State:
    intent: int = 1
    active: int = 1
    staged: int = 0
    activation_at: int = 0
    now_tick: int = 0
    phase: str = "Executing"
    measured_crossed: bool = False
    accepted: bool = False
    planned_endpoint: bool = False
    world_version: int = 1
    certified_world: int = 1
    lease_live: bool = True
    hold: str = "Inactive"
    localization_epoch: int = 1
    command_epoch: int = 1
    command_disposition: str = "Authorized"
    reference_exposed: bool = True
    diagnostic_marker: int = 0

EVENTS = ("new_intent", "stage_current", "stage_stale", "cutover", "cross",
          "tick", "planned_endpoint", "accept", "world_update",
          "recertify", "expire", "brake",
          "late_nominal", "measured_stop", "hold_request", "hold_api_ok",
          "hold_status", "reset", "old_plan", "old_command", "new_command",
          "diagnostic_update")

def step(s, event):
    if event == "new_intent": return replace(s, intent=s.intent + 1, staged=0, activation_at=0)
    if event == "stage_current" and s.phase == "Executing" and s.lease_live and s.hold == "Inactive":
        return replace(s, staged=s.intent, activation_at=s.now_tick + 1)
    if event == "stage_stale": return s
    if event == "tick": return replace(s, now_tick=s.now_tick + 1)
    if event == "cutover" and s.phase == "Executing" and s.hold == "Inactive" and s.staged and s.now_tick >= s.activation_at and s.staged == s.intent and s.lease_live and s.certified_world == s.world_version:
        return replace(s, active=s.staged, staged=0, activation_at=0)
    if event == "cross": return replace(s, measured_crossed=True)
    if event == "planned_endpoint": return replace(s, planned_endpoint=True)
    if event == "accept" and s.measured_crossed and s.phase == "Executing" and s.active == s.intent:
        return replace(s, accepted=True)
    if event == "world_update": return replace(s, world_version=s.world_version + 1)
    if event == "recertify" and s.phase == "Executing" and s.lease_live:
        return replace(s, certified_world=s.world_version)
    if event == "expire": return replace(s, lease_live=False, phase="Stopping", command_disposition="Stopping", reference_exposed=False)
    if event == "brake": return replace(s, phase="Stopping", command_disposition="Stopping", reference_exposed=False)
    if event == "late_nominal": return s
    if event == "measured_stop" and s.phase == "Stopping": return replace(s, phase="Holding", command_disposition="Released")
    if event == "hold_request": return replace(s, hold="Requested", command_disposition="Released", reference_exposed=False)
    if event == "hold_api_ok" and s.hold == "Requested": return replace(s, hold="ApiComplete")
    if event == "hold_status" and s.hold in ("Requested", "ApiComplete"):
        return replace(s, hold="Confirmed")
    if event == "reset":
        return replace(s, intent=s.intent + 1, active=0, staged=0, activation_at=0,
                       phase="Stopping" if s.phase == "Stopping" else "Holding",
                       localization_epoch=s.localization_epoch + 1,
                       command_epoch=0, command_disposition="Stopping",
                       reference_exposed=False, lease_live=False,
                       measured_crossed=False, accepted=False)
    if event == "old_plan": return s
    if event == "old_command": return s
    if event == "new_command" and s.phase == "Executing" and s.lease_live and s.hold == "Inactive":
        return replace(s, command_epoch=s.localization_epoch,
                       command_disposition="Authorized", reference_exposed=True)
    if event == "diagnostic_update": return replace(s, diagnostic_marker=s.diagnostic_marker + 1)
    return s

def check(before, event, after):
    assert after.phase in ("Executing", "Stopping", "Holding")
    assert after.command_disposition in ("Authorized", "Stopping", "Released")
    assert after.reference_exposed == (after.command_disposition == "Authorized")
    assert (after.staged == 0) == (after.activation_at == 0)
    if after.reference_exposed:
        assert after.command_epoch == after.localization_epoch
        assert after.phase == "Executing"
        assert after.hold == "Inactive"
    assert after.active == before.active or event in ("cutover", "reset")
    assert not after.accepted or after.measured_crossed
    if event == "planned_endpoint":
        assert after.accepted == before.accepted
    if before.phase == "Stopping" and event != "measured_stop":
        assert after.phase == "Stopping"
    if event in ("late_nominal", "stage_stale", "old_plan", "old_command"):
        assert after == before
    if event == "diagnostic_update":
        assert replace(after, diagnostic_marker=before.diagnostic_marker) == before
    if after.hold == "Confirmed" and before.hold != "Confirmed":
        assert event == "hold_status"
    if event == "cutover" and after.active != before.active:
        assert before.now_tick >= before.activation_at
        assert before.staged == before.intent
        assert before.certified_world == before.world_version
        assert before.lease_live

def main():
    start = State()
    seen = {start}
    q = deque([(start, 0)])
    edges = 0
    max_depth = 6
    while q:
        state, depth = q.popleft()
        if depth == max_depth: continue
        for event in EVENTS:
            nxt = step(state, event)
            check(state, event, nxt)
            edges += 1
            if nxt not in seen:
                seen.add(nxt)
                q.append((nxt, depth + 1))
    print(f"proposal model PASS states={len(seen)} edges={edges} depth={max_depth}")
    # Explicit counterexample to collapse: desired can advance while old active stays.
    assert step(start, "new_intent").intent != step(start, "new_intent").active
    # API completion is not status confirmation.
    assert step(step(start, "hold_request"), "hold_api_ok").hold == "ApiComplete"
    # Planned endpoint and diagnostic changes cannot authorize mission progress.
    assert not step(start, "planned_endpoint").accepted
    assert step(start, "diagnostic_update").active == start.active
    # A reset fences the old command; absence of a reference has an explicit meaning.
    reset = step(start, "reset")
    assert not step(reset, "old_command").reference_exposed
    assert reset.command_disposition == "Stopping"

if __name__ == "__main__": main()
