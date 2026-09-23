#!/usr/bin/env python3
"""Small proposal model; exhaustive interleavings are design evidence only."""
from collections import deque
from dataclasses import dataclass, replace

@dataclass(frozen=True)
class State:
    intent: int = 1
    active: int = 1
    staged: int = 0
    phase: str = "Executing"
    measured_crossed: bool = False
    accepted: bool = False
    world_version: int = 1
    certified_world: int = 1
    lease_live: bool = True
    hold: str = "Inactive"
    localization_epoch: int = 1

EVENTS = ("new_intent", "stage_current", "stage_stale", "cutover", "cross",
          "accept", "world_update", "recertify", "expire", "brake",
          "late_nominal", "measured_stop", "hold_request", "hold_api_ok",
          "hold_status", "reset", "old_plan")

def step(s, event):
    if event == "new_intent": return replace(s, intent=s.intent + 1, staged=0)
    if event == "stage_current" and s.phase == "Executing" and s.lease_live:
        return replace(s, staged=s.intent)
    if event == "stage_stale": return s
    if event == "cutover" and s.phase == "Executing" and s.staged == s.intent and s.lease_live and s.certified_world == s.world_version:
        return replace(s, active=s.staged, staged=0)
    if event == "cross": return replace(s, measured_crossed=True)
    if event == "accept" and s.measured_crossed and s.phase == "Executing" and s.active == s.intent:
        return replace(s, accepted=True)
    if event == "world_update": return replace(s, world_version=s.world_version + 1)
    if event == "recertify" and s.phase == "Executing" and s.lease_live:
        return replace(s, certified_world=s.world_version)
    if event == "expire": return replace(s, lease_live=False, phase="Stopping")
    if event == "brake": return replace(s, phase="Stopping")
    if event == "late_nominal": return s
    if event == "measured_stop" and s.phase == "Stopping": return replace(s, phase="Holding")
    if event == "hold_request": return replace(s, hold="Requested")
    if event == "hold_api_ok" and s.hold == "Requested": return replace(s, hold="ApiComplete")
    if event == "hold_status" and s.hold in ("Requested", "ApiComplete"):
        return replace(s, hold="Confirmed")
    if event == "reset":
        return replace(s, intent=s.intent + 1, active=0, staged=0,
                       phase="Stopping" if s.phase == "Stopping" else "Holding",
                       localization_epoch=s.localization_epoch + 1,
                       lease_live=False, measured_crossed=False, accepted=False)
    if event == "old_plan": return s
    return s

def check(before, event, after):
    assert after.active == before.active or event in ("cutover", "reset")
    assert not after.accepted or after.measured_crossed
    if before.phase == "Stopping" and event != "measured_stop":
        assert after.phase == "Stopping"
    if event in ("late_nominal", "stage_stale", "old_plan"):
        assert after == before
    if after.hold == "Confirmed" and before.hold != "Confirmed":
        assert event == "hold_status"
    if event == "cutover" and after.active != before.active:
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

if __name__ == "__main__": main()
