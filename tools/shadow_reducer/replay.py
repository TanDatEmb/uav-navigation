"""Replay normalized diagnostic copies. Observer order is evidence, not a scheduler proof."""
from __future__ import annotations
import csv
import json
from collections import Counter, defaultdict
from pathlib import Path
from .model import (ShadowReducer, RouteIdentity, RouteCursor, Crossing, Quality,
                    ContextKey, Trajectory, ExecutionPhase)


def _rows(path: Path):
    if not path.exists():
        return []
    with path.open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def replay_run(run_dir: Path) -> dict:
    events = sorted(_rows(run_dir / 'audit_events.jsonl'),
                    key=lambda e: (e.get('bag_observer_ns', 0), e.get('producer_id', 0), e.get('diagnostic_sequence', 0)))
    integrity = json.loads((run_dir / 'trace_integrity.json').read_text())
    s = ShadowReducer()
    counts = Counter()
    issues = []
    seq_seen = {}
    seq_keys = set()
    latest_world = (0, 0)
    hold_status_after_request = 0
    for e in events:
        prod = (e.get('process_incarnation', 0), e.get('producer_id', 0))
        seq = e.get('diagnostic_sequence', 0)
        last = seq_seen.get(prod)
        if (prod, seq) in seq_keys:
            counts['duplicate_event'] += 1
            continue
        seq_keys.add((prod, seq))
        if last is not None and seq < last:
            counts['observer_order_inversion'] += 1
        seq_seen[prod] = max(seq, last or 0)
        kind = e['event_name']
        if kind == 'MISSION_GATE':
            flags = e['flags']; wp = e['waypoint_index']; observed = bool(flags & (1 << 9))
            route = RouteIdentity(e['mission_hash'], e['route_revision'], e['localization_epoch'])
            s.mission.reset_route(route)
            s.mission.set_gate(wp, e['request_id'])
            if flags & (1 << 2):
                s.mission.continuation(wp, e['request_id'], e['bundle_generation'])
            if (flags & (1 << 1)) and e['phase'] in (1, 2):
                before = RouteCursor(max(0, wp-1), None, e['previous_sample_stamp_ns'], e['previous_sample_sequence'])
                after = RouteCursor(wp, None, e['current_sample_stamp_ns'], e['current_sample_sequence'])
                s.mission.observe_crossing(Crossing(route, wp, before, after, e['crossing_error_m'], Quality.OBSERVED, 'BALL' if e['phase'] == 1 else 'SEGMENT', e['request_id']))
            if flags & (1 << 10):
                if observed:
                    predicted = s.mission.accept_pass(wp)
                    verdict = s.compare(predicted, observed)
                    counts['pass_'+verdict.value] += 1
                    if verdict.value != 'MATCH': issues.append({'kind':'PASS_ACCEPT','sequence':seq,'verdict':verdict.value})
            elif observed and e['outcome'] == 3:
                # Source proof: STOP acceptance is measured-stop confirmation.
                predicted = s.mission.accept_stop(wp, True)
                verdict = s.compare(predicted, observed)
                counts['stop_'+verdict.value] += 1
        elif kind == 'WORLD_TRANSITION':
            if e['phase'] == 2:
                latest_world = (e['world_generation'], e['world_revision'])
            elif e['phase'] == 4:
                s.execution.safety_stop()
                counts['world_suspend_observed'] += 1
            elif e['phase'] == 8:
                counts['world_resume_observed'] += 1
                if s.execution.phase == ExecutionPhase.RELEASED:
                    issues.append({'kind':'OLD_SESSION_RESUME','sequence':seq,'verdict':'LEGACY_BEHAVIOR_HAZARD'})
        elif kind == 'COMMAND_PUBLISH':
            # Upstream publication never rearms a downstream Hold fence.
            if s.execution.phase == ExecutionPhase.RELEASED:
                counts['publish_after_fence_upstream_only'] += 1
            elif s.execution.phase == ExecutionPhase.NONE:
                context = ContextKey(e['localization_epoch'], e['mission_hash'], e['route_revision'],
                                     e['world_generation'], e['world_revision'], e['goal_epoch'], e['waypoint_index'], e['request_id'])
                s.execution.restart_new_session(context)
                a = Trajectory(e['bundle_generation'], 'UNKNOWN_ROLE', context)
                s.execution.stage(a); s.execution.activate()
        elif kind == 'COMMAND_RECEIVE':
            if s.execution.phase == ExecutionPhase.RELEASED:
                if e['outcome'] == 1 and s.execution.fenced_generation is None:
                    # Startup Hold had no observed predecessor generation.
                    # A valid downstream receive is evidence of a new session;
                    # its explicit product rearm event is not in this trace.
                    context = ContextKey(e['localization_epoch'], e['mission_hash'], e['route_revision'],
                                         e['world_generation'], e['world_revision'], e['goal_epoch'],
                                         e['waypoint_index'], e['request_id'])
                    s.execution.restart_new_session(context)
                    s.execution.stage(Trajectory(e['bundle_generation'], 'UNKNOWN_ROLE', context))
                    s.execution.activate()
                    s.px4.hold = None
                    counts['startup_rearm_inferred'] += 1
                    counts['startup_rearm_boundary_INSUFFICIENT_TRACE'] += 1
                elif e['outcome'] == 1:
                    counts['receive_admitted_after_fence'] += 1
                    issues.append({'kind':'RECEIVE_ADMITTED_AFTER_FENCE','sequence':seq,'verdict':'LEGACY_BEHAVIOR_HAZARD'})
                else:
                    counts['receive_rejected_after_fence'] += 1
            if e['outcome'] == 1:
                key = tuple(e[k] for k in ('localization_epoch','goal_epoch','mission_hash','route_revision',
                                           'request_id','bundle_generation','sample_id'))
                s.px4.reference(key, e['ros_now_ns'])
        elif kind == 'LEASE_DISPOSITION':
            if e['reason'] == 1:
                counts['lease_expiry_observed'] += 1
                s.execution.release()
        elif kind == 'HOLD_TRANSFER':
            if e['phase'] == 1:
                s.px4.request_hold(e['attempt_generation'], e['ros_now_ns'])
                s.execution.release()
                counts['hold_request_observed'] += 1
            elif e['phase'] in (2, 3) and s.px4.hold:
                s.px4.hold.retry_deadline_ns = e['retry_deadline_steady_ns'] or None
            elif e['phase'] == 4:
                s.px4.completed(e['ros_now_ns'], e['outcome'])
                if s.px4.hold and e['retry_deadline_steady_ns'] > 0:
                    s.px4.hold.retry_deadline_ns = e['retry_deadline_steady_ns']
                counts['hold_callback_observed'] += 1
            elif e['phase'] == 5:
                s.px4.vehicle_status(e['source_stamp_ns'], e['reason'], e['ros_now_ns'])
                if e['flags'] & (1 << 4):
                    s.px4.takeover('FAILSAFE')
                elif e['flags'] & (1 << 5):
                    s.px4.takeover('OPERATOR_OR_FAILSAFE')
                if s.px4.hold and e['reason'] == 4 and s.px4.hold.confirmed():
                    hold_status_after_request += 1
            elif e['phase'] == 6:
                s.px4.takeover('DEACTIVATED_UNKNOWN_CAUSE')
    # Causal joins are exact only under the normalized integrity contract.
    pair_counts = Counter(p['status'] for p in integrity.get('command_pairs', []))
    counts['command_exact_pair'] = pair_counts['PAIRED_ROS_CLOCK']
    counts['command_INSUFFICIENT_TRACE'] = pair_counts['AMBIGUOUS_OR_UNPAIRED']
    counts['first_use_exact'] = sum(p.get('first_use') == 'EXACT_KEY_OBSERVED' for p in integrity.get('command_pairs', []))
    counts['hold_status_after_request'] = hold_status_after_request
    counts['hold_confirmed_end'] = int(bool(s.px4.hold and s.px4.hold.confirmed()))
    counts['hold_callback_without_status_end'] = int(bool(s.px4.hold and s.px4.hold.completion and not s.px4.hold.confirmed()))
    counts['queue_drops_reported'] = integrity.get('queue_drops_observed', 0)
    counts['integrity_sequence_gap_ranges'] = len(integrity.get('sequence_gaps', []))
    counts['events'] = len(events)
    # Observed producer-specific gaps make any comparison across a missing
    # causal interval insufficient; startup truncation is reported separately.
    counts['startup_gap_ranges'] = sum(g['first_missing'] == 1 for g in integrity.get('sequence_gaps', []))
    counts['internal_gap_ranges'] = counts['integrity_sequence_gap_ranges'] - counts['startup_gap_ranges']
    # The trace does not include route geometry, measured P/V/A or certificate
    # validity at every frame; these are explicit unknowns, not negative facts.
    return {'run': run_dir.name.removeprefix('run-').removesuffix('-normalized'),
            'counts': dict(counts), 'issues': issues[:100],
            'evidence_limits': ['observer order does not prove cross-process happens-before',
                                'route geometry and physical stop samples unavailable in audit event',
                                'command pair gaps marked insufficient by trace_integrity']}


def replay_all(root: Path) -> dict:
    runs = [replay_run(p) for p in sorted(root.glob('run-*-normalized')) if (p/'audit_events.jsonl').exists()]
    return {'schema':1, 'runs':runs, 'totals':dict(sum((Counter(r['counts']) for r in runs), Counter()))}
