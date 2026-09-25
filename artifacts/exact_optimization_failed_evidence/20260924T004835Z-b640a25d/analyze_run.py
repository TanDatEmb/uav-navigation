#!/usr/bin/env python3
"""Read-only focused SITL evidence check; bag receive time is not PX4 consumption."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def quantile(values: list[float], p: float) -> float | None:
    if not values:
        return None
    v = sorted(values)
    x = (len(v)-1)*p
    lo = int(x)
    hi = min(lo+1, len(v)-1)
    return v[lo]+(v[hi]-v[lo])*(x-lo)


def gaps_ms(timestamps: list[int]) -> dict:
    gaps = [(b-a)/1e6 for a,b in zip(timestamps,timestamps[1:])]
    return {"n":len(gaps), "p50":quantile(gaps,.5), "p95":quantile(gaps,.95),
            "p99":quantile(gaps,.99), "max":max(gaps) if gaps else None}


def topic(conn: sqlite3.Connection, name: str):
    row = conn.execute("SELECT id,type FROM topics WHERE name=?", (name,)).fetchone()
    if row is None:
        return []
    cls = get_message(row[1])
    return [(t,deserialize_message(data,cls)) for t,data in conn.execute(
        "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp",(row[0],))]


def fields(status):
    return {x.key:x.value for x in status.values}


def file_sha(path: Path):
    h=hashlib.sha256()
    with path.open("rb") as f:
        for b in iter(lambda:f.read(1024*1024),b""):
            h.update(b)
    return {"path":str(path),"size_bytes":path.stat().st_size,"sha256":h.hexdigest()}


def analyze(session: Path, injected: bool):
    report=json.loads((session/"report.json").read_text())
    metadata=json.loads((session/"metadata.json").read_text())
    bag=next((session/"rosbag").glob("*.db3"))
    with sqlite3.connect(bag) as conn:
        diag=topic(conn,"/navigation/diagnostics")
        command=topic(conn,"/navigation/navigation_command")
        admission=topic(conn,"/navigation/command_admission")
        px4=topic(conn,"/fmu/in/trajectory_setpoint")
    events=[]; traces=[]; retained=[]
    for t,msg in diag:
        for st in msg.status:
            if st.name=="navigation_runtime/exact_optimization_failure_injection":
                events.append({"observer_ns":t,"event":st.message,"fields":fields(st),
                    "ros_ns":msg.header.stamp.sec*10**9+msg.header.stamp.nanosec})
            elif st.name=="navigation_runtime/planner" and st.message=="DECISION_TRACE":
                traces.append({"observer_ns":t,"fields":fields(st)})
            elif st.name=="navigation_runtime/retained_command_decision":
                retained.append({"observer_ns":t,"fields":fields(st)})
    armed=[e for e in events if e["event"]=="FAULT_INJECTION_ARMED"]
    applied=[e for e in events if e["event"]=="FAULT_INJECTION_APPLIED"]
    rejected=[e for e in events if e["event"]=="FAULT_INJECTION_REJECTED"]
    injection_state = ("INJECTION_REJECTED" if rejected and not armed else
                       "INJECTION_NOT_ARMED" if not armed else
                       "INJECTION_ARMED" if not applied else
                       "INJECTION_APPLIED" if len(applied)==1 else
                       "INJECTION_APPLIED_MULTIPLE_TIMES")
    # This is the analyzer's terminal one-shot state, derived from the single
    # applied event. It is not a fabricated second runtime diagnostic event.
    one_shot_terminal_state = ("INJECTION_ALREADY_CONSUMED" if len(applied)==1 else
                               "INJECTION_NOT_ARMED" if not armed else
                               "INJECTION_ARMED")
    fault=applied[0] if len(applied)==1 else None
    cycle=fault["fields"]["planning_cycle_id"] if fault else None
    trace=[e for e in traces if e["fields"].get("planning_cycle_id")==cycle] if cycle else []
    decision=[e for e in retained if e["fields"].get("planning_cycle_id")==cycle] if cycle else []
    desired=int(fault["fields"]["desired_request"]) if fault else None
    active=int(fault["fields"]["active_request"]) if fault else None
    at=fault["observer_ns"] if fault else None
    before=[(t,m) for t,m in admission if at is not None and t<at and m.request_id==active]
    command_before=[(t,m) for t,m in command if at is not None and t<at and
                    m.request_id==active]
    after=[(t,m) for t,m in admission if at is not None and t>at and m.request_id==active]
    successor=[(t,m) for t,m in admission if at is not None and t>at and m.request_id==desired]
    next_successor=successor[0][0] if successor else None
    last_predecessor_before_successor=next((t for t,m in reversed(admission)
        if next_successor is not None and t<next_successor and m.request_id==active),None)
    stop=next_successor or at
    predecessor_admitted=[t for t,m in after if stop is None or t<stop]
    command_window=[t for t,m in command if at is not None and command_before and
        command_before[-1][0]<=t<=stop and m.request_id in (active,desired)] if stop else []
    admission_window=[t for t,m in admission if at is not None and before and
        before[-1][0]<=t<=stop and m.request_id in (active,desired)] if stop else []
    publish_gap=gaps_ms(command_window)
    admission_gap=gaps_ms(admission_window)
    decision_fields=decision[-1]["fields"] if decision else {}
    trace_fields=trace[-1]["fields"] if trace else {}
    hold=report.get("external_mode",{}).get("px4_hold_handover_requested_sim_ns")
    complete=report["mission_outcome"]["acceptance"]["mission_complete_observed"]
    accepted=report["mission_outcome"]["acceptance"]["waypoint_acceptance_indices"]
    failure_lines=[]
    for path in (session/"logs").glob("*.log"):
        if "navigation_runtime_node" not in path.name and "external_mode" not in path.name:
            continue
        for line in path.read_text(errors="replace").splitlines():
            lower=line.lower()
            if any(s in lower for s in ("stale pva", "command lease expir", "identity reject", "continuity reject")):
                failure_lines.append(f"{path.name}: {line[:350]}")
    checks={"mission_complete":bool(complete),"accepted_all":[0,1,2,3,4]==accepted,
            "no_unexpected_hold":hold is None,
            "no_boundary_failure_log":not failure_lines}
    if injected:
        checks.update({
            "armed_exactly_once":len(armed)==1,
            "applied_exactly_once":len(applied)==1,
            "exact_status":bool(fault and fault["fields"].get("injected_planner_status")=="6" and
                                 trace_fields.get("planner_result_after_injection")=="6"),
            "original_status_preserved":bool(fault and
                fault["fields"].get("original_planner_status") is not None and
                trace_fields.get("planner_result_before_injection")==
                    fault["fields"].get("original_planner_status") and
                trace_fields.get("planner_backend_outcome") is not None),
            "hot_handoff":bool(fault and desired==active+1 and active in (2,3) and
                                trace_fields.get("transition_kind")=="same_route_waypoint_advance" and
                                trace_fields.get("desired_request_id")==str(desired) and
                                trace_fields.get("executing_request_id")==str(active)),
            "classifier_retains":trace_fields.get("planner_disposition")=="4",
            "hg023_validator_observed":bool(decision and decision_fields.get("purpose")=="0" and
                                             decision_fields.get("owner_snapshot_current")=="1" and
                                             decision_fields.get("callback_request_current")=="1" and
                                             decision_fields.get("after_command_available")=="1" and
                                             decision_fields.get("disposition") in {"6","7","8"}),
            "predecessor_before_and_after":bool(before and command_before and
                                                predecessor_admitted),
            "successor_admitted":bool(successor),
            "publication_gap_under_100ms":bool(publish_gap["max"] is not None and publish_gap["max"]<100),
            "adapter_gap_under_100ms":bool(admission_gap["max"] is not None and admission_gap["max"]<100),
        })
    else:
        checks["no_fault_injected"]=not applied
    raw=[file_sha(p) for p in [bag,session/"scenario.jsonl",session/"metadata.json",
        session/"report.json"] if p.exists()]
    for pattern in ("*navigation_runtime_node*.log","*px4_navigation_external_mode_node*.log"):
        raw += [file_sha(p) for p in (session/"logs").glob(pattern)]
    return {"session":str(session),"injected_expected":injected,"verdict":"FOCUSED_PASS" if all(checks.values()) else "FOCUSED_FAIL",
        "checks":checks,"events":events,"trace":trace[-1] if trace else None,
        "retained_decision":decision[-1] if decision else None,
        "injection_state":injection_state,
        "one_shot_terminal_state":one_shot_terminal_state,
        "runs_rejected":len(rejected),"runs_armed":len(armed),
        "runs_injected":len(applied),"exact_events":sum(
            e["fields"].get("injected_planner_status")=="6" for e in applied),
        "predecessor_admissions_after_fault":len(predecessor_admitted),
        "fault_to_successor_admission_ms":(next_successor-at)/1e6 if next_successor and at else None,
        "command_publication_interarrival_ms":publish_gap,
        "adapter_admission_interarrival_ms":admission_gap,
        "successor_handoff_gap_ms":(next_successor-last_predecessor_before_successor)/1e6
            if next_successor and last_predecessor_before_successor else None,
        "mission_complete":complete,"accepted":accepted,"hold_request_sim_ns":hold,
        "failure_log_lines":failure_lines,"bag_message_counts":{
            "command":len(command),"admission":len(admission),"diagnostics":len(diag),"px4_setpoint":len(px4)},
        "raw_evidence":raw,"source_head":metadata.get("build_provenance",{}).get("manifest",{}).get("source",{}).get("git_head"),
        "qualification_verdict":report.get("verdict"),"qualification_eligible":report.get("qualification_eligible")}


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("session",type=Path)
    parser.add_argument("--nominal",action="store_true")
    parser.add_argument("--output",type=Path)
    args=parser.parse_args()
    result=analyze(args.session,not args.nominal)
    payload=json.dumps(result,indent=2,sort_keys=True)
    if args.output:
        args.output.write_text(payload+"\n")
    print(json.dumps({"session":result["session"],"verdict":result["verdict"],
        "injection_state":result["injection_state"],
        "checks":result["checks"],"fault_to_successor_admission_ms":result["fault_to_successor_admission_ms"],
        "adapter_gap_max_ms":result["adapter_admission_interarrival_ms"]["max"]},sort_keys=True))
    raise SystemExit(0 if result["verdict"]=="FOCUSED_PASS" else 1)
