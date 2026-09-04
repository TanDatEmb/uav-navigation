#!/usr/bin/env python3
"""Finalize the scenario-separated H10 report from generated CSV artifacts."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

MISSING = "NOT_RECORDED"

def load_rows(path: Path):
    return list(csv.DictReader(path.open(encoding="utf-8")))

def vals(rows, key):
    out=[]
    for row in rows:
        try: out.append(float(row[key]))
        except (KeyError, TypeError, ValueError): pass
    return sorted(out)

def metric(rows, key):
    a=vals(rows,key)
    if not a: return {"count":0,"rms":None,"p50":None,"p95":None,"p99":None,"max":None}
    def q(f): return a[min(len(a)-1,round((len(a)-1)*f))]
    return {"count":len(a),"rms":(sum(x*x for x in a)/len(a))**0.5,"p50":q(.5),"p95":q(.95),"p99":q(.99),"max":a[-1]}

def text_metric(m):
    return f"P95={m['p95'] if m['p95'] is not None else MISSING}; MAX={m['max'] if m['max'] is not None else MISSING}"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--date-root", type=Path, required=True)
    ap.add_argument("--exact", type=Path, required=True)
    ap.add_argument("--open", type=Path, required=True)
    ap.add_argument("--dynamic", nargs="*", type=Path, default=[])
    args=ap.parse_args()
    exact=load_rows(args.exact/"h10_exact_e5_closed_loop.csv")
    control=load_rows(args.open/"h10_open_control.csv")
    events=[]
    event_path=args.date_root/"h10_exact_e5_events.csv"
    if event_path.is_file(): events=load_rows(event_path)
    envelope=json.loads((args.date_root/"h10_dynamic_envelope.json").read_text())
    run_id=args.exact.name
    exact_metrics={
        "tracking_error":metric(exact,"aligned_lio_tracking_error_m"),
        "lio_gt_position_error":metric(exact,"lio_gt_position_error_m"),
        "px4_gt_position_error":metric(exact,"px4_gt_position_error_m"),
        "frame_velocity_residual":metric(exact,"frame_velocity_residual_mps"),
        "planner_velocity":metric(exact,"planner_speed_mps"),
        "planner_acceleration":metric(exact,"planner_acceleration_mps2"),
        "planner_lateral_acceleration":metric(exact,"planner_lateral_acceleration_mps2"),
        "planner_jerk":metric(exact,"planner_jerk_mps3"),
        "px4_delta_v":metric(exact,"delta_v_px4_controller_mps"),
        "px4_delta_a":metric(exact,"delta_a_px4_controller_mps2"),
    }
    control_metrics={
        "tracking_error":metric(control,"aligned_lio_tracking_error_m"),
        "lio_gt_position_error":metric(control,"lio_gt_position_error_m"),
        "px4_gt_position_error":metric(control,"px4_gt_position_error_m"),
        "planner_acceleration":metric(control,"planner_acceleration_mps2"),
        "planner_lateral_acceleration":metric(control,"planner_lateral_acceleration_mps2"),
        "planner_jerk":metric(control,"planner_jerk_mps3"),
        "px4_delta_v":metric(control,"delta_v_px4_controller_mps"),
        "px4_delta_a":metric(control,"delta_a_px4_controller_mps2"),
    }
    report_path=args.date_root/"runtime_report.json"
    report=json.loads(report_path.read_text()) if report_path.is_file() else {"schema_version":1,"hypotheses":{},"critical_events":[],"runs":[]}
    # Make regeneration idempotent while preserving all pre-H10 evidence.
    report["runs"]=[r for r in report.get("runs",[]) if r.get("experiment") not in {"H10_S_BAD_E5_EXACT","H10_S_OPEN_CONTROL"}]
    report["critical_events"]=[e for e in report.get("critical_events",[]) if e.get("type") != "H10_exact_e5_observability_run"]
    h=report.setdefault("hypotheses",{})
    common_note="Scenario-scoped H10 evidence is incomplete: the selected run has PX4_INPUT_SETPOINT and independent ground truth, but no injected_replan_failure=1 marker and no valid increasing-demand S_DYNAMIC_ID segment meeting the stated envelope criterion."
    h.update({
        "H10a_planner_demand_exceeds_usable_closed_loop_envelope":{"status":"INCONCLUSIVE","evidence":[common_note,str(args.exact/"h10_exact_e5_closed_loop.csv"),str(args.date_root/"h10_dynamic_envelope.json")]},
        "H10b_px4_controller_materially_reshapes_command":{"status":"INCONCLUSIVE","evidence":[str(args.exact/"h10_exact_e5_closed_loop.csv"),"PX4 effective setpoint and exact External Mode input are recorded, but no clean injected-E5 boundary isolates causal reshaping."]},
        "H10c_lio_estimator_material_contributor":{"status":"INCONCLUSIVE","evidence":["LIO-vs-ground-truth position is recorded for the exact scenario; attribution is not isolated from vehicle/controller response."]},
        "H10d_px4_estimator_material_contributor":{"status":"INCONCLUSIVE","evidence":["PX4-vs-ground-truth position is recorded; exact-run velocity estimator attribution is NOT_RECORDED."]},
        "H10e_vehicle_cannot_follow_px4_effective_setpoint":{"status":"INCONCLUSIVE","evidence":["PX4 effective setpoint, measured state, and ground truth are present, but no clean dynamic-ID segment separates plant limitation from command reshaping."]},
    })
    report.setdefault("critical_events",[]).append({"type":"H10_exact_e5_observability_run","run_id":run_id,"injected_failure_observed":False,"T_cross_ns":next((e["timestamp_ns"] for e in events if e["event"]=="tracking_error_0.25_m"),MISSING)})
    report.setdefault("runs",[]).append({"experiment":"H10_S_BAD_E5_EXACT","scenario":"S_BAD_E5","run_id":run_id,"path":str(args.exact),"result":"INCONCLUSIVE","data_quality":"PARTIAL_VALID_LAYERS_NO_INJECTED_MARKER","metrics":exact_metrics})
    report.setdefault("runs",[]).append({"experiment":"H10_S_OPEN_CONTROL","scenario":"S_OPEN_CONTROL","run_id":args.open.name,"path":str(args.open),"result":"INCONCLUSIVE","data_quality":"VALID_LAYERS","metrics":control_metrics})
    report["h10"]={"envelope":envelope,"exact_e5_metrics":exact_metrics,"open_control_metrics":control_metrics,"event_csv":str(event_path),"figures":str(args.date_root/"h10_figures")}
    report_path.write_text(json.dumps(report,indent=2)+"\n")

    def row_for(name):
        return next((r for r in events if r.get("event")==name),None)
    def time_s(row):
        if not row or row.get("timestamp_ns")==MISSING:return MISSING
        try:return f"{(int(row['timestamp_ns'])-int(exact[0]['timestamp_ns']))/1e9:.6f}"
        except (KeyError,ValueError):return MISSING
    event_md=[]
    for row in events:
        event_md.append(f"| {row['event']} | {time_s(row)} | {row.get('e_lio_m',MISSING)} | {row.get('planner_speed_mps',MISSING)} | {row.get('planner_acceleration_mps2',MISSING)} | {row.get('px4_effective_speed_mps',MISSING)} | {row.get('px4_effective_acceleration_mps2',MISSING)} | {row.get('external_mode_output_state',MISSING)} / {row.get('recovery_state',MISSING)} |")
    exact_cross=next((row for row in events if row.get("event")=="tracking_error_0.25_m"),None)
    t_cross = exact_cross.get("timestamp_ns",MISSING) if exact_cross else MISSING
    md=["# H10 Analysis","","## Scenario scope","",f"- `S_BAD_E5`: `{args.exact}`; `sanity_open`, route `external_mode_open_route`, requested speed 3.0 m/s. This is the new-observability run.",f"- `S_OPEN_CONTROL`: `{args.open}`; matched open/straight control, same requested speed and planner/PX4 configuration. Statistics are separate.",f"- `S_DYNAMIC_ID`: {[str(x) for x in args.dynamic]}; low-demand exploratory runs only; no certified envelope is claimed.","","## Evidence validity","",f"The selected S_BAD_E5 run has {len(exact)} canonical PX4-input samples and independent `/sim/ground_truth/odometry`, but `injected_replan_failure=1` was not observed. The natural planner failure and emergency therefore cannot be used as injected-failure causality.",f"Synchronized first 0.25 m crossing: `{t_cross}` ns. Raw layers are retained in `{args.exact/'h10_exact_e5_closed_loop.csv'}`.","","## Required event table","","| Event | Relative time (s) | e LIO (m) | planner speed | planner accel | PX4 eff speed | PX4 eff accel | mode / recovery |","|---|---:|---:|---:|---:|---:|---:|---|",*event_md,"","`injected_solve_start` and `injected_solve_failure` are `NOT_RECORDED`, not inferred.","","## Matched comparison (not pooled)","","| Metric | S_BAD_E5 exact | S_OPEN_CONTROL |","|---|---:|---:|",f"| LIO-GT position P95 | {text_metric(exact_metrics['lio_gt_position_error'])} | {text_metric(control_metrics['lio_gt_position_error'])} |",f"| PX4-GT position P95 | {text_metric(exact_metrics['px4_gt_position_error'])} | {text_metric(control_metrics['px4_gt_position_error'])} |",f"| planner acceleration P95 | {text_metric(exact_metrics['planner_acceleration'])} | {text_metric(control_metrics['planner_acceleration'])} |",f"| planner jerk P95 | {text_metric(exact_metrics['planner_jerk'])} | {text_metric(control_metrics['planner_jerk'])} |",f"| PX4 controller delta-V P95 | {text_metric(exact_metrics['px4_delta_v'])} | {text_metric(control_metrics['px4_delta_v'])} |",f"| PX4 controller delta-A P95 | {text_metric(exact_metrics['px4_delta_a'])} | {text_metric(control_metrics['px4_delta_a'])} |",f"| command/LIO tracking P95 | {text_metric(exact_metrics['tracking_error'])} | {text_metric(control_metrics['tracking_error'])} |","","## H10 classifications","","| Hypothesis | Status | Scenario-scoped conclusion |","|---|---|---|","| H10a planner demand exceeds usable closed-loop envelope | INCONCLUSIVE | Demand/error correlation is present, but no valid increasing-demand envelope matrix. |","| H10b PX4 controller materially reshapes planner trajectory | INCONCLUSIVE | Exact input/effective layers are recorded; causal isolation is incomplete. |","| H10c LIO estimator is a material contributor | INCONCLUSIVE | LIO-GT error is measured, not isolated as the cause. |","| H10d PX4 estimator is a material contributor | INCONCLUSIVE | PX4-GT position is measured; velocity attribution is NOT_RECORDED. |","| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | No clean ID segment separates plant limitation from reshaping. |","","## Usable closed-loop envelope","",f"Status: **{envelope.get('status','NOT_TESTED')}**. Criterion: P95 tracking <= 0.10 m, MAX <= 0.175 m, no recovery/emergency, estimator health valid. No production planner limit was changed.","","## Causal ordering","", "The evidence orders tracking degradation before the natural planner failure/emergency. It does not establish that an injected failure caused the degradation, because the required injection marker is absent.","","## Figures","",*([f"- `{args.date_root/'h10_figures'/x}`" for x in sorted(p.name for p in (args.date_root/"h10_figures").glob("*.png"))]),"","## FIRST FIX RECOMMENDATION","", "Do not implement a behavioral fix from this incomplete attribution. The highest-leverage next action is a valid controlled dynamics-ID run plus exact E5 rerun with the four synchronized layers; if that confirms H10a with small H10b, the first product change should be one product-owned `VehicleControlEnvelope` coupling planner demand to measured closed-loop limits. Preserve the 0.25 m certificate, fail-closed recovery, estimator-health gates, and exact E5/open-control regression tests."]
    (args.date_root/"h10_analysis.md").write_text("\n".join(md)+"\n")
    print(json.dumps({"exact":exact_metrics,"open":control_metrics,"envelope":envelope.get("status"),"events":len(events)},indent=2))

if __name__ == "__main__": main()
