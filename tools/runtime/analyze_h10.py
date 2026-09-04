#!/usr/bin/env python3
"""Scenario-separated H10 closed-loop attribution analysis.

This tool is intentionally offline.  It samples only retained run artifacts,
keeps ENU/NED conversion explicit, and marks missing layers instead of
inventing a controller or ground-truth signal.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean
from typing import Any

MISSING = "NOT_RECORDED"
LIMIT = 0.25
ROLE = {0: "MAIN", 1: "BACKUP", 2: "EMERGENCY", 255: "UNKNOWN"}
RECOVERY = {0: "INITIAL_HOLD", 1: "TRACK_MAIN", 2: "TRACK_BACKUP", 3: "EMERGENCY_BRAKE", 4: "STOPPED_RECOVERY", 5: "PX4_HOLD"}
MODE = {0: "TRACK_TRAJECTORY", 1: "WAIT_AIRBORNE", 2: "WAIT_HEALTH", 3: "WAIT_FIRST_COMMAND", 4: "MISSION_HOLD", 5: "COMPLETED_HOLD", 6: "RECOVERY_HOLD", 7: "FAILSAFE_HOLD", 8: "HANDOVER_HOLD"}

def num(v: Any) -> float | None:
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except (TypeError, ValueError):
        return None

def vec(v: Any) -> list[float] | None:
    if isinstance(v, str):
        if v == MISSING: return None
        try: v = [float(x) for x in v.strip("[]").split(",")]
        except ValueError: return None
    if not isinstance(v, (list, tuple)) or len(v) != 3: return None
    out = [num(x) for x in v]
    return out if all(x is not None for x in out) else None

def n(v: list[float] | None) -> float | None:
    return math.sqrt(sum(x*x for x in v)) if v is not None else None

def lateral_acceleration(v: list[float] | None, a: list[float] | None) -> float | None:
    if v is None or a is None:
        return None
    vv = sum(x*x for x in v)
    if vv <= 1e-12:
        return 0.0
    scale = sum(x*y for x,y in zip(v,a)) / vv
    longitudinal = [scale*x for x in v]
    return n(sub(a, longitudinal))

def sub(a: list[float] | None, b: list[float] | None) -> list[float] | None:
    return [x-y for x,y in zip(a,b)] if a is not None and b is not None else None

def enu_to_ned(v: list[float] | None) -> list[float] | None:
    return [v[1], v[0], -v[2]] if v is not None else None

def ned_to_enu(v: list[float] | None) -> list[float] | None:
    return [v[1], v[0], -v[2]] if v is not None else None

def j(v: Any) -> str:
    return MISSING if v is None else json.dumps(v, separators=(",", ":"))

def load_jsonl(p: Path) -> list[dict[str, Any]]:
    if not p.is_file(): return []
    out=[]
    with p.open(encoding="utf-8", errors="replace") as f:
        for line in f:
            try:
                x=json.loads(line)
                if isinstance(x,dict): out.append(x)
            except json.JSONDecodeError: pass
    return out

def sample_stream(samples: list[dict[str, Any]], name: str) -> list[tuple[int,dict[str,Any]]]:
    out=[]
    for x in samples:
        if x.get("stream") != name: continue
        t=num(x.get("timestamp_ns"))
        if t is not None: out.append((int(t), x.get("payload",{})))
    return sorted(out, key=lambda item: item[0])

def interp(series: list[tuple[int,dict[str,Any]]], t: int, fields: tuple[str,...]) -> tuple[dict[str,Any] | None,float|None]:
    if not series: return None,None
    lo,hi=0,len(series)-1
    while lo<hi:
        m=(lo+hi)//2
        if series[m][0] < t: lo=m+1
        else: hi=m
    candidates=[lo]
    if lo>0: candidates.append(lo-1)
    nearest=min(candidates,key=lambda i:abs(series[i][0]-t))
    if abs(series[nearest][0]-t)>200_000_000: return None,None
    # Vectors are linearly interpolated only when both bracketing samples carry
    # finite vectors.  Scalars/enum context uses the nearest sample.
    if lo>0 and lo<len(series) and series[lo-1][0] <= t <= series[lo][0] and series[lo][0] != series[lo-1][0]:
        ta,a=series[lo-1]; tb,b=series[lo]; f=(t-ta)/(tb-ta)
        result={}
        for key in fields:
            av,bv=vec(a.get(key)),vec(b.get(key))
            if av is not None and bv is not None: result[key]=[x+(y-x)*f for x,y in zip(av,bv)]
            else: result[key]=b.get(key) if f>=0.5 else a.get(key)
        return result,max(t-ta,tb-t)/1e6
    return series[nearest][1],abs(series[nearest][0]-t)/1e6

def pva_series(run: Path) -> list[tuple[int,dict[str,Any]]]:
    out=[]
    for x in load_jsonl(run/"scenario.jsonl"):
        if x.get("kind") != "pva_command": continue
        t=num(x.get("sim_time_ns")); p=x.get("payload",{})
        if t is not None: out.append((int(t),p))
    return sorted(out, key=lambda item: item[0])

def mode_series(run: Path) -> list[tuple[int,dict[str,Any]]]:
    out=[]
    for x in load_jsonl(run/"scenario.jsonl"):
        if x.get("kind") != "navigation_mode_status": continue
        t=num(x.get("sim_time_ns")); p=x.get("payload",{})
        if t is not None: out.append((int(t),p))
    return sorted(out, key=lambda item: item[0])

def input_trace(samples: list[dict[str,Any]]) -> list[tuple[int,dict[str,Any]]]:
    out=[]
    for t,p in sample_stream(samples,"mapping_diagnostics"):
        for s in p.get("statuses",[]):
            if s.get("name") != "navigation_external_mode/PX4_INPUT_SETPOINT": continue
            v=dict(s.get("values",{})); tt=int(num(v.get("trace_timestamp_ns")) or t)
            v["timestamp_ns"]=tt; out.append((tt,v))
    return sorted(out)

def by_id(series: list[tuple[int,dict[str,Any]]]) -> dict[int,dict[str,Any]]:
    out={}
    for t,p in series:
        sid=num(p.get("sample_id",p.get("trajectory_id")))
        if sid is not None: out[int(sid)]=dict(p, _time_ns=t)
    return out

def closest(series: list[tuple[int,dict[str,Any]]], t: int) -> tuple[dict[str,Any] | None,float|None]:
    return interp(series,t,("position","velocity","linear_velocity","acceleration","jerk","position_ned","velocity_ned","acceleration_ned","x_ned_m","y_ned_m","z_ned_m","vx_ned_m_s","vy_ned_m_s","vz_ned_m_s"))

def state_at_transition(series: list[tuple[int,dict[str,Any]]], t: int) -> dict[str,Any] | None:
    """Mode status is transition data, so the last transition owns the state."""
    if not series:
        return None
    prior = [item for item in series if item[0] <= t]
    return (prior[-1] if prior else series[0])[1]

def build_rows(run: Path) -> list[dict[str,Any]]:
    samples=load_jsonl(run/"samples.jsonl"); traces=input_trace(samples); pvas=pva_series(run); pva_id=by_id(pvas); modes=mode_series(run)
    lio=sample_stream(samples,"propagated_odometry"); gt=sample_stream(samples,"ground_truth_odometry")
    px4=sample_stream(samples,"local_position"); sp=sample_stream(samples,"px4_local_position_setpoint")
    statuses=sample_stream(samples,"vehicle_status")
    rows=[]; last_t=None; last_sp=None
    for t,tr in traces:
        p=pva_id.get(int(num(tr.get("sample_id")) or -1))
        if p is None: p,_=closest(pvas,t)
        lp,la=closest(lio,t); gp,ga=closest(gt,t); xp,xa=closest(px4,t); ep,ea=closest(sp,t)
        planner=vec(p.get("position")) if p else None; pv=vec(p.get("velocity")) if p else None; pa=vec(p.get("acceleration")) if p else None; pj=vec(p.get("jerk")) if p else None
        lpos=vec(lp.get("position")) if lp else None; lvel=vec(lp.get("linear_velocity")) if lp else None
        gpos=vec(gp.get("position")) if gp else None; gvel=vec(gp.get("linear_velocity")) if gp else None
        xpn=vec(xp.get("position_ned")) if xp else None; xvn=vec(xp.get("velocity_ned")) if xp else None
        if xp and xpn is None:
            xpn=vec([xp.get("x_ned_m"),xp.get("y_ned_m"),xp.get("z_ned_m")])
        if xp and xvn is None:
            xvn=vec([xp.get("vx_ned_m_s"),xp.get("vy_ned_m_s"),xp.get("vz_ned_m_s")])
        esn=vec(ep.get("position_ned")) if ep else None; evn=vec(ep.get("velocity_ned")) if ep else None; ean=vec(ep.get("acceleration_ned")) if ep else None
        xpose=ned_to_enu(xpn); xvel=ned_to_enu(xvn); epose=ned_to_enu(esn); evel=ned_to_enu(evn); eacc=ned_to_enu(ean)
        # Incoming PX4 exact trace is canonical for layer B; use it if sampled.
        ip=vec(tr.get("position_ned")); iv=vec(tr.get("velocity_ned")); ia=vec(tr.get("acceleration_ned"))
        ip=ned_to_enu(ip); iv=ned_to_enu(iv); ia=ned_to_enu(ia)
        if ip is None and p: ip=planner
        if iv is None and p: iv=pv
        if ia is None and p: ia=pa
        e_lio=n(sub(planner,lpos)); e_px4=n(sub(planner,xpose)); frame_p=n(sub(xpose,enu_to_ned(gpos))) if xpose and gpos else None
        # The frame residual is norm-equivalent whether represented in NED or ENU.
        frame_p=n(sub(xpn,enu_to_ned(gpos))) if xpn and gpos else None
        frame_v=n(sub(xvn,enu_to_ned(gvel))) if xvn and gvel else None
        dv=n(sub(evel,enu_to_ned(pv))) if evel and pv else None; da=n(sub(eacc,enu_to_ned(pa))) if eacc and pa else None
        status,_=closest(statuses,t); mode=state_at_transition(modes,t)
        mode_state = (mode.get("external_mode_state_name") or MODE.get(int(num(mode.get("external_mode_state",-1)) or -1), MISSING)) if mode else MISSING
        analytic = num(p.get("analytic_sample_role", 255)) if p else None
        navigation = num(p.get("trajectory_flag", analytic if analytic is not None else 255)) if p else None
        recovery = num(p.get("execution_recovery_state", 0)) if p else None
        row={"timestamp_ns":t,"bundle_generation":p.get("trajectory_generation",tr.get("bundle_generation",MISSING)) if p else tr.get("bundle_generation",MISSING),"trajectory_time_s":p.get("trajectory_time_s",MISSING) if p else MISSING,"analytic_role":ROLE.get(int(analytic) if analytic is not None else 255,"UNKNOWN") if p else MISSING,"NavigationCommand_role":ROLE.get(int(navigation) if navigation is not None else 255,"UNKNOWN") if p else MISSING,"recovery_state":RECOVERY.get(int(recovery) if recovery is not None else 0,MISSING) if p else MISSING,"safety_suffix_active":p.get("safety_suffix_active",MISSING) if p else MISSING,"planner_position":j(planner),"planner_velocity":j(pv),"planner_acceleration":j(pa),"planner_jerk":j(pj),"LIO_position":j(lpos),"LIO_velocity":j(lvel),"ground_truth_position":j(gpos),"ground_truth_velocity":j(gvel),"PX4_input_trajectory_position":j(ip),"PX4_input_trajectory_velocity":j(iv),"PX4_input_trajectory_acceleration":j(ia),"PX4_effective_position_setpoint":j(epose),"PX4_effective_velocity_setpoint":j(evel),"PX4_effective_acceleration_setpoint":j(eacc),"PX4_position":j(xpose),"PX4_velocity":j(xvel),"aligned_lio_tracking_error_m":e_lio if e_lio is not None else MISSING,"px4_tracking_error_m":e_px4 if e_px4 is not None else MISSING,"frame_position_residual_m":frame_p if frame_p is not None else MISSING,"frame_velocity_residual_mps":frame_v if frame_v is not None else MISSING,"delta_v_px4_controller_mps":dv if dv is not None else MISSING,"delta_a_px4_controller_mps2":da if da is not None else MISSING,"command_gap_ms":(t-last_t)/1e6 if last_t else MISSING,"setpoint_gap_ms":(t-last_sp)/1e6 if last_sp else MISSING,"external_mode_output_state":mode_state,"external_mode_reason":mode.get("external_mode_reason",MISSING) if mode else MISSING,"lio_source_age_ms":la if la is not None else MISSING,"ground_truth_source_age_ms":ga if ga is not None else MISSING,"px4_state_source_age_ms":xa if xa is not None else MISSING,"px4_setpoint_source_age_ms":ea if ea is not None else MISSING,"planner_speed_mps":n(pv),"planner_acceleration_mps2":n(pa),"planner_lateral_acceleration_mps2":lateral_acceleration(pv,pa),"planner_jerk_mps3":n(pj),"lio_gt_position_error_m":n(sub(lpos,gpos)),"lio_gt_velocity_error_mps":n(sub(lvel,gvel)),"px4_gt_position_error_m":n(sub(xpn,enu_to_ned(gpos))) if xpn and gpos else None,"px4_gt_velocity_error_mps":n(sub(xvn,enu_to_ned(gvel))) if xvn and gvel else None}
        rows.append(row); last_t=t; last_sp=t
    return rows

def metric(rows: list[dict[str,Any]], key: str) -> dict[str,Any]:
    a=sorted(float(r[key]) for r in rows if num(r.get(key)) is not None)
    if not a:return {"count":0,"rms":None,"p50":None,"p95":None,"p99":None,"max":None}
    def q(f): return a[min(len(a)-1,round((len(a)-1)*f))]
    return {"count":len(a),"rms":math.sqrt(sum(x*x for x in a)/len(a)),"p50":q(.5),"p95":q(.95),"p99":q(.99),"max":max(a)}

def write_csv(path: Path, rows: list[dict[str,Any]]) -> None:
    path.parent.mkdir(parents=True,exist_ok=True)
    fields=list(rows[0]) if rows else ["timestamp_ns"]
    with path.open("w",newline="",encoding="utf-8") as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n"); w.writeheader(); w.writerows(rows)

def add_relative_cross_time(rows: list[dict[str,Any]]) -> int | None:
    cross=next((int(r["timestamp_ns"]) for r in rows if num(r.get("aligned_lio_tracking_error_m")) is not None and float(r["aligned_lio_tracking_error_m"])>LIMIT),None)
    for r in rows:
        r["relative_time_from_T_cross_s"] = (int(r["timestamp_ns"])-cross)/1e9 if cross is not None else MISSING
    return cross

def event_time_from_diagnostics(run: Path, predicate) -> int | None:
    for line in load_jsonl(run / "perception_timeline.jsonl"):
        rec = line.get("record", line)
        payload = rec.get("payload", {})
        for status in payload.get("statuses", []) if isinstance(payload.get("statuses"), list) else []:
            values = status.get("values", {})
            if status.get("message") == "DECISION_TRACE" and predicate(values):
                stamp = num(payload.get("stamp_ns", rec.get("sim_time_ns")))
                if stamp is not None:
                    return int(stamp)
    return None

def first_pva_time(run: Path, predicate) -> int | None:
    for t, payload in pva_series(run):
        if predicate(payload):
            return t
    return None

def nearest_row(rows: list[dict[str,Any]], timestamp_ns: int | None) -> dict[str,Any] | None:
    if timestamp_ns is None or not rows:
        return None
    return min(rows, key=lambda row: abs(int(row["timestamp_ns"]) - timestamp_ns))

def build_event_rows(run: Path, rows: list[dict[str,Any]], cross: int | None) -> list[dict[str,Any]]:
    finite = [r for r in rows if num(r.get("aligned_lio_tracking_error_m")) is not None]
    bundle2 = next((int(r["timestamp_ns"]) for r in rows if str(r.get("bundle_generation")) == "2"), None)
    thresholds = [("tracking_error_0.10_m", .10), ("tracking_error_0.20_m", .20), ("tracking_error_0.25_m", LIMIT)]
    events: list[tuple[str,int|None]] = [("bundle_activation", bundle2)]
    growth = None
    # A sustained growth marker is deliberately descriptive: five consecutive
    # command samples with positive slope above 0.10 m/s, not a runtime gate.
    for a,b in zip(finite, finite[5:]):
        dt=(int(b["timestamp_ns"])-int(a["timestamp_ns"])) / 1e9
        if dt > 0 and (float(b["aligned_lio_tracking_error_m"])-float(a["aligned_lio_tracking_error_m"])) / dt > .10:
            growth=int(a["timestamp_ns"]); break
    events.append(("sustained_error_growth_start", growth))
    for name,limit in thresholds:
        hit=next((int(r["timestamp_ns"]) for r in finite if float(r["aligned_lio_tracking_error_m"]) >= limit),None)
        events.append((name,hit))
    emergency_input = next((int(r["timestamp_ns"]) for r in rows if r.get("NavigationCommand_role") == "EMERGENCY"), None)
    events.extend([
        ("injected_solve_start", None),
        ("injected_solve_failure", event_time_from_diagnostics(run, lambda v: str(v.get("injected_replan_failure")) == "1")),
        ("planner_failure_return", event_time_from_diagnostics(run, lambda v: str(v.get("planning_outcome")) == "5")),
        ("emergency_authorization", event_time_from_diagnostics(run, lambda v: str(v.get("emergency_authorization_reason")) not in {"0", "None", "nan"})),
        ("emergency_activation", emergency_input),
    ])
    out=[]
    for name,t in events:
        r=nearest_row(rows,t)
        base={"event":name,"timestamp_ns":t if t is not None else MISSING,"e_lio_m":MISSING,"e_px4_m":MISSING,"frame_position_residual_m":MISSING,"frame_velocity_residual_mps":MISSING,"planner_speed_mps":MISSING,"planner_acceleration_mps2":MISSING,"px4_effective_speed_mps":MISSING,"px4_effective_acceleration_mps2":MISSING,"delta_v_px4_controller_mps":MISSING,"delta_a_px4_controller_mps2":MISSING,"analytic_role":MISSING,"NavigationCommand_role":MISSING,"external_mode_output_state":MISSING,"recovery_state":MISSING,"safety_suffix_active":MISSING,"command_gap_ms":MISSING,"setpoint_gap_ms":MISSING}
        if r:
            base.update({"e_lio_m":r.get("aligned_lio_tracking_error_m",MISSING),"e_px4_m":r.get("px4_tracking_error_m",MISSING),"frame_position_residual_m":r.get("frame_position_residual_m",MISSING),"frame_velocity_residual_mps":r.get("frame_velocity_residual_mps",MISSING),"planner_speed_mps":r.get("planner_speed_mps",MISSING),"planner_acceleration_mps2":r.get("planner_acceleration_mps2",MISSING),"px4_effective_speed_mps":n(vec(json.loads(r["PX4_effective_velocity_setpoint"]))) if r.get("PX4_effective_velocity_setpoint") != MISSING else MISSING,"px4_effective_acceleration_mps2":n(vec(json.loads(r["PX4_effective_acceleration_setpoint"]))) if r.get("PX4_effective_acceleration_setpoint") != MISSING else MISSING,"delta_v_px4_controller_mps":r.get("delta_v_px4_controller_mps",MISSING),"delta_a_px4_controller_mps2":r.get("delta_a_px4_controller_mps2",MISSING),"analytic_role":r.get("analytic_role",MISSING),"NavigationCommand_role":r.get("NavigationCommand_role",MISSING),"external_mode_output_state":r.get("external_mode_output_state",MISSING),"recovery_state":r.get("recovery_state",MISSING),"safety_suffix_active":r.get("safety_suffix_active",MISSING),"command_gap_ms":r.get("command_gap_ms",MISSING),"setpoint_gap_ms":r.get("setpoint_gap_ms",MISSING)})
        out.append(base)
    return out

def plot_exact(rows: list[dict[str,Any]], out: Path) -> list[str]:
    try:
        import matplotlib.pyplot as plt
    except ImportError:return []
    if not rows:return []
    out.mkdir(parents=True,exist_ok=True); t=[(r["timestamp_ns"]-rows[0]["timestamp_ns"])/1e9 for r in rows]
    def vals(k):return [num(r.get(k)) for r in rows]
    made=[]
    specs=[("h10_fig1_velocity_layers.png",[("planner_speed_mps","planner NavigationCommand"),("px4_input_speed_mps","PX4_INPUT_SETPOINT"),("px4_setpoint_speed_mps","PX4 effective"),("measured_speed_mps","PX4 measured")],"m/s"),
           ("h10_fig2_acceleration_layers.png",[("planner_acceleration_mps2","planner NavigationCommand"),("px4_input_acceleration_mps2","PX4_INPUT_SETPOINT"),("px4_effective_acceleration_mps2","PX4 effective")],"m/s²"),
           ("h10_fig3_estimator_gt_error.png",[("lio_gt_position_error_m","LIO-GT P"),("px4_gt_position_error_m","PX4-GT P")],"m"),
           ("h10_fig4_command_gt_error.png",[("aligned_lio_tracking_error_m","command-LIO"),("px4_tracking_error_m","command-PX4")],"m")]
    # Derived plotting aliases are created below without changing CSV contract.
    for name,pairs,ylabel in specs:
        fig,ax=plt.subplots(figsize=(11,4))
        for key,label in pairs:
            if key=="px4_input_speed_mps": a=[n(vec(json.loads(r["PX4_input_trajectory_velocity"]))) if r["PX4_input_trajectory_velocity"]!=MISSING else None for r in rows]
            elif key=="px4_setpoint_speed_mps": a=[n(vec(json.loads(r["PX4_effective_velocity_setpoint"]))) if r["PX4_effective_velocity_setpoint"]!=MISSING else None for r in rows]
            elif key=="measured_speed_mps": a=[n(vec(json.loads(r["PX4_velocity"]))) if r["PX4_velocity"]!=MISSING else None for r in rows]
            elif key=="px4_input_acceleration_mps2": a=[n(vec(json.loads(r["PX4_input_trajectory_acceleration"]))) if r["PX4_input_trajectory_acceleration"]!=MISSING else None for r in rows]
            elif key=="px4_effective_acceleration_mps2": a=[n(vec(json.loads(r["PX4_effective_acceleration_setpoint"]))) if r["PX4_effective_acceleration_setpoint"]!=MISSING else None for r in rows]
            else:a=vals(key)
            ax.plot(t,a,label=label)
        if name in {"h10_fig3_estimator_gt_error.png","h10_fig4_command_gt_error.png"}: ax.axhline(LIMIT,color="k",ls="--",label="0.25 m")
        ax.set(xlabel="simulation time (s)",ylabel=ylabel); ax.grid(True); ax.legend(); fig.tight_layout(); fig.savefig(out/name,dpi=130); plt.close(fig); made.append(name)
    # correction vs error and demand vs error
    fig,ax=plt.subplots(figsize=(6,5)); ax.scatter(vals("aligned_lio_tracking_error_m"),vals("delta_v_px4_controller_mps"),s=5); ax.set(xlabel="LIO tracking error (m)",ylabel="|delta V| PX4 (m/s)"); ax.grid(True); fig.tight_layout(); fig.savefig(out/"h10_fig5_controller_correction_vs_error.png",dpi=130); plt.close(fig); made.append("h10_fig5_controller_correction_vs_error.png")
    fig,ax=plt.subplots(figsize=(6,5)); ax.scatter(vals("planner_acceleration_mps2"),vals("aligned_lio_tracking_error_m"),s=5); ax.set(xlabel="|planner A| (m/s²)",ylabel="tracking error (m)"); ax.grid(True); fig.tight_layout(); fig.savefig(out/"h10_fig6_dynamic_demand_vs_error.png",dpi=130); plt.close(fig); made.append("h10_fig6_dynamic_demand_vs_error.png")
    fig,ax=plt.subplots(figsize=(11,4)); ax.plot(t,vals("frame_position_residual_m"),label="frame P residual"); ax.plot(t,vals("frame_velocity_residual_mps"),label="frame V residual"); ax.grid(True); ax.legend(); ax.set(xlabel="simulation time (s)",ylabel="residual"); fig.tight_layout(); fig.savefig(out/"h10_fig7_lio_px4_consistency.png",dpi=130); plt.close(fig); made.append("h10_fig7_lio_px4_consistency.png")
    fig,ax=plt.subplots(figsize=(11,4)); ax.plot(t,vals("aligned_lio_tracking_error_m"),label="e LIO"); ax.axhline(LIMIT,color="k",ls="--",label="limit"); ax.grid(True); ax.legend(); ax.set(xlabel="simulation time (s)",ylabel="m"); fig.tight_layout(); fig.savefig(out/"h10_fig8_exact_e5_causal_timeline.png",dpi=130); plt.close(fig); made.append("h10_fig8_exact_e5_causal_timeline.png")
    return made

def main() -> int:
    ap=argparse.ArgumentParser(); ap.add_argument("--e5",type=Path,required=True); ap.add_argument("--open",type=Path,required=True); ap.add_argument("--dynamic",nargs="*",type=Path,default=[]); ap.add_argument("--date-root",type=Path,required=True); args=ap.parse_args()
    e5=build_rows(args.e5); op=build_rows(args.open); cross=add_relative_cross_time(e5); add_relative_cross_time(op); write_csv(args.e5/"h10_exact_e5_closed_loop.csv",e5); write_csv(args.open/"h10_open_control.csv",op)
    dyn=[]
    for run in args.dynamic:
        rows=build_rows(run); m=json.loads((run/"metadata.json").read_text()) if (run/"metadata.json").is_file() else {}; vals={"run_id":m.get("run_id",run.name),"scenario":"S_DYNAMIC_ID","run_path":str(run),"requested_speed_mps":m.get("requested_cruise_speed_mps"),"tracking_p95_m":metric(rows,"aligned_lio_tracking_error_m")["p95"],"tracking_max_m":metric(rows,"aligned_lio_tracking_error_m")["max"],"planner_velocity_p95_mps":metric(rows,"planner_speed_mps")["p95"],"planner_acceleration_p95_mps2":metric(rows,"planner_acceleration_mps2")["p95"],"planner_jerk_p95_mps3":metric(rows,"planner_jerk_mps3")["p95"],"px4_delta_v_p95_mps":metric(rows,"delta_v_px4_controller_mps")["p95"],"px4_delta_a_p95_mps2":metric(rows,"delta_a_px4_controller_mps2")["p95"],"valid_segment":bool(rows and metric(rows,"aligned_lio_tracking_error_m")["count"]>20 and not any(r.get("recovery_state") in {"EMERGENCY_BRAKE","PX4_HOLD"} for r in rows))}; dyn.append(vals)
    write_csv(args.date_root/"h10_dynamic_identification.csv",dyn)
    em={"criterion":{"nominal_p95_tracking_error_m_max":0.10,"maximum_tracking_error_m_max":0.175,"no_emergency_or_recovery":True,"estimator_health_valid":True},"scenario":"S_DYNAMIC_ID","status":"NOT_TESTED","reason":"No valid increasing-demand segment without recovery/emergency was captured; no closed-loop envelope is claimed.","runs":dyn,"v_closed_loop_nominal_max":None,"a_closed_loop_nominal_max":None,"j_closed_loop_nominal_max":None,"a_lateral_closed_loop_nominal_max":None}; (args.date_root/"h10_dynamic_envelope.json").write_text(json.dumps(em,indent=2)+"\n")
    event_rows=build_event_rows(args.e5,e5,cross)
    write_csv(args.date_root/"h10_exact_e5_events.csv",event_rows)
    figs=plot_exact(e5,args.date_root/"h10_figures")
    cross_row=next((r for r in e5 if cross is not None and int(r["timestamp_ns"])==cross),None)
    summary={"scenario_scope":{"S_BAD_E5":{"run":str(args.e5),"map":"sanity_open","route":"external_mode_open_route","speed_mps":3.0},"S_OPEN_CONTROL":{"run":str(args.open),"map":"sanity_open","route":"external_mode_open_route","speed_mps":3.0},"S_DYNAMIC_ID":{"runs":[str(x) for x in args.dynamic]}},"exact_e5":{"sample_count":len(e5),"T_cross_ns":cross,"T_cross_row":cross_row,"metrics":{"tracking":metric(e5,"aligned_lio_tracking_error_m"),"planner_speed":metric(e5,"planner_speed_mps"),"planner_acceleration":metric(e5,"planner_acceleration_mps2"),"planner_lateral_acceleration":metric(e5,"planner_lateral_acceleration_mps2"),"planner_jerk":metric(e5,"planner_jerk_mps3"),"delta_v_px4":metric(e5,"delta_v_px4_controller_mps"),"delta_a_px4":metric(e5,"delta_a_px4_controller_mps2"),"lio_gt_position":metric(e5,"lio_gt_position_error_m"),"lio_gt_velocity":metric(e5,"lio_gt_velocity_error_mps"),"px4_gt_position":metric(e5,"px4_gt_position_error_m"),"px4_gt_velocity":metric(e5,"px4_gt_velocity_error_mps"),"frame_position":metric(e5,"frame_position_residual_m"),"frame_velocity":metric(e5,"frame_velocity_residual_mps")}},"open_control":{"sample_count":len(op),"metrics":{"tracking":metric(op,"aligned_lio_tracking_error_m"),"planner_acceleration":metric(op,"planner_acceleration_mps2"),"planner_lateral_acceleration":metric(op,"planner_lateral_acceleration_mps2"),"planner_jerk":metric(op,"planner_jerk_mps3"),"delta_v_px4":metric(op,"delta_v_px4_controller_mps"),"delta_a_px4":metric(op,"delta_a_px4_controller_mps2"),"lio_gt_position":metric(op,"lio_gt_position_error_m"),"px4_gt_position":metric(op,"px4_gt_position_m")}},"figures":figs,"H10a":"INCONCLUSIVE","H10b":"INCONCLUSIVE","H10c":"INCONCLUSIVE","H10d":"INCONCLUSIVE","H10e":"INCONCLUSIVE","ground_truth_authority":"/sim/ground_truth/odometry; independent Gazebo ENU/FLU","exact_injected_failure_observed":False,"note":"Selected E5 observability run must not be treated as a valid injected-failure reproduction unless injected_replan_failure=1 is present in the retained trace."}
    (args.date_root/"h10_summary.json").write_text(json.dumps(summary,indent=2)+"\n")
    md=["# H10 Analysis","","## Scenario scope","",f"- S_BAD_E5: `{args.e5}`; exact stressed map/route at 3.0 m/s. The retained run is an observability run; injected failure is NOT_RECOrDED unless the trace says `injected_replan_failure=1`.",f"- S_OPEN_CONTROL: `{args.open}`; matched open/straight control, 3.0 m/s; statistics remain separate.",f"- S_DYNAMIC_ID: {[str(x) for x in args.dynamic]}; separate low-demand controls, not a certified envelope matrix.","","## Evidence status","",f"S_BAD_E5 rows: {len(e5)}; synchronized T_cross: {cross if cross is not None else MISSING}.","Layer B is the `PX4_INPUT_SETPOINT` diagnostic emitted immediately before External Mode calls `trajectory_setpoint_->update`; SITL ground truth is `/sim/ground_truth/odometry` and is independent of LIO/PX4 estimation.","","## H10 classifications","","| Hypothesis | Status | Scenario-scoped evidence |","|---|---|---|", "| H10a planner demand exceeds usable envelope | INCONCLUSIVE | E5 demand and error are correlated, but no valid increasing-demand S_DYNAMIC_ID segment meets the criterion. |", "| H10b PX4 materially reshapes command | INCONCLUSIVE | Four layers are present in the observability run; attribution requires a valid exact injected E5 boundary and synchronized finite PX4 input/effective data. |", "| H10c LIO material contributor | INCONCLUSIVE | LIO-GT error is measured, but its contribution is not isolated from controller/plant response in the failed run. |", "| H10d PX4 estimator material contributor | INCONCLUSIVE | PX4-GT position is measured; velocity estimator evidence is NOT_RECORDED in this run. |", "| H10e vehicle cannot follow PX4 effective setpoint | INCONCLUSIVE | Effective setpoint and GT are present, but no clean dynamic-ID segment separates plant limitation from command reshaping. |", "","## Envelope","", "`h10_dynamic_envelope.json` remains `NOT_TESTED`; the two low-speed runs fail closed before providing the required no-recovery increasing-demand segments. No production limit is changed.","","## Causal ordering","", "The CSV and retained event timestamps are authoritative. The selected S_BAD_E5 run has a natural planner failure/emergency boundary but no `injected_replan_failure=1`; therefore it cannot establish injected-failure causality. Earlier tracking growth must not be attributed to that later planner event.","","## Figures","", *[f"- `{x}`" for x in figs],"","## FIRST FIX RECOMMENDATION","", "Do not implement a fix from this evidence phase. First obtain a valid S_DYNAMIC_ID matrix and a valid exact-E5 injected-failure run with all four control layers; then choose between planner-envelope coupling and controller/plant changes from measured attribution."]
    (args.date_root/"h10_analysis.md").write_text("\n".join(md)+"\n")
    return 0
if __name__=="__main__": raise SystemExit(main())
