"""Compact Hold-related observations; preserves incompatible clock domains."""
import csv
import json
from pathlib import Path
import re
import sys


def main():
    root,out=map(Path,sys.argv[1:3]);rows=[]
    for run in sorted(root.glob("*20260922*")):
        meta_path=run/"metadata.json"
        if not meta_path.exists():continue
        meta=json.loads(meta_path.read_text())
        if meta.get("repo_commit")!="7da3e97cb399c2e39d62cfe60213a45e8a92300e" or meta.get("repo_dirty"):continue
        base=dict(run=run.name,source="",event="",sim_time_ns="",observer_steady_ns="",px4_boot_us="",log_wall_s="",nav_state="",executor_in_charge="",failsafe="",result="",detail="")
        for path in sorted((run/"logs").glob("px4_navigation_external_mode_node*")):
            for line in path.read_text(errors="replace").splitlines():
                if not any(s in line for s in ("PX4 Hold handover","executor deactivated","Avoidance Mission completed")):continue
                match=re.search(r"\[(\d+\.\d+)\]",line)
                event=("hold_request_log" if "requesting PX4 Hold" in line else "hold_completion_callback_log" if "PX4 Hold handover completed" in line else "hold_attempt_failure_log" if "PX4 Hold handover attempt" in line else "executor_deactivated_log" if "executor deactivated" in line else "mission_completed_log")
                x={**base,"source":"adapter_log","event":event,"log_wall_s":match.group(1) if match else "","result":"Deactivated" if "result=Deactivated" in line else "Success" if "result=Success" in line else "","detail":line[-240:]}
                rows.append(x)
        path=run/"execution_timeline.jsonl"
        if path.exists():
            for line in path.open():
                r=json.loads(line)["record"]
                if r.get("kind")!="vehicle_status":continue
                p=r.get("payload",{})
                rows.append({**base,"source":"runner_vehicle_status_change","event":"vehicle_status_change","sim_time_ns":r.get("sim_time_ns",""),"observer_steady_ns":r.get("observer_record_steady_ns",""),"px4_boot_us":p.get("timestamp_us",""),"nav_state":p.get("nav_state",""),"executor_in_charge":p.get("executor_in_charge",""),"failsafe":p.get("failsafe","")})
        path=run/"planning_timeline.jsonl"
        if path.exists():
            for line in path.open():
                r=json.loads(line)["record"];p=r.get("payload",{})
                if r.get("kind")=="event" and p.get("name") in ("px4_hold_handover_requested","external_mode_exit_observed","external_mode_entered"):
                    rows.append({**base,"source":"runner_event","event":p["name"],"sim_time_ns":r.get("sim_time_ns",""),"observer_steady_ns":r.get("observer_record_steady_ns",""),"detail":json.dumps(p.get("detail",{}),sort_keys=True)})
    fields=list(rows[0])
    with out.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");w.writeheader();w.writerows(rows)
    print(f"runs={len(set(r['run'] for r in rows))} rows={len(rows)}")


if __name__=="__main__":main()
