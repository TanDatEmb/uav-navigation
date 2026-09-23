"""Produce per-run timing summaries without combining incompatible clocks.

Diagnostic duration fields are samples of the last reported operation, not an
event-paired end-to-end latency. Percentiles use nearest rank. No metric here
measures runtime publisher to adapter callback receipt.
"""
import csv
import json
import math
from pathlib import Path
import sys


def events(path):
    with path.open() as f:
        for line in f:
            yield json.loads(line)["record"]


def number(value):
    try:
        v = float(value)
        return v if math.isfinite(v) else None
    except (TypeError, ValueError):
        return None


def percentile(xs, p):
    return xs[max(0, math.ceil(p * len(xs)) - 1)]


def read_csv(path):
    with path.open() as f:
        return list(csv.DictReader(f))


def main():
    runs_root, raw_root, out_path = map(Path, sys.argv[1:4])
    out = []
    for run in sorted(runs_root.glob("*20260922*")):
        meta_path = run / "metadata.json"
        if not meta_path.exists(): continue
        meta = json.loads(meta_path.read_text())
        if meta.get("repo_commit") != "7da3e97cb399c2e39d62cfe60213a45e8a92300e" or meta.get("repo_dirty"):
            continue
        name = run.name
        raw = raw_root / name
        profile = meta.get("scenario_identity",{}).get("resolved",{}).get("profile","")
        report = json.loads((run / "report.json").read_text())
        infrastructure = report.get("infrastructure",{}).get("classification","")
        run_verdict = report.get("verdict","")
        vals = {}
        def add(k, v):
            n = number(v)
            if n is not None and n >= 0:
                vals.setdefault(k, []).append(n)

        command_path = raw / "command.csv"
        if command_path.exists():
            commands = read_csv(command_path)
            prev = None
            for c in commands:
                add("world_source_age_at_command_stamp_ms", (int(c["source_ns"])-int(c["world_source_ns"]))/1e6)
                if prev and c["epoch"] == prev["epoch"]:
                    add("command_source_interarrival_ms",(int(c["source_ns"])-int(prev["source_ns"]))/1e6)
                    add("command_recorder_interarrival_ms",(int(c["bag_ns"])-int(prev["bag_ns"]))/1e6)
                prev=c

        for r in events(run / "planning_timeline.jsonl"):
            if r.get("kind") != "planner_trace": continue
            v=r.get("payload",{}).get("values",{})
            for source, key, scale in [
                ("active_revalidation_us","active_revalidation_diagnostic_ms",1e-3),
                ("pending_revalidation_us","pending_revalidation_diagnostic_ms",1e-3),
                ("command_store_publish_us","command_store_publish_diagnostic_ms",1e-3),
                ("command_transport_publish_us","command_transport_publish_diagnostic_ms",1e-3),
                ("planning_scheduling_gap_us","planning_scheduling_gap_diagnostic_ms",1e-3),
                ("planning_worker_runtime_us","planning_worker_runtime_diagnostic_ms",1e-3),
            ]:
                n=number(v.get(source))
                if n is not None and n >= 0: add(key,n*scale)
            for source in ["command_publication_deadline_miss_count","world_freshness_command_suspend_count","world_freshness_command_recovery_count"]:
                n=number(v.get(source))
                if n is not None: vals.setdefault("COUNTER_LAST_"+source,[]).append(n)

        last_mapping_identity = None
        for r in events(run / "perception_timeline.jsonl"):
            if r.get("stream") != "mapping_diagnostics": continue
            p=r.get("payload",{})
            statuses=p.get("statuses",[])
            status=next((s for s in statuses if s.get("name")=="navigation_mapping/world_model"),None)
            if not status: continue
            v=status.get("values",{})
            if str(v.get("world_snapshot_published")) not in ("1","True","true"): continue
            identity=(v.get("world_generation"),v.get("world_revision"),v.get("observation_stamp_ns"))
            if identity==last_mapping_identity: continue
            last_mapping_identity=identity
            for source,key,scale in [
                ("mapping_callback_total_us","mapping_callback_diagnostic_ms",1e-3),
                ("snapshot_export_us","mapping_snapshot_export_diagnostic_ms",1e-3),
            ]:
                n=number(v.get(source))
                if n is not None and n >= 0:add(key,n*scale)
            stamp=number(p.get("stamp_ns")); observation=number(v.get("observation_stamp_ns"))
            if stamp is not None and observation is not None:
                add("mapping_observation_to_diagnostic_stamp_proxy_ms",(stamp-observation)/1e6)

        for r in events(run / "execution_timeline.jsonl"):
            if r.get("kind") != "px4_input_trace": continue
            p=r.get("payload",{});tv=p.get("trace_values",{})
            tracking = p.get("setpoint_kind") == "tracking"
            update_duration = number(p.get("setpoint_update_duration_ns"))
            add("adapter_setpoint_update_duration_ms",update_duration/1e6 if update_duration is not None else None)
            if tracking: add("adapter_setpoint_update_duration_tracking_ms",update_duration/1e6 if update_duration is not None else None)
            state=number(tv.get("state_source_stamp_ros_ns")); update=number(tv.get("update_start_ros_ns"))
            if state is not None and update is not None:
                add("adapter_state_source_age_at_update_ms",(update-state)/1e6)
                if tracking: add("adapter_state_source_age_at_update_tracking_ms",(update-state)/1e6)
            recv=number(tv.get("state_receive_steady_ns")); start=number(tv.get("update_start_steady_ns"))
            if recv is not None and start is not None:
                add("adapter_state_receive_to_update_ms",(start-recv)/1e6)

        for metric, xs in sorted(vals.items()):
            if not xs: continue
            if metric.startswith("COUNTER_LAST_"):
                out.append(dict(run=name,profile=profile,infrastructure=infrastructure,run_verdict=run_verdict,metric=metric,clock_domain="counter",scope="last observed cumulative counter",n=len(xs),min="",p50="",p95="",p99="",p999="",max=xs[-1],mean="",over_100_ms=""))
                continue
            xs.sort()
            domain=("ROS source time" if metric in ("command_source_interarrival_ms","world_source_age_at_command_stamp_ms","mapping_observation_to_diagnostic_stamp_proxy_ms","adapter_state_source_age_at_update_ms") else "recorder wall time" if metric=="command_recorder_interarrival_ms" else "steady duration" if metric=="adapter_state_receive_to_update_ms" else "diagnostic duration")
            out.append(dict(run=name,profile=profile,infrastructure=infrastructure,run_verdict=run_verdict,metric=metric,clock_domain=domain,scope="per run; diagnostic samples not event-paired",n=len(xs),min=xs[0],p50=percentile(xs,.5),p95=percentile(xs,.95),p99=percentile(xs,.99),p999=percentile(xs,.999) if len(xs)>=1000 else "",max=xs[-1],mean=sum(xs)/len(xs),over_100_ms=sum(x>100 for x in xs) if "interarrival" in metric else ""))
    fields=["run","profile","infrastructure","run_verdict","metric","clock_domain","scope","n","min","p50","p95","p99","p999","max","mean","over_100_ms"]
    with out_path.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=fields,lineterminator="\n");w.writeheader();w.writerows(out)
    print(f"runs={len(set(x['run'] for x in out))} rows={len(out)}")


if __name__=="__main__": main()
