"""Audit-only exact-identity join. Authorization→setpoint is a bound, not transport latency."""
import csv
import json
import math
from pathlib import Path
import sys

TARGET = "7da3e97cb399c2e39d62cfe60213a45e8a92300e"
KEY_FIELDS = ("runtime_instance_id", "localization_epoch", "goal_epoch", "request_id", "bundle_generation", "sample_id")


def key(payload):
    values = tuple(payload.get(k) for k in KEY_FIELDS)
    if not values[0] or any(v is None for v in values[1:]):
        return None
    return values


def paired_duration_ms(start_ns, start_domain, end_ns, end_domain):
    if start_domain != end_domain or start_ns <= 0 or end_ns <= 0 or end_ns < start_ns:
        return None
    return (end_ns-start_ns)/1e6


def records(path):
    if not path.exists():
        return
    with path.open() as f:
        for line in f:
            yield json.loads(line)["record"]


def rank(xs, q):
    return xs[max(0, math.ceil(q * len(xs)) - 1)] if xs else ""


def summarize(run, profile, classification, metric, samples, n_commands, n_traces, duplicates, unmatched, invalid):
    xs = sorted(samples)
    return dict(run=run, profile=profile, infrastructure=classification, metric=metric,
                n=len(xs), commands=n_commands, tracking_trace_rows=n_traces,
                duplicate_command_keys=duplicates, unmatched_tracking_traces=unmatched,
                invalid_clock_pairs=invalid, p50_ms=rank(xs,.50), p95_ms=rank(xs,.95),
                p99_ms=rank(xs,.99), p999_ms=rank(xs,.999) if len(xs)>=1000 else "",
                max_ms=xs[-1] if xs else "", gt20=sum(x>20 for x in xs),
                gt40=sum(x>40 for x in xs), gt60=sum(x>60 for x in xs),
                gt80=sum(x>80 for x in xs), ge100=sum(x>=100 for x in xs))


def analyze_run(run):
    meta = json.loads((run/"metadata.json").read_text())
    if meta.get("repo_commit") != TARGET or meta.get("repo_dirty"):
        return []
    profile = meta.get("scenario_identity",{}).get("resolved",{}).get("profile","")
    report = json.loads((run/"report.json").read_text())
    classification = report.get("infrastructure",{}).get("classification","")
    commands = {}
    duplicates = 0
    command_observer = []
    for rec in records(run/"planning_timeline.jsonl"):
        if rec.get("kind") != "pva_command":
            continue
        p=rec.get("payload",{}); k=key(p)
        if k is None: continue
        if k in commands:
            duplicates += 1
            continue
        commands[k]=p
        command_observer.append((rec.get("observer_record_steady_ns",0),k))
    first_use={}; repeated=[]; unmatched=0; invalid=0; n_traces=0
    for rec in records(run/"execution_timeline.jsonl"):
        if rec.get("kind") != "px4_input_trace": continue
        p=rec.get("payload",{})
        if p.get("setpoint_kind") != "tracking": continue
        n_traces += 1
        k=key(p)
        if k not in commands:
            unmatched += 1
            continue
        auth=int(commands[k].get("execution_authorization_steady_ns") or 0)
        start=int(p.get("trace_values",{}).get("update_start_steady_ns") or 0)
        delay=paired_duration_ms(auth,'host_steady',start,'host_steady')
        if delay is None:
            invalid += 1
            continue
        if k in first_use:
            repeated.append(delay)
        else:
            first_use[k]=(start,delay)
    ordered=sorted(first_use.values())
    first_delay=[d for _,d in ordered]
    adapter_first_gap=[(ordered[i][0]-ordered[i-1][0])/1e6 for i in range(1,len(ordered))]
    obs=sorted((t,k) for t,k in command_observer if t>0)
    recorder_gap=[(obs[i][0]-obs[i-1][0])/1e6 for i in range(1,len(obs))]
    measures={"authorization_to_first_tracking_setpoint_upper_bound_ms":first_delay,
              "authorization_to_reused_tracking_setpoint_age_ms":repeated,
              "first_tracking_setpoint_interarrival_ms":adapter_first_gap,
              "runner_command_observer_interarrival_ms":recorder_gap}
    return [summarize(run.name,profile,classification,name,xs,len(commands),n_traces,duplicates,unmatched,invalid)
            for name,xs in measures.items()]


def main():
    root,out=map(Path,sys.argv[1:3]); rows=[]
    for run in sorted(root.glob("*20260922*")):
        if (run/"metadata.json").exists() and (run/"report.json").exists():
            rows += analyze_run(run)
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]) if rows else [],lineterminator='\n')
        w.writeheader();w.writerows(rows)
    print(json.dumps({"runs":len({r['run'] for r in rows}),"rows":len(rows),
                      "total_unique_commands":sum(r['commands'] for r in rows if r['metric'].startswith('authorization_to_first')),
                      "unmatched_traces":sum(r['unmatched_tracking_traces'] for r in rows if r['metric'].startswith('authorization_to_first'))},indent=2))


if __name__=="__main__":main()
