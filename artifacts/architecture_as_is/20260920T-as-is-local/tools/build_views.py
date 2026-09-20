#!/usr/bin/env python3
"""Generate offline tables, evidence excerpts and Graphviz/Mermaid views from architecture.json."""
import argparse, csv, html, json, pathlib, subprocess, sys, textwrap

HERE = pathlib.Path(__file__).resolve().parent
OUT = HERE.parent
ROOT = OUT.parents[2]
MODEL = OUT / "model/architecture.json"

def link_evidence(ids):
    return ", ".join(f"[<code>{html.escape(i)}</code>](../evidence/refs.html#{html.escape(i)})" for i in ids)
def html_evidence(ids):
    return ", ".join(f"<a href='../evidence/refs.html#{html.escape(i)}'>{html.escape(i)}</a>" for i in ids)

def md_table(headers, rows):
    return "| " + " | ".join(headers) + " |\n|" + "|".join(["---"]*len(headers)) + "|\n" + "\n".join("| " + " | ".join(str(c).replace("|", "\\|").replace("\n", " ") for c in row) + " |" for row in rows) + "\n"

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument("--skip-render", action="store_true")
    a=ap.parse_args()
    m=json.loads(MODEL.read_text())
    manifest=json.loads((OUT/"baseline/source_manifest.json").read_text())
    files={x["path"]:x for x in manifest["files"]}
    snap=OUT/"baseline/source_snapshot"
    ev={x["id"]:x for x in m["evidence_refs"]}
    # Capture compact, line-numbered excerpts from the exact bytes in the baseline snapshot.
    refs=[]
    for r in m["evidence_refs"]:
        item=files.get(r["path"])
        if not item: raise SystemExit(f"evidence {r['id']} path not in snapshot: {r['path']}")
        source=snap/r["path"]
        lines=source.read_text(errors="replace").splitlines()
        if r["start"] < 1 or r["end"] < r["start"] or r["end"] > len(lines):
            raise SystemExit(f"invalid evidence range {r['id']}: {r['start']}-{r['end']} / {len(lines)}")
        # Keep one contextual excerpt; larger ranges are clipped to 18 lines with an explicit notice.
        selected=r.get("segments", [[r["start"], min(r["end"], r["start"]+17)]])
        excerpt_parts=[]
        for seg_start,seg_end in selected:
            if seg_start < r["start"] or seg_end > r["end"] or seg_start < 1 or seg_end < seg_start or seg_end > len(lines):
                raise SystemExit(f"invalid evidence segment {r['id']}: {seg_start}-{seg_end}")
            # Bound each displayed block while preserving the explicitly selected line coverage.
            chunk_start=seg_start
            while chunk_start <= seg_end:
                chunk_end=min(seg_end,chunk_start+24)
                excerpt_parts.append("\n".join(f"{n:>6} | {lines[n-1]}" for n in range(chunk_start,chunk_end+1)))
                chunk_start=chunk_end+1
        excerpt="\n\n…\n\n".join(excerpt_parts)
        refs.append({"id":r["id"],"path":r["path"],"symbol":r["symbol"],"line_start":r["start"],"line_end":r["end"],"excerpt_segments":selected,"sha256":item["sha256"],"claim":r["claim"],"excerpt":excerpt})
    (OUT/"evidence/refs.jsonl").write_text("".join(json.dumps(x,ensure_ascii=False)+"\n" for x in refs))
    # Inventory views.
    (OUT/"tables/objects.md").write_text("# Object and process inventory\n\n"+md_table(["ID","Symbol / instance","Lifetime owner","Decision authority","Evidence"],[[o["id"],f"`{o['symbol']}` — {o['instance']}",o["lifetime_owner"],o["decision_authority"],link_evidence(o["evidence"])] for o in m["objects"]]))
    fields=m["fields"]
    with (OUT/"tables/fields.csv").open("w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=["id","object","name","type","initialization","meaning","scope","writers","readers","reset","clock_unit","identity","protection","roles","evidence"],lineterminator="\n")
        w.writeheader()
        for x in fields:
            w.writerow({**x,"writers":"; ".join(x["writers"]),"readers":"; ".join(x["readers"]),"roles":"; ".join(x["roles"]),"evidence":"; ".join(x["evidence"])})
    (OUT/"tables/fields.md").write_text("# Critical field inventory\n\nThis is the behavior-relevant field set traced in this run, not an exhaustive inventory of every private member. Role labels may overlap.\n\n"+md_table(["Field ID","Owner.field / type","Observed meaning and lifetime","Writer → reader; reset/protection","Identity / clock / role","Evidence"],[[x["id"],f"`{x['object']}.{x['name']}` · `{x['type']}`",f"{x['meaning']} Scope: {x['scope']}. Init: {x['initialization']}.",f"W: {'; '.join(x['writers'])} → R: {'; '.join(x['readers'])}. Reset: {x['reset']}. Lock: {x['protection']}.",f"Identity: {x['identity']}. Clock: {x['clock_unit']}. Role: {', '.join(x['roles'])}.",link_evidence(x["evidence"])] for x in fields]))
    (OUT/"tables/transitions.md").write_text("# Transition inventory\n\nA transition record can span callbacks/objects. Separate records represent non-atomic boundaries. Branch detail is concise; source links open captured excerpts.\n\n"+md_table(["ID / object","Entry point · event","Predicate / guard · priority","Read set → ordered write set","Side effects / command effect","Concurrency / evidence"],[[x["id"]+" · "+x["object"],f"`{x['entrypoint']}` · {x['event']}",f"{x['predicate_before']} Guard: {x['guard']} Order: {x['branch_priority']}",f"R: {', '.join(x['read_set'])} → W: {', '.join(x['write_set'])}","; ".join(x["effects"]),f"{x['concurrency']} {link_evidence(x['evidence'])}"] for x in m["transitions"]]))
    trans_html=['<!doctype html><meta charset="utf-8"><title>Transitions</title><style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:0 1rem}.row{border-top:1px solid #bbb;padding:1rem 0;scroll-margin-top:1rem}pre{white-space:pre-wrap;background:#f5f5f5;padding:.6rem}a{color:#0645ad}</style><h1>Transition records</h1>']
    for t in m["transitions"]:
        trans_html.append(f"<section id='{html.escape(t['id'])}' class='row'><h2>{html.escape(t['id'])}: {html.escape(t['event'])}</h2><p><b>Owner/entry:</b> {html.escape(t['object'])}; {html.escape(t['entrypoint'])}</p><p><b>Before/guard:</b> {html.escape(t['predicate_before'])} Guard: {html.escape(t['guard'])}</p><p><b>Read → write:</b> {html.escape(', '.join(t['read_set']))} → {html.escape(', '.join(t['write_set']))}</p><p><b>Effects/order:</b> {html.escape('; '.join(t['effects']))} Priority: {html.escape(t['branch_priority'])}</p><p><b>Concurrency:</b> {html.escape(t['concurrency'])}</p><p>Evidence: {html_evidence(t['evidence'])}</p></section>")
    (OUT/"tables/transitions.html").write_text("\n".join(trans_html)+"\n")
    (OUT/"tables/decisions.md").write_text("# Decision authority matrix\n\n"+md_table(["Decision ID / sink","Authority","Participating fields","Effect","Evidence"],[[x["id"]+" / "+x["sink"],x["authority"],", ".join(x["participants"]),x["effects"],link_evidence(x["evidence"])] for x in m["decisions"]]))
    (OUT/"tables/scenarios.md").write_text("# Scenario traces\n\nEvidence class: `FACT_FROM_CODE` describes inspected source path; `TEST_SUPPORTED` points to assertion sites read but not run; `RUNTIME_OBSERVED` is absent in this run; `INFERENCE/UNRESOLVED` marks reachability/behavior not proven.\n\n"+md_table(["ID / scenario","Pre-state and event order","Branch / writes","Command and post-state","Status / evidence"],[[x["id"]+" / "+x["name"],x["pre_state"]+" Events: "+" → ".join(x["events"]),x["branch"],x["post_state"],x["status"]+"; "+link_evidence(x["evidence"])] for x in m["scenarios"]]))
    (OUT/"tables/authority_findings.md").write_text("# Cross-object authority findings\n\nThese findings identify discussion points, not proposed replacement architecture. A distributed representation is not automatically a defect; identities, lifetimes and serialization can make it legitimate.\n\n"+md_table(["ID / class / priority","Decision and participating fields","Evidence for overlap","Mechanism that can make it valid","Synchronization","Demonstrated effect / risk hypothesis","Test and gap","Evidence"],[[x["id"]+" / "+x["classification"]+" / "+x["priority"],x["decision"]+" Fields: "+", ".join(x["objects_fields"]),x["supporting_evidence"],x["counterexample_or_validity_mechanism"],x["synchronization"],x["impact"],x["tests"],link_evidence(x["evidence"])] for x in m["authority_findings"]]))
    (OUT/"tables/cross_layer_combinations.md").write_text("# Important cross-layer combinations\n\nA combination without direct observed evidence is not called impossible.\n\n"+md_table(["ID","State tuple","Reachability status","Reason / evidence"],[[x["id"],x["tuple"],x["status"],x["why"]+" "+link_evidence(x["evidence"])] for x in m["important_cross_layer_combinations"]]))
    cov=m["coverage"]
    (OUT/"tables/coverage.md").write_text("# Coverage and limits\n\n"+md_table(["Measure","Captured result"],[[k,str(v)] for k,v in cov.items()]+[["objects modeled",str(len(m["objects"]))],["fields modeled",str(len(m["fields"]))],["transitions modeled",str(len(m["transitions"]))],["decision sinks modeled",str(len(m["decisions"]))],["scenarios addressed",str(len(m["scenarios"]))],["source evidence references",str(len(m["evidence_refs"]))],["package manifests inventoried",str(json.loads((OUT/"baseline/package_inventory.json").read_text())["count"])] ]))
    (OUT/"tables/package_scope.md").write_text("# Package and profile scope\n\nClassification is based on package manifests, CMake targets, source paths and launch/config. It does not prove each installed node runs in a particular deployment.\n\n"+md_table(["Package","Path","Classification"],[[p["name"],p["manifest"],p["classification"]] for p in json.loads((OUT/"baseline/package_inventory.json").read_text())["packages"]]))
    # Offline evidence index with directly inspectable numbered source excerpts.
    ev_html=['<!doctype html><meta charset="utf-8"><title>Evidence excerpts</title><style>body{font:16px system-ui;max-width:1100px;margin:2rem auto;padding:0 1rem}pre{white-space:pre-wrap;background:#f5f5f5;padding:1rem;border:1px solid #ddd}code{overflow-wrap:anywhere}.ref{scroll-margin-top:1rem;margin:2rem 0}a{color:#0645ad}</style><h1>Evidence excerpts</h1><p>Excerpts are taken from captured source snapshot bytes. Each record includes full declared line range, SHA-256 and relative path.</p>']
    for r in refs:
        ev_html.append(f"<section class='ref' id='{html.escape(r['id'])}'><h2>{html.escape(r['id'])}: {html.escape(r['claim'])}</h2><p><code>{html.escape(r['path'])}:{r['line_start']}-{r['line_end']}</code> · SHA-256 <code>{r['sha256']}</code></p><p>Snapshot: <a href='../baseline/source_snapshot/{html.escape(r['path'])}'>open captured source</a></p><pre>{html.escape(r['excerpt'])}</pre></section>")
    (OUT/"evidence/refs.html").write_text("\n".join(ev_html)+"\n")
    # DOT source; edge URLs target transition/evidence tables for offline inspection.
    def node(n,label,shape="box",style="rounded,filled",fill="#edf3fa"):
        return f'{n} [shape={shape},style="{style}",fillcolor="{fill}",label="{label.replace(chr(34), chr(92)+chr(34))}"];'
    def write_dot(name,body,rankdir="LR"):
        p=OUT/f"diagrams/src/{name}.dot"
        p.write_text('digraph G {\n graph [rankdir='+rankdir+', bgcolor="white", pad="0.2", nodesep="0.35", ranksep="0.65", fontname="DejaVu Sans"];\n node [fontname="DejaVu Sans",fontsize=10,margin="0.12,0.08"];\n edge [fontname="DejaVu Sans",fontsize=9,arrowsize=0.75];\n'+body+'\n}\n')
        if not a.skip_render:
            subprocess.run(["dot","-Tsvg",str(p),"-o",str(OUT/f"diagrams/svg/{name}.svg")],check=True)
    deployment='\n'.join([node("launch","navigation_bringup launch\ncombined launch: runtime + PX4 external mode"),node("fastlio","FAST-LIO process\nseparate launch path"),node("runtime","Process: navigation_runtime\nNavigationRuntimeNode / 2-thread executor"),node("modeproc","Process: px4_navigation_external_mode\nNavigationMode + Executor + state-input thread"),node("store","Runtime authority\nExecutionTimelineStore + ExecutionEpisode"),node("worker","Planner worker / backend\nserial solve"),node("mission","MissionController\nmeasured progress"),node("px4","PX4 interface\nsetpoint update input"),node("world","Mapping/world snapshot\nworker + store")]+[
        'launch -> runtime [label="creates process",style=solid];','launch -> modeproc [label="creates process",style=solid];','fastlio -> runtime [label="ROS state / observations",style=dashed];','fastlio -> modeproc [label="odometry + health",style=dashed];','modeproc -> runtime [label="goal / mode status",labeltooltip="async ROS messages",style=dashed];','runtime -> modeproc [label="NavigationCommand",style=dashed];','runtime -> worker [label="PlanningRequest",style=solid];','worker -> store [label="candidate; guarded commit",style=solid];','world -> store [label="world identity / freshness",style=dotted];','store -> runtime [label="sample / publish gate",style=solid];','modeproc -> px4 [label="P/V/A/yaw setpoint input",style=solid];','px4 -> modeproc [label="VehicleStatus / local position",style=dashed];','mission -> modeproc [label="timer callback / event",style=solid];','legend [shape=note,label="Legend: solid = in-process call/ownership; dashed = ROS/PX4 async message; dotted = state observation/validation"];'])
    write_dot("deployment",deployment,"LR")
    owners='\n'.join([node("runtime","NavigationRuntimeNode\nactive_goal_ (desired)\nexecuting_goal_ (physical identity)"),node("pending","PendingGoalHandoffOwner\none deferred goal slot"),node("episode","ExecutionEpisode\nphase + recovery snapshot + latch"),node("timeline","ExecutionTimelineStore\nactive + pending immutable bundle"),node("sampler","CommandSampler\nread-only time/role sample"),node("worker","PlanningWorker\nactive + latest pending job"),node("mode","NavigationMode\nadmitted NavigationCommand cache"),node("mission","MissionController\nroute progress / waypoint / request"),node("executor","NavigationModeExecutor\nAPI handover + status confirmation")]+[
        'runtime -> pending [label="owns",style=solid];','runtime -> episode [label="owns",style=solid];','runtime -> timeline [label="owns",style=solid];','runtime -> sampler [label="owns",style=solid];','runtime -> worker [label="owns",style=solid];','runtime -> mode [label="NavigationCommand async ROS",style=dashed];','mode -> runtime [label="NavigationGoal / ModeStatus async ROS",style=dashed];','mode -> mission [label="owns / calls under trajectory lock",style=solid];','mode -> executor [label="Hold request callback",style=solid];','timeline -> sampler [label="snapshot read",style=dotted];','episode -> runtime [label="snapshot decision input",style=dotted];','legend [shape=note,label="Legend: solid = ownership/synchronous call; dashed = async handoff/message; dotted = read-only snapshot or observation. Desired, executing, committed, cached and PX4 vehicle state are distinct."];'])
    write_dot("ownership",owners,"LR")
    trans_groups={
      "runtime_goal_commit":{"ids":["T_GOAL_ACCEPT","T_SCHEDULE","T_CANDIDATE","T_ACTIVATE_SUCCESSOR","T_PUBLISH"],"rank":"TB","links":[("T_GOAL_ACCEPT","T_SCHEDULE","request accepted"),("T_SCHEDULE","T_CANDIDATE","solve result"),("T_CANDIDATE","T_ACTIVATE_SUCCESSOR","staged successor only"),("T_CANDIDATE","T_PUBLISH","immediate admission path"),("T_ACTIVATE_SUCCESSOR","T_PUBLISH","after due activation")]},
      "runtime_recovery":{"ids":["T_SAMPLE_BACKUP","T_RECOVERY_STOP","T_EMERGENCY_FAIL","T_EPOCH_RESET"],"rank":"TB","links":[("T_SAMPLE_BACKUP","T_RECOVERY_STOP","sampled suffix may reach measured stop")]},
      "mission_progress":{"ids":["T_ACTIVATE","T_ADMIT_COMMAND","T_PASS_PROGRESS","T_STOP_COMPLETE"],"rank":"TB","links":[("T_ACTIVATE","T_ADMIT_COMMAND","goal then command, async ROS delivery"),("T_ADMIT_COMMAND","T_PASS_PROGRESS","current accepted witness"),("T_PASS_PROGRESS","T_STOP_COMPLETE","next STOP or terminal branch")]},
      "px4_authority":{"ids":["T_FRAME_RESET","T_LEASE_TIMEOUT","T_HOLD_API"],"rank":"TB","links":[("T_LEASE_TIMEOUT","T_HOLD_API","stale command safety path when applicable")]}}
    tmap={x["id"]:x for x in m["transitions"]}
    for view,spec in trans_groups.items():
        body=[]
        for tid in spec["ids"]:
            t=tmap[tid]
            event=textwrap.fill(t["event"],width=62,break_long_words=False).replace("\n","\\n")
            guard=textwrap.fill(t["guard"],width=66,break_long_words=False).replace("\n","\\n")
            writes=textwrap.fill(", ".join(t["write_set"]),width=62,break_long_words=False).replace("\n","\\n")
            effect=textwrap.fill("; ".join(t["effects"]),width=66,break_long_words=False).replace("\n","\\n")
            label="\\n".join([f"{tid}: {event}",f"Guard: {guard}",f"Writes: {writes}",f"Effect: {effect}"])
            body.append(node(tid,label,fill="#fff4d6"))
            body.append(f'{tid} [URL="../../tables/transitions.html#{tid}",tooltip="{tid}: {t["entrypoint"]}"];')
        # Invisible ordering edges constrain layout only; they carry no semantics or happens-before claim.
        for src,dst in zip(spec["ids"],spec["ids"][1:]): body.append(f'{src} -> {dst} [style=invis,weight=10];')
        for src,dst,label in spec["links"]: body.append(f'{src} -> {dst} [style=dashed,color="#555555",label="{label}"];')
        body.append(node("legend","Each box is one guarded transition. Dashed labeled edges show supported paths; invisible edges only arrange the page. Independent callbacks may interleave.",shape="note",fill="#f5f5f5"))
        write_dot(view,"\n".join(body),spec["rank"])
    dec='\n'.join([node("goal","Goal acceptance / deferral\nD_GOAL"),node("commit","Candidate commit / activation / revoke\nD_COMMIT"),node("expose","Sample and ROS command exposure\nD_EXPOSE"),node("admit","PX4 mode message admission\nD_ADMIT"),node("setpoint","Setpoint / Hold request\nD_SETPOINT")]+[
        'goal -> commit [label="desired request -> candidate guards",style=solid];','commit -> expose [label="active timeline only",style=solid];','expose -> admit [label="ROS NavigationCommand",style=dashed];','admit -> setpoint [label="cached accepted message + state gates",style=solid];','setpoint -> commit [label="next measured/request callback; async",style=dashed,constraint=false];','note [shape=note,label="No edge means PX4 acceptance is not inferred. Process boundaries and leases create separate decisions."];','goal -> note [style=invis];'])
    write_dot("decision_authority",dec,"LR")
    # Sequence sources are rendered from model.sequence_diagrams; they are not SVG because mmdc is unavailable.
    for seq in m.get("sequence_diagrams", []):
        lines=["sequenceDiagram"]
        for part in seq["participants"]:
            lines.append(f"    participant {part['id']} as {part['label']}")
        for step in seq["steps"]:
            kind=step["kind"]
            if kind=="message": lines.append(f"    {step['from']}{step.get('arrow','->>')}{step['to']}: {step['text']}")
            elif kind=="note": lines.append(f"    Note over {','.join(step['over'])}: {step['text']}")
            elif kind=="alt": lines.append(f"    alt {step['text']}")
            elif kind=="else": lines.append(f"    else {step['text']}")
            elif kind=="end": lines.append("    end")
            else: raise SystemExit(f"unknown Mermaid step kind {kind} in {seq['id']}")
        (OUT/f"diagrams/src/{seq['file']}").write_text("\n".join(lines)+"\n")
    (OUT/"diagrams/svg/README.md").write_text("Graphviz SVG views are rendered by `tools/build_views.py`. Mermaid CLI (`mmdc`) is not available in the captured tool environment; four `.mmd` sequence sources are retained but intentionally not claimed rendered.\n")
    # Compact machine-readable validation; verify_views.py performs strict checks.
    dot_count = len(list((OUT/"diagrams/src").glob("*.dot")))
    print(f"generated {len(fields)} field rows, {len(m['transitions'])} transition rows, {len(refs)} evidence excerpts, {dot_count} DOT/SVG views, {len(m.get('sequence_diagrams', []))} Mermaid sources")
    if not a.skip_render:
        print("Graphviz rendering completed")

if __name__ == "__main__": main()
