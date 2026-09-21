#!/usr/bin/env python3
"""Regenerate claim-focused tables and source evidence from pinned Git objects."""
from __future__ import annotations
import argparse, csv, hashlib, json, subprocess
from pathlib import Path

ASIS = "f2bd3f46f9936d622377ea4761f733f988273b66"
ASIS_ROOT = "artifacts/architecture_as_is/20260920T-as-is-local"
OUT_REL = Path("artifacts/audit_state_contracts/20260921T013409Z-8eaab3d3")

def git(repo: Path, *args: str, text: bool = True):
    return subprocess.check_output(["git", *args], cwd=repo, text=text, stderr=subprocess.PIPE)

def main() -> int:
    ap=argparse.ArgumentParser()
    ap.add_argument("--repo", type=Path, required=True)
    ap.add_argument("--target-sha", required=True)
    a=ap.parse_args(); repo=a.repo.resolve(); out=repo/OUT_REL
    target=a.target_sha
    model=json.loads((out/"model/architecture.json").read_text())
    if model["target_sha"] != target: raise SystemExit("model target_sha does not match argument")
    manifest_bytes=git(repo,"show",f"{ASIS}:{ASIS_ROOT}/baseline/source_manifest.json",text=False)
    manifest=json.loads(manifest_bytes)
    if hashlib.sha256(manifest_bytes).hexdigest() != "f9481b2f7312556ae91a8bff590cbc412de8d7e3e068fc98d4c85bb34c730413":
        raise SystemExit("frozen A source manifest hash mismatch")
    delta=[]; a_paths=set()
    for f in manifest["files"]:
        p=f["path"]; a_paths.add(p)
        try: b=git(repo,"show",f"{target}:{p}",text=False); h=hashlib.sha256(b).hexdigest(); status="same" if h==f["sha256"] else "changed"
        except subprocess.CalledProcessError: h=None; status="missing_from_target"
        if status!="same": delta.append({"path":p,"status":status,"A_sha256":f["sha256"],"TARGET_sha256":h,"A_source_kind":f.get("source_kind")})
    target_paths=set(git(repo,"ls-tree","-r","--name-only",target).splitlines())
    roots=["src/planning/navigation_planning_backend/","src/planning/navigation_planning/","src/runtime/navigation_runtime/","src/px4/px4_navigation_external_mode/","src/contracts/navigation_mission/","src/contracts/navigation_contracts/","src/execution/navigation_execution/"]
    package=[]
    for prefix in roots:
        old={p for p in a_paths if p.startswith(prefix)}; new={p for p in target_paths if p.startswith(prefix)}
        package.append({"prefix":prefix,"A_files":len(old),"TARGET_files":len(new),"removed":sorted(old-new),"added":sorted(new-old)})
    data={"as_is_artifact_commit":ASIS,"as_is_artifact_root":ASIS_ROOT,"as_is_HEAD_at_capture":manifest["head"],"as_is_source_manifest_sha256":hashlib.sha256(manifest_bytes).hexdigest(),"as_is_source_count":manifest["source_files"],"claim_validation_commit":"f2ed429bef80b2c7a2964b3b32d3c00e8089e55f","target_sha":target,"target_tree":git(repo,"rev-parse",f"{target}^{{tree}}").strip(),"changed_or_missing_count":len(delta),"changed_or_missing":delta,"audited_package_membership":package,"note":"A source snapshot came from a dirty working tree at HEAD 9534d8dc; comparison reads A manifest hashes and TARGET git object bytes. A broad path-prefix additions list is not used to infer vendor changes in the focused package comparison."}
    (out/"baseline/A_to_TARGET_delta.json").write_text(json.dumps(data,indent=2)+"\n")
    refs=[]; spec=json.loads((out/"evidence/refs_spec.json").read_text())
    for r in spec["refs"]:
        p=r["path"]
        b=git(repo,"show",f"{target}:{p}",text=False)
        lines=b.decode("utf-8",errors="replace").splitlines(); lo=min(max(1,r["requested_lines"][0]),len(lines)); hi=min(max(lo,r["requested_lines"][1]),len(lines))
        blob=git(repo,"rev-parse",f"{target}:{p}").strip()
        refs.append({"id":r["id"],"path":p,"symbol":r["symbol"],"line_range":[lo,hi],"snapshot_sha256":hashlib.sha256(b).hexdigest(),"git_blob":blob,"target_sha":target,"excerpt":"\n".join(f"{n}: {lines[n-1]}" for n in range(lo,hi+1)),"status":"RESOLVED"})
    (out/"evidence/refs.jsonl").write_text("".join(json.dumps(x,ensure_ascii=False)+"\n" for x in refs))
    fields=[]
    for obj in model["objects"]:
        for f in obj["fields"]: fields.append((obj,f))
    with (out/"tables/state_inventory.csv").open("w",newline="") as stream:
        w=csv.writer(stream,lineterminator="\n"); w.writerow(["object_id","symbol","instance","field_id","field","type","role","initialization","writer/reset","reader","meaning","clear/invalidation","protection","evidence_ids"])
        for obj,f in fields: w.writerow([obj["id"],obj["symbol"],obj["instance"],f["id"],f["name"],f["type"],f["role"],f["init"],f["writer"],f["reader"],f["meaning"],f["clear"],obj["protection"],";".join(f.get("evidence", obj["evidence"]))])
    # One row per field: distinguish storage owner from domain/command authority.
    authority=["# State authority / writer-reader matrix (generated from model)", "",
      "Scoped to the three requested contracts. A row records the source-backed field meaning and the selected-path writer/read inventory; it does not imply whole-repository exhaustiveness or runtime observation.", "",
      "| Owner / field ID | Type / role | Init; meaning | Writers / reset | Readers / decision | Clear / invalidation | Protection | Evidence |",
      "|---|---|---|---|---|---|---|---|"]
    for obj,f in fields:
        evlinks=[]
        for eid in f.get("evidence",obj.get("evidence",[])):
            evlinks.append(f"[`{eid}`](evidence/index.md#{eid})")
        authority.append(f"| `{obj['symbol']}` / `{f['id']}` `{f['name']}` | `{f['type']}` / {f['role']} | `{f['init']}`; {f['meaning']} | {f['writer']} | {f['reader']} | {f['clear']} | {obj['protection']} | {'; '.join(evlinks)} |")
    authority += ["", "## Competing-authority assessment", "",
      "- Desired and active identity are distinct: planner intent can be newer while its predecessor remains sampled; successor commit is the command identity cutover.",
      "- Suspended generation is a resume witness, not a second bundle store.",
      "- The two 5 s timers have different owners, clocks, start events and consumers; equality of duration alone does not establish conflict. The required cross-layer budget relation remains a specification question.",
      "- Adapter command cache is a transport snapshot, not proof PX4 applied a setpoint.",
      "- MissionController terminal state, adapter mission terminal latch, hold request, Hold API completion and VehicleStatus confirmation are separate observations. API Success/Deactivated clears pending/in-flight, while only VehicleStatus AUTO_LOITER sets confirmed; Failure retains pending and retries. Deactivated is treated as authority loss.",
      "- Not closed: live deployment parameters, target PX4 status ordering/application, independent callback scheduling in target profile, and whole-repository alias/write exhaustiveness.", ""]
    (out/"STATE_AUTHORITY_MATRIX.md").write_text("\n".join(authority))
    # Compact evidence index; full source excerpts remain once in refs.jsonl.
    refs_by_id={r["id"]:r for r in refs}
    ref_lines={r["id"]:i for i,r in enumerate(refs,1)}
    ev=["# Evidence index", "", f"Pinned product source: `{target}`. Commit-pinned source lines and SHA-256 are linked below; complete excerpts are preserved once in `refs.jsonl`.", ""]
    for eid,r in refs_by_id.items():
        lo,hi=r["line_range"]; end=f"L{lo}-L{hi}" if hi!=lo else f"L{lo}"
        url=f"https://github.com/TanDatEmb/uav-navigation/blob/{target}/{r['path']}#{end}"
        line=ref_lines[eid]
        ev += [f"<a id=\"{eid}\"></a>", f"- `{eid}` — {r['symbol']}: [`{r['path']}:{lo}-{hi}`]({url}); [excerpt/hash record](refs.jsonl#L{line}); SHA-256 `{r['snapshot_sha256']}`"]
    (out/"evidence/index.md").write_text("\n".join(ev)+"\n")
    rows=["# Transition inventory (generated from model)","", "| ID | Owner | Event | Guard | Ordered writes / effect | Evidence |","|---|---|---|---|---|---|"]
    for t in model["transitions"]:
        writes='; '.join(t['writes'])
        field_refs=', '.join(f"`{x['object']}.{x['field']}`" for x in t.get('key_field_writes',[]))
        rows.append(f"| `{t['id']}` | `{t['object']}` | {t['event']} | {t['guard']} | {writes}; **key field writes:** {field_refs}; **effect:** {t['effect']} | {', '.join('`'+x+'`' for x in t['evidence'])} |")
    (out/"tables/transitions.md").write_text("\n".join(rows)+"\n")
    rows=["# Contract verdict table (generated from model)","", "| ID | Verdict | Statement | Reachability | Counterevidence | Open |","|---|---|---|---|---|---|"]
    for c in model["contracts"]: rows.append(f"| `{c['id']}` | `{c['verdict']}` | {c['statement']} | {c['reachability']} | {c['counterevidence']} | {c['open']} |")
    (out/"tables/contracts.md").write_text("\n".join(rows)+"\n")
    cov=model["coverage"]
    (out/"tables/coverage.md").write_text("# Coverage (generated from model)\n\n"+json.dumps(cov,indent=2,ensure_ascii=False)+"\n\n`PARTIAL_AS_IS`: this scoped model does not claim whole-repository exhaustive writers, target workload observation, or PX4 applied-state evidence.\n")
    print(json.dumps({"target_sha":target,"changed_or_missing":len(delta),"refs":len(refs),"fields":len(fields),"transitions":len(model["transitions"])},indent=2))
    return 0
if __name__ == "__main__": raise SystemExit(main())
