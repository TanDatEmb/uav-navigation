#!/usr/bin/env python3
"""Fail-closed structural and baseline checker for this claim audit."""
import hashlib
import json
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from html.parser import HTMLParser
from pathlib import Path
from urllib.parse import unquote, urlsplit

OUT = Path(__file__).resolve().parents[1]
REPO = OUT.parents[2]
ARTIFACT_A = Path(os.environ.get(
    "AUDIT_ARTIFACT_A",
    str(REPO.parent / "uav-navigation-as-is-audit-20260920/artifacts/architecture_as_is/20260920T-as-is-local"),
))
SNAPSHOT = ARTIFACT_A / "baseline/source_snapshot"
DEPENDENCY_REV = "4a3370f084ac6f1ef001a4afa2b007845ffd0837"
errors = []

def fail(message):
    errors.append(message)

def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def git(*args):
    return subprocess.check_output(["git", "-C", str(REPO), *args], text=False)

refs = {}
refs_path = OUT / "evidence/refs.jsonl"
for line_no, line in enumerate(refs_path.read_text().splitlines(), 1):
    try:
        record = json.loads(line)
    except Exception as exc:
        fail(f"refs.jsonl:{line_no}: invalid JSON: {exc}")
        continue
    evidence_id = record.get("id")
    if not evidence_id or evidence_id in refs:
        fail(f"refs.jsonl:{line_no}: duplicate or empty evidence ID {evidence_id!r}")
        continue
    refs[evidence_id] = record
    provenance = record.get("provenance", {})
    origin = provenance.get("source")
    if origin == "A/baseline/source_snapshot":
        source = SNAPSHOT / record["path"]
    elif origin == "pinned_dependency_worktree":
        source = REPO / record["path"]
        dep_path = REPO / "src/external/px4_ros2_interface_lib"
        revision = subprocess.check_output(["git", "-C", str(dep_path), "rev-parse", "HEAD"], text=True).strip()
        if revision != DEPENDENCY_REV:
            fail(f"{evidence_id}: pinned dependency revision drifted: {revision}")
    elif origin == "audit_generated_test":
        source = OUT / record["path"]
    else:
        fail(f"{evidence_id}: unsupported provenance {origin!r}")
        continue
    if not source.is_file():
        fail(f"{evidence_id}: source path missing: {source}")
        continue
    if sha256(source) != record.get("sha256"):
        fail(f"{evidence_id}: SHA-256 mismatch: {record['path']}")
        continue
    source_lines = source.read_text(encoding="utf-8").splitlines()
    start, end = record.get("line_start"), record.get("line_end")
    if not isinstance(start, int) or not isinstance(end, int) or start < 1 or end < start or end > len(source_lines):
        fail(f"{evidence_id}: invalid line range {start}-{end}")
        continue
    excerpt = "\n".join(f"{i}: {source_lines[i-1]}" for i in range(start, end + 1))
    if excerpt != record.get("excerpt"):
        fail(f"{evidence_id}: excerpt does not match hashed source lines")

claims = json.loads((OUT / "claims.json").read_text())
claim_ids = set()
for claim in claims.get("claims", []):
    cid = claim.get("id")
    if not cid or cid in claim_ids:
        fail(f"claims.json: duplicate or empty claim ID {cid!r}")
    claim_ids.add(cid)
    for evidence_id in claim.get("evidence", []):
        if evidence_id not in refs:
            fail(f"{cid}: unresolved evidence ID {evidence_id}")
    for test in claim.get("execution", []):
        if test.get("status") == "TEST_EXECUTED" and "exit_code" not in test:
            fail(f"{cid}: TEST_EXECUTED entry has no exit_code")
        if test.get("status") == "TEST_EXECUTED" and int(test.get("exit_code", -1)) != 0:
            fail(f"{cid}: reported TEST_EXECUTED entry is not successful")

manifest_path = ARTIFACT_A / "baseline/source_manifest.json"
manifest_hash = sha256(manifest_path)
expected_manifest_hash = json.loads((OUT / "baseline/repo_state.json").read_text())["source_manifest_sha256"]
if manifest_hash != expected_manifest_hash:
    fail(f"source manifest hash mismatch: {manifest_hash}")
manifest = json.loads(manifest_path.read_text())
manifest_entries = manifest.get("files", [])
if len(manifest_entries) != 596:
    fail(f"manifest file count changed: {len(manifest_entries)}")
missing_a = []
changed_a = []
current_drift = []
for entry in manifest_entries:
    rel = entry["path"]
    a_path = SNAPSHOT / rel
    b_path = REPO / rel
    if not a_path.is_file():
        missing_a.append(rel)
        continue
    a_hash = sha256(a_path)
    if a_hash != entry["sha256"]:
        changed_a.append(rel)
    if not b_path.is_file():
        current_drift.append({"path": rel, "current_sha256": None, "A_sha256": a_hash})
    else:
        b_hash = sha256(b_path)
        if b_hash != a_hash:
            current_drift.append({"path": rel, "current_sha256": b_hash, "A_sha256": a_hash})
if missing_a:
    fail(f"A manifest snapshot missing paths: {missing_a[:8]}")
if changed_a:
    fail(f"A source snapshot differs from its manifest: {changed_a[:8]}")

initial_status = (OUT / "baseline/status_porcelain_v2.txt").read_text().splitlines()
current_status = git("status", "--porcelain=v2", "--untracked-files=all").decode().splitlines()
own_prefix = f"? artifacts/audit_claim_validation/{OUT.name}/"
outside_audit_status = [line for line in current_status if not line.startswith(own_prefix)]
if outside_audit_status:
    fail(f"current worktree has changes outside this audit output: {outside_audit_status[:8]}")
initial_state = json.loads((OUT / "baseline/repo_state.json").read_text())
current_head = git("rev-parse", "HEAD").decode().strip()
current_branch = git("branch", "--show-current").decode().strip()
current_staged_hash = hashlib.sha256(git("diff", "--cached", "--no-ext-diff")).hexdigest()
current_unstaged_hash = hashlib.sha256(git("diff", "--no-ext-diff")).hexdigest()
captured_head = initial_state["head"]
head_changed = current_head != captured_head
status_changed = sorted(initial_status) != sorted(outside_audit_status)
staged_changed = current_staged_hash != initial_state["staged_diff_sha256"]
unstaged_changed = current_unstaged_hash != initial_state["unstaged_diff_sha256"]
commit_paths = git("diff", "--name-only", f"{captured_head}..{current_head}").decode().splitlines() if head_changed else []
drift = {
    "status": "DRIFT_DETECTED" if (head_changed or status_changed or staged_changed or unstaged_changed or current_drift) else "NO_DRIFT",
    "captured_checkout": {"head": captured_head, "branch": initial_state["branch"],
                          "staged_diff_sha256": initial_state["staged_diff_sha256"],
                          "unstaged_diff_sha256": initial_state["unstaged_diff_sha256"]},
    "current_checkout": {"head": current_head, "branch": current_branch,
                         "staged_diff_sha256": current_staged_hash,
                         "unstaged_diff_sha256": current_unstaged_hash,
                         "status_outside_audit": outside_audit_status},
    "captured_status_changed": status_changed,
    "head_changed": head_changed,
    "committed_paths_since_capture": commit_paths,
    "A_manifest_paths_different_in_current_checkout": current_drift,
    "interpretation": "A findings remain bound to its frozen source snapshot; current checkout requires a separate verdict for every changed claim path.",
}
(OUT / "validation/final_checkout_drift.json").write_text(json.dumps(drift, indent=2) + "\n")

dot_files = sorted((OUT / "diagrams/src").glob("*.dot"))
svg_dir = OUT / "diagrams/svg"
if not dot_files:
    fail("no DOT source diagrams")
for dot_file in dot_files:
    svg = svg_dir / (dot_file.stem + ".svg")
    if not svg.is_file() or svg.stat().st_size < 100:
        fail(f"missing/empty SVG for {dot_file.name}")
        continue
    try:
        root = ET.parse(svg).getroot()
        if root.tag.split("}")[-1] != "svg" or not root.attrib.get("viewBox"):
            fail(f"invalid SVG root/viewBox: {svg.name}")
    except ET.ParseError as exc:
        fail(f"invalid SVG XML {svg.name}: {exc}")

class LocalLinks(HTMLParser):
    def __init__(self):
        super().__init__()
        self.targets = []
    def handle_starttag(self, tag, attrs):
        values = dict(attrs)
        for key in ("href", "src"):
            if values.get(key):
                self.targets.append(values[key])

link_parser = LocalLinks()
link_parser.feed((OUT / "index.html").read_text(encoding="utf-8"))
for target in link_parser.targets:
    parsed = urlsplit(target)
    if parsed.scheme or not parsed.path:
        continue
    resolved = (OUT / unquote(parsed.path)).resolve()
    if not resolved.exists():
        fail(f"offline index link target missing: {target}")

if errors:
    for error in errors:
        print("FAIL:", error, file=sys.stderr)
    raise SystemExit(1)

report = {
    "status": drift["status"],
    "evidence_records": len(refs),
    "claims": len(claim_ids),
    "manifest_source_files": len(manifest_entries),
    "A_source_snapshot_matches_manifest": True,
    "A_paths_different_in_current_checkout": len(current_drift),
    "captured_checkout_status_preserved": not (head_changed or status_changed or staged_changed or unstaged_changed),
    "final_checkout_drift_report": "validation/final_checkout_drift.json",
    "rendered_dot_svg_pairs": len(dot_files),
    "offline_index_links_checked": len(link_parser.targets),
}
(OUT / "validation/artifact_check.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2))
