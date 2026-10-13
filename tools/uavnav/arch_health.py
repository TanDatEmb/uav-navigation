#!/usr/bin/env python3
"""Architecture stress indicators from Git history (docs/CHANGE_CONTROL.md §5, D34).

Prints a Markdown report. A signal over its threshold means "hold an Architecture Review of this
subsystem", never "redesign" and never "stop unrelated work". The numbers are objective; deciding what
they mean is the reviewer's job (commit counts alone prove nothing about erosion; read the diffs).

Window per subsystem: commits after `--base` (default: merge-base with main) and after the latest
reviewed window end (`Window end` column of the AR table in DECISIONS.md), so a review that decided
KEEP silences the same history until new evidence accumulates.

Indicators (arch_rules.yaml `stress`):
  S1 state flags    new bool members / new state-enum enumerators in non-test headers
  S2 workarounds    added lines with workaround markers
  S3 repeated fixes the same non-test file touched by many fix/refactor commits
  S4 contract churn the same contract file changed repeatedly; one contract change fanning out to many packages
  S5 deviations     open deviations past their review date; number of open deviations
  S6 coupling       package pairs that keep changing together
  S7 doc drift      arch_check findings on TRACEABILITY
  S8 regressions    TRACEABILITY rows whose status went down
Not mechanically detectable (reviewer checklist in the PR template): logic duplication between
subsystems, special-case paths without a unified model, responsibility leakage beyond the dependency rules.
"""
from __future__ import annotations

import argparse
import collections
import datetime
import pathlib
import re
import sys

from tools.uavnav import arch_check, archlib, check_change

BOOL_MEMBER = re.compile(r"^\+\s*bool\s+(\w+)\s*(?:=\s*[^;(]+|\{[^}]*\})?\s*;")
FIX_SUBJECT = re.compile(r"^(fix|refactor)(\(|!|:)")
LEGACY_RANK = {"todo": 0, "doing": 1, "done": 2, "deferred": 0}


def architecture_reviews(root: pathlib.Path) -> list[dict]:
    text = (root / "docs/architecture/DECISIONS.md").read_text(encoding="utf-8")
    out = []
    for table in archlib.parse_tables(text):
        if table.header and table.header[0] == "ID" and table.col("Window end") is not None:
            sub, win, res = table.col("Hệ con"), table.col("Window end"), table.col("Kết quả")
            for _, cells in table.rows:
                if re.fullmatch(r"AR\d+", cells[0]):
                    spans = archlib.code_spans(cells[win])
                    out.append({"id": cells[0], "subsystem": cells[sub].strip(), "window_end": spans[0] if spans else "",
                                "result": cells[res].split(" ")[0].strip("*`")})
    return out


def default_base(root: pathlib.Path) -> str:
    result = archlib.git(root, "merge-base", "main", "HEAD", check=False).strip()
    if result:
        return result
    return archlib.git(root, "rev-list", "--max-parents=0", "HEAD").split()[0]


def commits_in(root: pathlib.Path, base: str) -> list[str]:
    return archlib.git(root, "rev-list", "--reverse", "--no-merges", f"{base}..HEAD").split()


def analyse(root: pathlib.Path, rules: dict, base: str, today: datetime.date | None = None) -> dict:
    today = today or datetime.date.today()
    thresholds = rules["stress"]
    workaround = re.compile(rules["workaround_regex"])
    reviews = architecture_reviews(root)
    all_commits = commits_in(root, base)

    index = archlib.EvidenceIndex(root)
    window: dict[str, set[str]] = {}
    for sub in rules["subsystems"]:
        ends = [r["window_end"] for r in reviews
                if r["subsystem"] == sub and r["window_end"] and index.commit_exists(r["window_end"])]
        # the latest reviewed window end is the one with the fewest commits after it
        window[sub] = set(commits_in(root, min(ends, key=lambda e: len(commits_in(root, e))))) if ends else set(all_commits)

    in_tree = set(archlib.git(root, "ls-files").splitlines())
    sub_stats = {s: {"commits": 0, "fix": 0, "bools": set(), "states": [], "workarounds": 0,
                     "fix_files": collections.Counter(), "contract_files": collections.Counter(),
                     "fanout": []} for s in rules["subsystems"]}
    pair_counts: collections.Counter = collections.Counter()
    pair_commits: dict[tuple, set] = collections.defaultdict(set)

    for commit in all_commits:
        subject = archlib.git(root, "log", "-1", "--format=%s", commit).strip()
        status = {}
        for entry in archlib.git(root, "diff-tree", "--no-commit-id", "--name-status", "-r", "-M", "--root", commit).splitlines():
            parts = entry.split("\t")
            status[parts[-1]] = parts[0][0]
        files = list(status)
        patch = archlib.git(root, "show", "-U0", "--format=", commit)
        by_file: dict[str, list[str]] = collections.defaultdict(list)
        current = None
        for line in patch.splitlines():
            if line.startswith("+++ b/"):
                current = line[6:]
            elif current and line.startswith("+") and not line.startswith("+++"):
                by_file[current].append(line)
        code_files = [f for f in files if not archlib.is_doc_path(f) and not archlib.is_test_path(f)]
        packages = {"/".join(f.split("/")[:3]) for f in code_files if f.startswith("src/") and f.count("/") >= 2}
        for a in packages:
            for b in packages:
                if a < b:
                    pair_commits[(a, b)].add(commit)

        for sub in rules["subsystems"]:
            if commit not in window[sub]:
                continue
            mine = [f for f in files if archlib.subsystem_of(f, rules) == sub]
            if not mine:
                continue
            st = sub_stats[sub]
            st["commits"] += 1
            is_fix = bool(FIX_SUBJECT.match(subject))
            st["fix"] += is_fix
            for f in mine:
                if archlib.is_test_path(f) or archlib.is_doc_path(f) or f not in in_tree:
                    continue
                # a file's creation or rename is not evidence of stress; only later modification is
                modified = status[f] == "M"
                if is_fix and modified:
                    st["fix_files"][f] += 1
                if f.endswith(".hpp") and modified:
                    for line in by_file.get(f, []):
                        m = BOOL_MEMBER.match(line)
                        if m:
                            st["bools"].add(f"{f}:{m.group(1)}")
                if modified and archlib.matches(f, rules["churn_paths"]):
                    st["contract_files"][f] += 1
                st["workarounds"] += sum(1 for line in by_file.get(f, []) if workaround.search(line))
            st["states"] += check_change.added_states(
                root, commit, [f for f in mine if status[f] == "M" and f in in_tree], rules["state_enum_regex"])
            contract_here = [f for f in mine if status[f] == "M" and archlib.matches(f, rules["churn_paths"])]
            if contract_here and len(packages) - 1 >= thresholds["producer_fanout_packages"]:
                st["fanout"].append(f"{commit[:8]} ({len(packages)} packages)")

    triggers: list[str] = []
    rows = []
    for sub, st in sub_stats.items():
        if not st["commits"]:
            continue
        repeated = {f: n for f, n in st["fix_files"].items() if n >= thresholds["fix_commits_same_file"]}
        churn = {f: n for f, n in st["contract_files"].items() if n >= thresholds["contract_file_changes"]}
        checks = [
            ("S1 new bool members", len(st["bools"]), thresholds["new_bool_members"], ", ".join(sorted(st["bools"]))),
            ("S1 new state enumerators", len(st["states"]), thresholds["new_state_enumerators"], "; ".join(st["states"])),
            ("S2 workaround markers", st["workarounds"], thresholds["workaround_markers"], ""),
            ("S3 files with repeated fixes", len(repeated), 1,
             ", ".join(f"{f} x{n}" for f, n in sorted(repeated.items()))),
            ("S4 contract files churned", len(churn), 1, ", ".join(f"{f} x{n}" for f, n in sorted(churn.items()))),
            ("S4 contract fan-out commits", len(st["fanout"]), 1, ", ".join(st["fanout"])),
        ]
        for name, value, limit, detail in checks:
            hit = value >= limit and value > 0
            rows.append((sub, name, value, limit, hit, detail))
            if hit:
                triggers.append(f"{sub}: {name} = {value} (>= {limit})")

    coupling = [(a, b, len(c)) for (a, b), c in pair_commits.items() if len(c) >= thresholds["co_change_commits"]]
    for a, b, n in sorted(coupling, key=lambda x: -x[2]):
        triggers.append(f"cross: S6 {a} and {b} changed together in {n} commits (>= {thresholds['co_change_commits']})")

    register = arch_check.deviation_register(root)
    open_dv = [k for k, v in register.items() if arch_check._cell(v, "Trạng thái").split(" ")[0].strip("*`") == "OPEN"]
    overdue = [f for f in arch_check.check_deviations(root, today, strict_dates=False) if "has passed" in f.message]
    if overdue:
        triggers += [f"deviations: S5 {f.message}" for f in overdue]
    if len(open_dv) >= 3:
        triggers.append(f"deviations: S5 {len(open_dv)} open deviations (>= 3): group them by root cause in one review")

    drift = [f for f in arch_check.check_traceability(root) if f.level == "ERROR"]
    if drift:
        triggers.append(f"docs: S7 {len(drift)} TRACEABILITY finding(s)")

    regressions = status_regressions(root, base, window["docs"])
    if len(regressions) >= thresholds["status_regressions"]:
        triggers.append(f"docs: S8 {len(regressions)} TRACEABILITY status regression(s)")

    return {"base": base, "commits": len(all_commits), "rows": rows, "coupling": coupling, "open_dv": open_dv,
            "overdue": overdue, "drift": drift, "regressions": regressions, "triggers": triggers,
            "reviews": reviews, "stats": sub_stats}


def _rank(word: str) -> int | None:
    word = word.strip("*` ").split(" ")[0]
    if word in archlib.LADDER:
        return archlib.LADDER.index(word)
    return LEGACY_RANK.get(word)


def status_regressions(root: pathlib.Path, base: str, only: set[str]) -> list[str]:
    revs = archlib.git(root, "log", "--reverse", "--format=%H", f"{base}..HEAD", "--", "docs/TRACEABILITY.md").split()
    previous: dict[str, int] = {}
    out = []
    for rev in revs:
        text = archlib.git(root, "show", f"{rev}:docs/TRACEABILITY.md", check=False)
        current = {}
        for table in archlib.parse_tables(text):
            sc = table.col("Trạng thái")
            if sc is None or table.header[0] in ("ID", "Trạng thái"):
                continue
            for _, cells in table.rows:
                rank = _rank(cells[sc]) if len(cells) > sc else None
                if rank is not None:
                    current[cells[0][:60]] = rank
        for key, rank in current.items():
            if key in previous and rank < previous[key] and rev in only:
                out.append(f"{rev[:8]}: {key}")
        previous = current
    return out


def render(result: dict) -> str:
    lines = [f"# Architecture health report", "",
             f"Window: {result['commits']} commits after `{result['base'][:8]}`. "
             f"Reviews on record: {', '.join(r['id'] + ' (' + r['subsystem'] + ', ' + r['result'] + ')' for r in result['reviews']) or 'none'}.",
             "", "| Subsystem | Indicator | Value | Review at | Signal |", "|---|---|---|---|---|"]
    for sub, name, value, limit, hit, detail in result["rows"]:
        lines.append(f"| {sub} | {name} | {value} | {limit} | {'**REVIEW**' if hit else 'ok'} |")
    lines += ["", "## Details", ""]
    for sub, name, value, limit, hit, detail in result["rows"]:
        if hit and detail:
            lines.append(f"- {sub} / {name}: {detail}")
    for a, b, n in result["coupling"]:
        lines.append(f"- cross / S6: {a} and {b}: {n} commits together")
    lines.append(f"- open deviations: {', '.join(result['open_dv']) or 'none'}")
    for r in result["regressions"]:
        lines.append(f"- status regression: {r}")
    lines += ["", "## Review triggers", ""]
    lines += [f"- {t}" for t in result["triggers"]] or ["- none"]
    lines += ["", "A trigger means: hold an Architecture Review of that subsystem (CHANGE_CONTROL.md §5), "
              "read the diffs, and record the result as an AR row in DECISIONS.md. It does not mean redesign."]
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=str(archlib.ROOT))
    parser.add_argument("--base", help="exclusive start (default: merge-base with main)")
    parser.add_argument("--fail-on-trigger", action="store_true")
    args = parser.parse_args(argv)
    root = pathlib.Path(args.root).resolve()
    result = analyse(root, archlib.load_rules(), args.base or default_base(root))
    print(render(result))
    return 1 if args.fail_on_trigger and result["triggers"] else 0


if __name__ == "__main__":
    sys.exit(main())
