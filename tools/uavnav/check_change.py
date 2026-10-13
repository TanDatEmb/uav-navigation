#!/usr/bin/env python3
"""Check the architecture-impact declarations of commits (docs/CHANGE_CONTROL.md §3, D34).

For each commit after `baseline_commit` (arch_rules.yaml) up to HEAD:
  * a commit that touches a contract path or an invariant path must carry `Arch-Level:` (0-3) and
    `Docs-Impact:` (none | repo paths | pending:<reason>) trailers; other commits need none (= Level 0);
  * Level >= 1 cannot say `Docs-Impact: none`, and the paths it lists must exist;
  * adding an enumerator to a state enum (state_enum_regex) is a new state: Level >= 2 (SYSTEM_DESIGN §2.2);
  * Level 2 needs `Review: AR<n>` with result LIMITED_REFACTOR or APPROVED in DECISIONS.md;
    Level 3 needs `Review: AR<n>` APPROVED and `Decision: D<n>` AGREED.
It reports which invariants a commit touched so the reviewer looks at them. It cannot judge whether a
declared level is right; that stays a human review. Exit 0 = no ERROR.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
from dataclasses import dataclass

from tools.uavnav import archlib

TRAILER = re.compile(r"^(Arch-Level|Docs-Impact|Review|Decision):[ \t]*(.*?)[ \t]*$", re.M)
ENUM = re.compile(r"\benum\s+class\s+(\w+)\s*(?::\s*[\w:]+\s*)?\{([^}]*)\}", re.S)


@dataclass(frozen=True)
class Finding:
    level: str  # ERROR | WARN | INFO
    where: str
    message: str

    def __str__(self) -> str:
        return f"{self.level} {self.where}: {self.message}"


def state_enumerators(source: str, name_regex: str) -> dict[str, set[str]]:
    out: dict[str, set[str]] = {}
    for name, body in ENUM.findall(source):
        if re.fullmatch(name_regex, name):
            body = re.sub(r"//.*|/\*.*?\*/", "", body, flags=re.S)
            out[name] = {part.split("=")[0].strip() for part in body.split(",") if part.split("=")[0].strip()}
    return out


def added_states(root: pathlib.Path, commit: str, files: list[str], name_regex: str) -> list[str]:
    added = []
    for rel in files:
        if not rel.endswith((".hpp", ".cpp", ".h")) or archlib.is_test_path(rel):
            continue
        new = state_enumerators(archlib.git(root, "show", f"{commit}:{rel}", check=False), name_regex)
        old = state_enumerators(archlib.git(root, "show", f"{commit}^:{rel}", check=False), name_regex)
        for enum, items in new.items():
            for item in sorted(items - old.get(enum, set())):
                added.append(f"{rel}: {enum}::{item}")
    return added


def parse_trailers(message: str) -> dict[str, str]:
    return {key: value for key, value in TRAILER.findall(message)}


def check_commit(root, rules, commit, decisions, range_files) -> list[Finding]:
    findings: list[Finding] = []
    short = commit[:8]
    subject = archlib.git(root, "log", "-1", "--format=%s", commit).strip()
    where = f"{short} {subject[:60]}"
    message = archlib.git(root, "log", "-1", "--format=%B", commit)
    files = archlib.git(root, "diff-tree", "--no-commit-id", "--name-only", "-r", "--root", commit).splitlines()
    trailers = parse_trailers(message)

    contract = [f for f in files if archlib.matches(f, rules["contract_paths"])]
    invariants = {inv: [f for f in files if f in paths] for inv, paths in rules["invariant_paths"].items()}
    invariants = {k: v for k, v in invariants.items() if v}
    states = added_states(root, commit, files, rules["state_enum_regex"])

    if invariants:
        findings.append(Finding("INFO", where, "touches invariant(s): " + ", ".join(sorted(invariants))))
    needs_declaration = bool(contract or invariants or states)
    level_text = trailers.get("Arch-Level")
    if needs_declaration and (level_text is None or "Docs-Impact" not in trailers):
        what = "contract path" if contract else "invariant path" if invariants else "state enumerator"
        findings.append(Finding("ERROR", where, f"touches a {what}: add `Arch-Level:` and `Docs-Impact:` trailers"))
        return findings
    if level_text is None:
        return findings
    if not re.fullmatch(r"[0-3]", level_text):
        return findings + [Finding("ERROR", where, f"Arch-Level {level_text!r} is not 0, 1, 2 or 3")]
    level = int(level_text)

    impact = trailers.get("Docs-Impact", "")
    if level >= 1 and impact.lower() in ("", "none"):
        findings.append(Finding("ERROR", where, f"Arch-Level {level} needs Docs-Impact naming the documents it updates"))
    if impact.startswith("pending:"):
        findings.append(Finding("WARN", where, f"documentation debt: {impact}"))
    elif impact.lower() not in ("", "none"):
        for path in [p.strip() for p in impact.split(",") if p.strip()]:
            if not (root / path).exists():
                findings.append(Finding("ERROR", where, f"Docs-Impact names {path}, which does not exist"))
            elif path not in range_files and path not in files:
                findings.append(Finding("WARN", where, f"Docs-Impact names {path}, which no commit in the range changes"))

    if states and level < 2:
        findings.append(Finding("ERROR", where, "adds a state enumerator (" + "; ".join(states) +
                                ") but declares Arch-Level < 2: a new state needs an Architecture Review (SYSTEM_DESIGN §2.2)"))
    if level >= 2:
        review = trailers.get("Review", "")
        result = decisions.get(review, "")
        if not review or not review.startswith("AR") or not result:
            findings.append(Finding("ERROR", where, f"Arch-Level {level} needs `Review: AR<n>` that exists in DECISIONS.md"))
        else:
            allowed = ("LIMITED_REFACTOR", "APPROVED") if level == 2 else ("APPROVED",)
            if result not in allowed:
                findings.append(Finding("ERROR", where,
                                        f"{review} result is {result}; Level {level} needs {' or '.join(allowed)}"))
        if level == 3 and decisions.get(trailers.get("Decision", ""), "") != "AGREED":
            findings.append(Finding("ERROR", where, "Arch-Level 3 needs `Decision: D<n>` with status AGREED"))
    return findings


def check_range(root: pathlib.Path, rules: dict, base: str, head: str = "HEAD") -> list[Finding]:
    decisions = archlib.decision_status(root)
    commits = archlib.git(root, "rev-list", "--reverse", "--no-merges", f"{base}..{head}").split()
    range_files: set[str] = set()
    for commit in commits:
        range_files.update(archlib.git(root, "diff-tree", "--no-commit-id", "--name-only", "-r", "--root", commit).splitlines())
    findings: list[Finding] = []
    for commit in commits:
        findings += check_commit(root, rules, commit, decisions, range_files)
    return findings


def check_body(rules: dict, body: str, commit_levels: list[int]) -> list[Finding]:
    """A PR description must repeat the declaration; it may not be lower than any commit's level."""
    trailers = {k: v for k, v in re.findall(r"^(Arch-Level|Docs-Impact):[ \t]*(.*?)[ \t]*$", body, re.M)}
    findings = []
    if "Arch-Level" not in trailers or "Docs-Impact" not in trailers:
        return [Finding("ERROR", "PR body", "missing `Arch-Level:` or `Docs-Impact:` line (see the PR template)")]
    if not re.fullmatch(r"[0-3]", trailers["Arch-Level"]):
        return [Finding("ERROR", "PR body", f"Arch-Level {trailers['Arch-Level']!r} is not 0, 1, 2 or 3")]
    if commit_levels and int(trailers["Arch-Level"]) < max(commit_levels):
        findings.append(Finding("ERROR", "PR body", f"Arch-Level {trailers['Arch-Level']} is below a commit's level {max(commit_levels)}"))
    return findings


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=str(archlib.ROOT))
    parser.add_argument("--base", help="exclusive start (default: baseline_commit in arch_rules.yaml)")
    parser.add_argument("--head", default="HEAD")
    parser.add_argument("--body-file", help="PR description to check as well")
    args = parser.parse_args(argv)
    root = pathlib.Path(args.root).resolve()
    rules = archlib.load_rules()
    findings = check_range(root, rules, args.base or rules["baseline_commit"], args.head)
    if args.body_file:
        levels = []
        for commit in archlib.git(root, "rev-list", f"{args.base or rules['baseline_commit']}..{args.head}").split():
            level = parse_trailers(archlib.git(root, "log", "-1", "--format=%B", commit)).get("Arch-Level", "")
            if level.isdigit():
                levels.append(int(level))
        findings += check_body(rules, pathlib.Path(args.body_file).read_text(encoding="utf-8"), levels)
    for finding in findings:
        print(finding)
    errors = sum(1 for f in findings if f.level == "ERROR")
    print(f"check_change: {errors} error(s), {sum(1 for f in findings if f.level == 'WARN')} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
