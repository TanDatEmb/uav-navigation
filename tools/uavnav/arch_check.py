#!/usr/bin/env python3
"""Static architecture-governance checks (docs/CHANGE_CONTROL.md, D34).

Checks only what is objective:
  * TRACEABILITY rows: status vocabulary, and the evidence each status needs (a test claimed as
    evidence must exist; a status above DESIGNED needs matching evidence; QUALIFIED needs an AGREED decision);
  * the deviation register: owner, review trigger, closing plan, overdue review dates;
  * references: D*/O*/AR* ids and spec section numbers named in TRACEABILITY exist;
  * dependency direction: ROS-free packages stay ROS-free, package.xml edges stay inside allowed_deps,
    product code writes no raw stdout/stderr.

Design judgement (is the model still right?) stays with the Architecture Reviewer.
Exit 0 = no ERROR. WARN never fails. `--strict-dates` turns overdue deviation reviews into ERRORs (CI).
"""
from __future__ import annotations

import argparse
import datetime
import pathlib
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass

from tools.uavnav import archlib

DEP_TAGS = ("depend", "build_depend", "exec_depend", "test_depend", "buildtool_depend", "build_export_depend")
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([\w/.+-]+)[>"]')
REF = re.compile(r"\b(D|O|AR)(\d+)\b")
SECTION = re.compile(r"§(\d+(?:\.\d+)?)")
RUN_ID = re.compile(r"^run:\d{8}T\d{6}Z-[\w.+-]+$")
ISO_DATE = re.compile(r"\b(\d{4}-\d{2}-\d{2})\b")


@dataclass(frozen=True)
class Finding:
    level: str  # ERROR | WARN
    where: str
    message: str

    def __str__(self) -> str:
        return f"{self.level} {self.where}: {self.message}"


# ---------------------------------------------------------------- traceability and deviations

def deviation_register(root: pathlib.Path) -> dict[str, dict]:
    """DV id -> {'status', 'row', 'cells', 'header'} from the register table in TRACEABILITY.md."""
    text = (root / "docs/TRACEABILITY.md").read_text(encoding="utf-8")
    out: dict[str, dict] = {}
    for table in archlib.parse_tables(text):
        if table.header and table.header[0] == "ID" and table.col("Hệ con") is not None:
            for number, cells in table.rows:
                if cells and re.fullmatch(r"DV\d+", cells[0]):
                    out[cells[0]] = {"row": number, "cells": cells, "header": table.header,
                                     "table": table}
    return out


def _cell(info: dict, *needles: str) -> str:
    idx = info["table"].col(*needles)
    return info["cells"][idx].strip() if idx is not None and idx < len(info["cells"]) else ""


def check_deviations(root, today: datetime.date, strict_dates: bool) -> list[Finding]:
    findings: list[Finding] = []
    decisions = archlib.decision_status(root)
    evidence = archlib.EvidenceIndex(root)
    for dv, info in deviation_register(root).items():
        where = f"docs/TRACEABILITY.md:{info['row']}"
        status = _cell(info, "Trạng thái").split(" ")[0].strip("*`")
        kind = _cell(info, "Loại").split(" ")[0].strip("*`")
        if status not in ("OPEN", "CLOSED", "ACCEPTED"):
            findings.append(Finding("ERROR", where, f"{dv}: status {status!r} is not OPEN, CLOSED or ACCEPTED"))
            continue
        if kind not in ("TEMPORARY", "PERMANENT"):
            findings.append(Finding("ERROR", where, f"{dv}: type {kind!r} is not TEMPORARY or PERMANENT"))
        for column in ("Hệ con", "Thiết kế gốc", "Thực tế", "Lý do", "Rủi ro", "Owner"):
            if not _cell(info, column):
                findings.append(Finding("ERROR", where, f"{dv}: column '{column}' is empty"))
        trigger = _cell(info, "Xem lại")
        if status == "OPEN":
            if not trigger:
                findings.append(Finding("ERROR", where, f"{dv}: an open deviation needs a review date or trigger"))
            if not _cell(info, "Kế hoạch đóng"):
                findings.append(Finding("ERROR", where, f"{dv}: an open deviation needs a closing plan"))
            for date in ISO_DATE.findall(trigger):
                if datetime.date.fromisoformat(date) < today:
                    level = "ERROR" if strict_dates else "WARN"
                    findings.append(Finding(level, where, f"{dv}: review date {date} has passed; re-evaluate it"))
        else:
            ev = _cell(info, "Bằng chứng")
            spans = archlib.code_spans(ev)
            if not spans:
                findings.append(Finding("ERROR", where, f"{dv}: a {status} deviation needs evidence in backticks"))
            for span in spans:
                if archlib.SHA.match(span) and not evidence.commit_exists(span):
                    findings.append(Finding("ERROR", where, f"{dv}: commit {span} not found"))
        if status == "ACCEPTED":
            refs = [m.group(0) for m in REF.finditer(_cell(info, "Kế hoạch đóng") + " " + _cell(info, "Bằng chứng"))]
            if not any(decisions.get(r, "") in ("AGREED", "APPROVED") for r in refs):
                findings.append(Finding("ERROR", where,
                                        f"{dv}: a permanent acceptance must cite an AGREED decision or APPROVED review"))
        if status != "OPEN" and kind == "TEMPORARY" and status == "ACCEPTED":
            findings.append(Finding("ERROR", where, f"{dv}: a TEMPORARY deviation cannot be ACCEPTED as permanent"))
    return findings


def check_traceability(root: pathlib.Path) -> list[Finding]:
    findings: list[Finding] = []
    path = root / "docs/TRACEABILITY.md"
    text = path.read_text(encoding="utf-8")
    decisions = archlib.decision_status(root)
    all_ids = set(decisions)
    design = (root / "docs/architecture/SYSTEM_DESIGN.md").read_text(encoding="utf-8")
    sections = set(re.findall(r"^#{2,4}\s+§(\d+(?:\.\d+)?)", design, re.M))
    open_dv = {k for k, v in deviation_register(root).items()
               if _cell(v, "Trạng thái").split(" ")[0].strip("*`") == "OPEN"}
    index = archlib.EvidenceIndex(root)
    analysis_text = "\n".join(f.read_text(encoding="utf-8") for f in sorted((root / "docs/analysis").glob("*.md")))
    for table in archlib.parse_tables(text):
        if table.col("Trạng thái") is None or table.col("Bằng chứng") is None or table.header[0] in ("ID", "Trạng thái"):
            continue
        status_col, evidence_col = table.col("Trạng thái"), table.col("Bằng chứng")
        spec_col = table.col("Mục spec")
        for number, cells in table.rows:
            where = f"docs/TRACEABILITY.md:{number}"
            if len(cells) != len(table.header):
                findings.append(Finding("ERROR", where, f"row has {len(cells)} cells, header has {len(table.header)}"))
                continue
            status = cells[status_col].split(" ")[0].strip("*`")
            if status not in archlib.STATUSES:
                findings.append(Finding("ERROR", where, f"status {status!r} is not one of {', '.join(archlib.STATUSES)}"))
                continue
            for cell in cells:
                for m in REF.finditer(cell):
                    ref = m.group(0)
                    if ref not in all_ids:
                        findings.append(Finding("ERROR", where, f"reference {ref} is not in DECISIONS.md"))
            if spec_col is not None:
                for sec in SECTION.findall(cells[spec_col]):
                    if sec not in sections:
                        findings.append(Finding("ERROR", where, f"spec section §{sec} not found in SYSTEM_DESIGN.md"))
            spans = archlib.code_spans(cells[evidence_col])
            shas, tests, runs, accepts, dvs = [], [], [], [], []
            for span in spans:
                if span.startswith("run:"):
                    if not RUN_ID.match(span):
                        findings.append(Finding("ERROR", where, f"{span} is not run:<YYYYMMDDTHHMMSSZ>-<name>"))
                    elif span[4:] not in analysis_text:
                        findings.append(Finding("ERROR", where, f"{span} is not recorded in docs/analysis/*.md (add it to the run register)"))
                    else:
                        runs.append(span)
                elif span.startswith("accept:"):
                    accepts.append(span[len("accept:"):])
                elif re.fullmatch(r"DV\d+", span):
                    dvs.append(span)
                elif archlib.SHA.match(span):
                    if index.commit_exists(span):
                        shas.append(span)
                    else:
                        findings.append(Finding("ERROR", where, f"evidence commit {span} not found"))
                elif archlib.TEST_ID.match(span) or archlib.CTEST_NAME.match(span):
                    if index.test_exists(span):
                        tests.append(span)
                    else:
                        findings.append(Finding("ERROR", where, f"evidence test {span} not found in the sources"))
                elif archlib.PATH_LIKE.match(span):
                    if not index.path_exists(span):
                        findings.append(Finding("ERROR", where, f"evidence path {span} does not exist"))
                    elif span == "tools/uavnav/gate.sh" and "Hạ tầng" in table.heading:
                        tests.append(span)
            if status == "IMPLEMENTED" and not shas:
                findings.append(Finding("ERROR", where, "IMPLEMENTED needs at least one existing commit as evidence"))
            if status == "COMPONENT_VERIFIED" and not tests:
                findings.append(Finding("ERROR", where,
                                        "COMPONENT_VERIFIED needs an existing test id as evidence (`tools/uavnav/gate.sh` counts only in the infrastructure table)"))
            if status in ("INTEGRATED", "QUALIFIED"):
                if not shas or not runs:
                    findings.append(Finding("ERROR", where,
                                            f"{status} needs a commit and a `run:<id>` of an integration run as evidence"))
            if status == "QUALIFIED":
                if not any(decisions.get(a, "") in ("AGREED", "APPROVED") for a in accepts):
                    findings.append(Finding("ERROR", where,
                                            "QUALIFIED needs `accept:<id>` naming an AGREED decision or APPROVED review"))
            if status in ("DEVIATED", "BLOCKED"):
                if not any(d in open_dv for d in dvs):
                    findings.append(Finding("ERROR", where, f"{status} needs an OPEN deviation (DV*) in the evidence"))
            for dv in dvs:
                if dv not in deviation_register(root):
                    findings.append(Finding("ERROR", where, f"{dv} is not in the deviation register"))
    return findings


# ---------------------------------------------------------------- dependencies

def _packages(root: pathlib.Path) -> dict[str, pathlib.PurePosixPath]:
    out = {}
    for rel in archlib.git(root, "ls-files", "--", "*package.xml").splitlines():
        try:
            name = ET.parse(root / rel).getroot().findtext("name")
        except ET.ParseError:
            continue
        if name:
            out[name.strip()] = pathlib.PurePosixPath(rel).parent
    return out


def _deps(root: pathlib.Path, package_dir: pathlib.PurePosixPath) -> set[str]:
    tree = ET.parse(root / package_dir / "package.xml").getroot()
    return {(e.text or "").strip() for tag in DEP_TAGS for e in tree.findall(tag)}


def check_dependencies(root: pathlib.Path, rules: dict) -> list[Finding]:
    findings: list[Finding] = []
    packages = _packages(root)
    markers = set(rules["ros_markers"])
    tracked = archlib.git(root, "ls-files").splitlines()

    listed = set(rules["allowed_deps"])
    for name, directory in sorted(packages.items()):
        if name in listed or name in rules["ros_shell_packages"]:
            continue
        if directory.parts[:2] in (("src", "external"), ("src", "vendor")):
            continue
        findings.append(Finding("WARN", f"{directory}/package.xml",
                                f"package {name} is not in arch_rules.yaml allowed_deps (legacy or new: classify it)"))

    for name, allowed in rules["allowed_deps"].items():
        directory = packages.get(name)
        if directory is None:
            findings.append(Finding("WARN", "tools/uavnav/arch_rules.yaml", f"allowed_deps names unknown package {name}"))
            continue
        for dep in sorted(_deps(root, directory)):
            if dep in packages and dep != name and dep not in allowed:
                findings.append(Finding("ERROR", f"{directory}/package.xml",
                                        f"{name} depends on {dep}, which is not in its allowed_deps"))

    for pkg_path, cfg in rules["ros_free"].items():
        allowed_globs = (cfg or {}).get("ros_allowed", [])
        package_files = [p for p in tracked if p.startswith(pkg_path + "/")]
        if not any(p.endswith("package.xml") for p in package_files):
            continue
        if not allowed_globs:
            for dep in sorted(_deps(root, pathlib.PurePosixPath(pkg_path)) & markers):
                findings.append(Finding("ERROR", f"{pkg_path}/package.xml", f"ROS-free package depends on {dep}"))
        for rel in package_files:
            inner = rel[len(pkg_path) + 1:]
            if not rel.endswith((".hpp", ".cpp", ".h", ".cc")) or archlib.matches(inner, allowed_globs):
                continue
            for number, line in enumerate((root / rel).read_text(errors="replace").splitlines(), 1):
                m = INCLUDE.match(line)
                if not m:
                    continue
                header = m.group(1)
                if header.split("/")[0] in markers or header.startswith("uavnav/ros/"):
                    findings.append(Finding("ERROR", f"{rel}:{number}",
                                            f"ROS include {header} in a ROS-free file (allowed only in: "
                                            f"{', '.join(allowed_globs) or 'nowhere'})"))

    banned = re.compile(rules["banned_product_patterns"]["regex"])
    for rel in tracked:
        if not archlib.matches(rel, rules["banned_product_patterns"]["globs"]) or not rel.endswith((".hpp", ".cpp")):
            continue
        for number, line in enumerate((root / rel).read_text(errors="replace").splitlines(), 1):
            code = line.split("//", 1)[0]
            if banned.search(code):
                findings.append(Finding("ERROR", f"{rel}:{number}", "product code writes to stdout/stderr; use the event logger"))
    return findings


def check_invariants(root: pathlib.Path, rules: dict) -> list[Finding]:
    """Invariant ids in arch_rules.yaml and in CHANGE_CONTROL.md must be the same set."""
    text = (root / "docs/CHANGE_CONTROL.md").read_text(encoding="utf-8") if (root / "docs/CHANGE_CONTROL.md").exists() else ""
    in_doc = set(re.findall(r"^\| ((?:LIO|PX4|TIME)-\d+) \|", text, re.M))
    in_yaml = {m.group(1) for k in rules["invariant_paths"] if (m := re.match(r"INV-((?:LIO|PX4|TIME)-\d+)-", k))}
    out = [Finding("ERROR", "docs/CHANGE_CONTROL.md", f"invariant {i} has no entry in arch_rules.yaml invariant_paths")
           for i in sorted(in_doc - in_yaml)]
    out += [Finding("ERROR", "tools/uavnav/arch_rules.yaml", f"invariant {i} is not in the CHANGE_CONTROL.md table")
            for i in sorted(in_yaml - in_doc)]
    return out


def run_checks(root: pathlib.Path, rules: dict, today: datetime.date | None = None,
               strict_dates: bool = False) -> list[Finding]:
    today = today or datetime.date.today()
    return (check_traceability(root) + check_deviations(root, today, strict_dates)
            + check_dependencies(root, rules) + check_invariants(root, rules))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=str(archlib.ROOT))
    parser.add_argument("--strict-dates", action="store_true")
    args = parser.parse_args(argv)
    root = pathlib.Path(args.root).resolve()
    findings = run_checks(root, archlib.load_rules(), strict_dates=args.strict_dates)
    for finding in findings:
        print(finding)
    errors = sum(1 for f in findings if f.level == "ERROR")
    print(f"arch_check: {errors} error(s), {len(findings) - errors} warning(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
