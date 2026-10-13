"""Shared helpers for the architecture-governance tools (docs/CHANGE_CONTROL.md, D34).

Standard library plus PyYAML. Nothing here judges design: it parses the repository's own records
(Git, TRACEABILITY.md, DECISIONS.md, arch_rules.yaml) so the checks stay objective.
"""
from __future__ import annotations

import fnmatch
import pathlib
import re
import subprocess
from dataclasses import dataclass, field

import yaml

ROOT = pathlib.Path(__file__).resolve().parents[2]
RULES_PATH = pathlib.Path(__file__).resolve().parent / "arch_rules.yaml"

# P2 vocabulary. The first five are an ordered ladder; DEVIATED and BLOCKED are side states that must
# name an open deviation (DV*).
LADDER = ["DESIGNED", "IMPLEMENTED", "COMPONENT_VERIFIED", "INTEGRATED", "QUALIFIED"]
SIDE = ["DEVIATED", "BLOCKED"]
STATUSES = LADDER + SIDE


def load_rules(path: pathlib.Path = RULES_PATH) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def git(root: pathlib.Path, *args: str, check: bool = True) -> str:
    result = subprocess.run(["git", "-C", str(root), *args], capture_output=True, text=True)
    if check and result.returncode != 0:
        raise RuntimeError(f"git {' '.join(args)}: {result.stderr.strip()}")
    return result.stdout


def matches(path: str, patterns: list[str]) -> bool:
    return any(fnmatch.fnmatchcase(path, p) for p in patterns)


def subsystem_of(path: str, rules: dict) -> str | None:
    for name, prefixes in rules["subsystems"].items():
        if any(path == p or path.startswith(p.rstrip("/") + "/") for p in prefixes):
            return name
    return None


def is_test_path(path: str) -> bool:
    parts = path.split("/")
    return "test" in parts or "tests" in parts or pathlib.PurePosixPath(path).name.startswith("test_")


def is_doc_path(path: str) -> bool:
    return path.startswith("docs/") or path == "AGENTS.md" or path.endswith(".md")


# ---------------------------------------------------------------- markdown tables

def split_row(line: str) -> list[str]:
    """Split a Markdown table row on `|`, ignoring pipes inside backtick spans."""
    cells, current, in_code = [], [], False
    body = line.strip()
    if body.startswith("|"):
        body = body[1:]
    if body.endswith("|") and not body.endswith("\\|"):
        body = body[:-1]
    for ch in body:
        if ch == "`":
            in_code = not in_code
        if ch == "|" and not in_code:
            cells.append("".join(current).strip())
            current = []
        else:
            current.append(ch)
    cells.append("".join(current).strip())
    return cells


@dataclass
class Table:
    heading: str
    header: list[str]
    rows: list[tuple[int, list[str]]] = field(default_factory=list)  # (1-based line number, cells)

    def col(self, *needles: str) -> int | None:
        for index, name in enumerate(self.header):
            if all(n.lower() in name.lower() for n in needles):
                return index
        return None


def parse_tables(text: str) -> list[Table]:
    tables: list[Table] = []
    heading = ""
    current: Table | None = None
    for number, line in enumerate(text.splitlines(), 1):
        if line.startswith("#"):
            heading = line.lstrip("#").strip()
            current = None
            continue
        if not line.lstrip().startswith("|"):
            current = None
            continue
        cells = split_row(line)
        if current is None:
            current = Table(heading, cells)
            tables.append(current)
        elif all(re.fullmatch(r":?-{2,}:?", c) for c in cells if c):
            continue
        else:
            current.rows.append((number, cells))
    return tables


# ---------------------------------------------------------------- decisions

def decision_status(root: pathlib.Path) -> dict[str, str]:
    """ID -> status word for D*, O*, AR* rows of DECISIONS.md (last cell = status/result)."""
    path = root / "docs/architecture/DECISIONS.md"
    result: dict[str, str] = {}
    for table in parse_tables(path.read_text(encoding="utf-8")):
        for _, cells in table.rows:
            if len(cells) >= 2 and re.fullmatch(r"(D|O|AR)\d+", cells[0]):
                result[cells[0]] = cells[-1].strip().strip("*`").split(" ")[0] if cells[-1].strip() else ""
    return result


# ---------------------------------------------------------------- evidence resolution

TEST_ID = re.compile(r"^[A-Z]\w*\.(?:[A-Z]\w*|test_\w+)$")
CTEST_NAME = re.compile(r"^test_\w+$")
SHA = re.compile(r"^[0-9a-f]{7,40}$")
PATH_LIKE = re.compile(r"^(?:src|tools|docs|config|test-results)/[\w./-]+?(?::\d+(?:-\d+)?)?$")
GTEST = re.compile(r"\bTEST(?:_F|_P)?\s*\(\s*(\w+)\s*,\s*(\w+)")
PY_CLASS = re.compile(r"^class\s+(\w+)")
PY_DEF = re.compile(r"^\s+def\s+(test_\w+)")


class EvidenceIndex:
    """Lazy index of test identifiers, test targets and commits in the working tree."""

    def __init__(self, root: pathlib.Path):
        self.root = root
        self._ids: set[str] | None = None
        self._stems: set[str] | None = None
        self._cmake: str | None = None

    def _tracked(self) -> list[str]:
        return git(self.root, "ls-files").splitlines()

    def _build(self) -> None:
        ids, stems, cmake = set(), set(), []
        for rel in self._tracked():
            p = self.root / rel
            name = p.name
            if name == "CMakeLists.txt":
                cmake.append(p.read_text(errors="replace"))
            if p.suffix in (".cpp", ".hpp", ".cc"):
                stems.add(p.stem)
                for suite, case in GTEST.findall(p.read_text(errors="replace")):
                    ids.add(f"{suite}.{case}")
            elif p.suffix == ".py":
                stems.add(p.stem)
                cls = None
                for line in p.read_text(errors="replace").splitlines():
                    m = PY_CLASS.match(line)
                    if m:
                        cls = m.group(1)
                    m = PY_DEF.match(line)
                    if m and cls:
                        ids.add(f"{cls}.{m.group(1)}")
        self._ids, self._stems, self._cmake = ids, stems, "\n".join(cmake)

    def test_exists(self, token: str) -> bool:
        if self._ids is None:
            self._build()
        if TEST_ID.match(token):
            return token in self._ids
        return token in self._stems or token in self._cmake

    def commit_exists(self, sha: str) -> bool:
        return subprocess.run(["git", "-C", str(self.root), "cat-file", "-e", f"{sha}^{{commit}}"],
                              capture_output=True).returncode == 0

    def path_exists(self, token: str) -> bool:
        return (self.root / token.split(":", 1)[0]).exists()


def code_spans(cell: str) -> list[str]:
    return re.findall(r"`([^`]+)`", cell)
