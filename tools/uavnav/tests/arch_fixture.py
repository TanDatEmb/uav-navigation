"""Throw-away Git repositories for the architecture-governance tests. No network, no ROS."""
import copy
import os
import re
import pathlib
import subprocess
import tempfile

from tools.uavnav import archlib

DESIGN = "# Design\n\n## §3. LIO\n\n### §3.1 State machine\n\n## §4. PX4\n\n### §4.1 Bridge\n"

DECISIONS = """# Decisions

## 2. Decisions

| ID | Decision | Status |
|---|---|---|
| D1 | something | AGREED |
| D2 | proposal | RECOMMENDED |

## 5. Architecture Review (AR)

| ID | Ngày | Hệ con | Tín hiệu | Phân tích | Window end | Owner xác nhận | Kết quả |
|---|---|---|---|---|---|---|---|
"""

TRACE_HEAD = """# Trace

| Yêu cầu | Mục spec | Lát | WP | Trạng thái | Bằng chứng |
|---|---|---|---|---|---|
"""

DV_HEAD = """
## Lệch

| ID | Hệ con | Thiết kế gốc | Thực tế | Lý do | Rủi ro và phạm vi | Loại | Owner | Xem lại khi | Kế hoạch đóng | Bằng chứng | Trạng thái |
|---|---|---|---|---|---|---|---|---|---|---|---|
"""


def dv_row(ident="DV1", kind="TEMPORARY", owner="owner", review="2999-01-01", plan="close it", evidence="", status="OPEN"):
    return (f"| {ident} | lio | §3.1 | actual | reason | risk | {kind} | {owner} | {review} | {plan} | {evidence} | {status} |\n")


class Repo:
    def __init__(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = pathlib.Path(self._tmp.name)
        self.run("init", "-q", "-b", "main")
        self.run("config", "user.email", "t@example.com")
        self.run("config", "user.name", "t")
        self.rules = copy.deepcopy(archlib.load_rules())
        self.write("docs/architecture/SYSTEM_DESIGN.md", DESIGN)
        self.write("docs/architecture/DECISIONS.md", DECISIONS)
        self.write("docs/TRACEABILITY.md", TRACE_HEAD + DV_HEAD)
        self.write("docs/analysis/runs.md", "Runs: 20261013T152418Z-gps-hover 20261013T154709Z-gps-hover\n")
        ids = [m.group(1) for k in self.rules["invariant_paths"] if (m := re.match(r"INV-((?:LIO|PX4|TIME)-\d+)-", k))]
        self.write("docs/CHANGE_CONTROL.md", "".join(f"| {i} | invariant |\n" for i in ids))
        self.write("src/estimation/uavnav_lio_core/src/other.cpp", "int a = 0;\n")
        self.write("src/estimation/uavnav_lio_core/include/uavnav/lio/state.hpp", "struct S {\n};\n")
        self.base = self.commit("chore: baseline")

    def close(self):
        self._tmp.cleanup()

    def run(self, *args):
        env = dict(os.environ, GIT_CONFIG_GLOBAL="/dev/null", GIT_CONFIG_SYSTEM="/dev/null")
        result = subprocess.run(["git", "-C", str(self.root), *args], capture_output=True, text=True, env=env)
        assert result.returncode == 0, result.stderr
        return result.stdout

    def write(self, rel, text):
        path = self.root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(text)

    def commit(self, message, trailers=""):
        self.run("add", "-A")
        self.run("commit", "-q", "-m", message + ("\n\n" + trailers if trailers else ""))
        return self.run("rev-parse", "HEAD").strip()

    def head(self):
        return self.run("rev-parse", "HEAD").strip()

    def set_trace(self, rows, dvs=""):
        self.write("docs/TRACEABILITY.md", TRACE_HEAD + rows + DV_HEAD + dvs)

    def add_ar(self, ident, subsystem, window_end, result, confirmed="có"):
        path = self.root / "docs/architecture/DECISIONS.md"
        path.write_text(path.read_text() + f"| {ident} | 2026-01-01 | {subsystem} | sig | analysis | `{window_end}` | {confirmed} | {result} |\n")
