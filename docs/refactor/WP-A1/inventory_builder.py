#!/usr/bin/env python3
"""Build the WP-A1 static C++ symbol inventory and method split.

This is intentionally a source census, not a compiler or runtime model.  It
uses a comment/string-masked brace scan for type definitions and lizard for
function ranges/CCN.  Caller columns are placeholders until
``reference_graph.py`` enriches the CSV from a compilation database; lexical
grep is not a caller oracle in R1.
"""

from __future__ import annotations

import csv
import os
import re
import shutil
import subprocess
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / "docs/refactor/WP-A1"
CPP_EXTENSIONS = {".h", ".hpp", ".cpp", ".cc", ".cxx"}
SKIP_PARTS = {"test", "tests", "external", "ikd_tree_vendor", "ikfom_vendor", "rog_map_vendor", "livox_ros_driver2"}

INVENTORY_COLUMNS = [
    "id",
    "package",
    "file",
    "line",
    "kind",
    "installed_public",
    "responsibility",
    "state_fields",
    "sync_primitives",
    "threads",
    "depends_on",
    "prod_callers_other_tu",
    "prod_callers_same_tu",
    "test_callers",
    "reference_sites",
    "definition_seen_in_ast",
    "max_ccn",
    "current_tier",
    "target_module",
    "action",
    "rationale",
    "evidence",
    "confidence",
]

METHOD_COLUMNS = [
    "method",
    "line_start",
    "line_end",
    "nloc",
    "ccn",
    "members_written",
    "members_read",
    "target_module",
    "notes",
]


@dataclass
class Symbol:
    package: str
    file: Path
    line: int
    token_kind: str
    name: str
    qualified_name: str
    body_start: int | None = None
    body_end: int | None = None
    synthetic: bool = False
    parent: str | None = None
    kind: str = "class"
    namespace: str = ""
    state_fields: int = 0
    sync_primitives: str = ""
    threads: str = "single"
    depends_on: str = ""
    prod_callers_other_tu: int = 0
    prod_callers_same_tu: int = 0
    test_callers: int = 0
    reference_sites: str = ""
    definition_seen_in_ast: str = "false"
    max_ccn: int = 0
    current_tier: str = "L2"
    target_module: str = "nav_core_types"
    action: str = "MOVE"
    rationale: str = ""
    confidence: str = "medium"

    @property
    def identifier(self) -> str:
        return f"{self.package}::{self.qualified_name}"

    @property
    def rel_file(self) -> str:
        return self.file.relative_to(ROOT).as_posix()


@dataclass
class FunctionMetric:
    file: Path
    function: str
    signature: str
    start: int
    end: int
    nloc: int
    ccn: int


def product_files() -> list[Path]:
    files = []
    for path in ROOT.joinpath("src").rglob("*"):
        if path.suffix not in CPP_EXTENSIONS or any(part in SKIP_PARTS for part in path.parts):
            continue
        files.append(path)
    return sorted(files)


def package_for(path: Path) -> tuple[str, Path]:
    for parent in (path.parent, *path.parents):
        package_xml = parent / "package.xml"
        if package_xml.exists():
            match = re.search(r"<name>\s*([^<]+?)\s*</name>", package_xml.read_text(errors="ignore"))
            return (match.group(1) if match else parent.name, parent)
    raise RuntimeError(f"no package.xml for {path}")


def mask_comments_and_strings(text: str) -> str:
    """Mask comments and quoted literals while preserving line/column offsets."""
    chars = list(text)
    i = 0
    state = "code"
    quote = ""
    while i < len(chars):
        if state == "code":
            if text.startswith("//", i):
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "line_comment"
                continue
            if text.startswith("/*", i):
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "block_comment"
                continue
            # C++ digit separators (1'000'000) are not character literals;
            # keeping them in code prevents a brace initializer from changing
            # the member-scan depth.
            if chars[i] == "'" and i > 0 and i + 1 < len(chars) and text[i - 1].isdigit() and text[i + 1].isdigit():
                i += 1
                continue
            if chars[i] in ('"', "'", "`"):
                quote = chars[i]
                chars[i] = " "
                i += 1
                state = "string"
                continue
            i += 1
            continue
        if state == "line_comment":
            if chars[i] == "\n":
                state = "code"
            else:
                chars[i] = " "
            i += 1
            continue
        if state == "block_comment":
            if text.startswith("*/", i):
                chars[i] = chars[i + 1] = " "
                i += 2
                state = "code"
            else:
                if chars[i] != "\n":
                    chars[i] = " "
                i += 1
            continue
        if state == "string":
            if chars[i] == "\\":
                chars[i] = " "
                if i + 1 < len(chars) and chars[i + 1] != "\n":
                    chars[i + 1] = " "
                    i += 2
                else:
                    i += 1
                continue
            if chars[i] == quote:
                chars[i] = " "
                i += 1
                state = "code"
            else:
                if chars[i] != "\n":
                    chars[i] = " "
                i += 1
    return "".join(chars)


def matching_brace(masked: str, open_pos: int) -> int | None:
    depth = 0
    for i in range(open_pos, len(masked)):
        if masked[i] == "{":
            depth += 1
        elif masked[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return None


def namespace_ranges(masked: str) -> list[tuple[int, int, str]]:
    ranges = []
    pattern = re.compile(r"\bnamespace\s+([A-Za-z_]\w*(?:::\w+)*)\s*\{")
    for match in pattern.finditer(masked):
        end = matching_brace(masked, masked.find("{", match.start(), match.end()))
        if end is not None:
            ranges.append((match.start(), end, match.group(1)))
    return ranges


def namespace_at(ranges: list[tuple[int, int, str]], pos: int) -> str:
    containing = [item for item in ranges if item[0] <= pos <= item[1]]
    if not containing:
        return ""
    containing.sort(key=lambda item: item[0], reverse=True)
    return containing[0][2]


def file_namespace(text: str) -> str:
    ranges = namespace_ranges(mask_comments_and_strings(text))
    if not ranges:
        return ""
    return min(ranges, key=lambda item: item[0])[2]


def conditional_variant(text: str, pos: int) -> str:
    """Disambiguate duplicate definitions behind USE_ROS1/USE_ROS2 guards."""
    prefix = text[:pos]
    matches = list(re.finditer(r"^\s*#\s*ifdef\s+(USE_ROS1|USE_ROS2)\b", prefix, re.M))
    return matches[-1].group(1) if matches else ""


def find_type_definitions(path: Path, text: str) -> list[Symbol]:
    masked = mask_comments_and_strings(text)
    ranges = namespace_ranges(masked)
    pattern = re.compile(
        r"\b(class|struct|enum\s+class|enum)\s+"
        r"((?:(?:[A-Za-z_]\w*(?:::[A-Za-z_]\w*)?)\s+)*"
        r"[A-Za-z_]\w*(?:::[A-Za-z_]\w*)?)"
        r"\s*(?::[^{};]*)?\{"
    )
    raw = []
    for match in pattern.finditer(masked):
        brace = masked.find("{", match.start(), match.end())
        end = matching_brace(masked, brace)
        if end is None:
            continue
        declared = match.group(2).strip()
        tokens = [token for token in declared.split() if token != "final"]
        # Visibility macros precede the real name (e.g. ``class
        # NAVIGATION_MAPPING_PUBLIC MappingActor``).  The last token after
        # removing a trailing ``final`` is the C++ name in both forms.
        declared = " ".join(tokens)
        declared_prefix, _, name = declared.rpartition("::")
        if not declared_prefix:
            name = tokens[-1]
            declared_prefix = ""
        namespace = namespace_at(ranges, match.start())
        raw.append((match, brace, end, name, namespace, declared_prefix))

    symbols = []
    for match, brace, end, name, namespace, declared_prefix in raw:
        parents = [item for item in raw if item[1] < match.start() < item[2]]
        parents.sort(key=lambda item: item[1], reverse=True)
        parent_name = parents[0][3] if parents else None
        qualified_parts = []
        if namespace:
            qualified_parts.extend(namespace.split("::"))
        if declared_prefix:
            qualified_parts.extend(declared_prefix.split("::"))
        elif parent_name:
            qualified_parts.append(parent_name)
        qualified_parts.append(name)
        qualified_name = "::".join(qualified_parts)
        variant = conditional_variant(text, match.start())
        if variant:
            qualified_name += "@" + variant
        prefix = masked[max(0, match.start() - 300):match.start()]
        is_template = bool(re.search(r"\btemplate\s*(?:<|requires)", prefix))
        token_kind = match.group(1).replace(" ", "_")
        body = masked[brace + 1:end]
        method_like = bool(re.search(r"\b[A-Za-z_]\w*\s*\([^;{}]*\)\s*(?:const|noexcept|override|final)?\s*(?:\{|;)", body))
        if token_kind.startswith("enum"):
            kind = "enum"
        elif is_template:
            kind = "template"
        elif token_kind == "struct" and not method_like:
            kind = "struct_pod"
        else:
            kind = "struct_logic" if token_kind == "struct" else "class"
        symbols.append(
            Symbol(
                package=package_for(path)[0],
                file=path,
                line=text.count("\n", 0, match.start()) + 1,
                token_kind=token_kind,
                name=name,
                qualified_name=qualified_name,
                body_start=brace + 1,
                body_end=end,
                parent="::".join(qualified_parts[:-1]) or None,
                kind=kind,
                namespace=namespace,
            )
        )
    return symbols


def top_level_function_group(path: Path, text: str, type_symbols: list[Symbol]) -> bool:
    masked = mask_comments_and_strings(text)
    type_ranges = [(s.body_start or 0, s.body_end or 0) for s in type_symbols]
    function_pattern = re.compile(
        r"(?:^|\n)\s*(?:template\s*<[^;{}]+>\s*)?"
        r"[A-Za-z_][\w:<>,~*&\s]*\s+([A-Za-z_]\w*)\s*\([^;{}]*\)"
        r"\s*(?:const|noexcept|override|final|&&|&|->\s*[^{]+)?\s*\{"
    )
    for match in function_pattern.finditer(masked):
        if any(start < match.start() < end for start, end in type_ranges):
            continue
        before = masked[max(0, match.start() - 180):match.start()]
        if "::" in before.split("\n")[-1]:
            continue
        return True
    return False


def field_names(symbol: Symbol, text: str) -> set[str]:
    if symbol.kind == "struct_pod" or symbol.body_start is None or symbol.body_end is None:
        return set()
    masked = mask_comments_and_strings(text)[symbol.body_start:symbol.body_end]
    names: set[str] = set()
    depth = 0
    for line in masked.splitlines():
        if depth == 0 and "(" not in line:
            names.update(re.findall(r"\b([A-Za-z_]\w*_)(?=\s*(?:[;={,)]|$))", line))
        depth += line.count("{") - line.count("}")
        depth = max(0, depth)
    return names


def sync_names(symbol: Symbol, text: str) -> list[str]:
    if symbol.body_start is None or symbol.body_end is None:
        return []
    body = mask_comments_and_strings(text)[symbol.body_start:symbol.body_end]
    primitives = set()
    for line in body.splitlines():
        low = line.lower()
        if "mutex" in low or "shared_mutex" in low:
            primitives.add("mutex")
        if "atomic" in low:
            primitives.add("atomic")
        if "condition_variable" in low or "condvar" in low:
            primitives.add("condvar")
        if "semaphore" in low:
            primitives.add("semaphore")
    return sorted(primitives)


def lizard_metrics(files: list[Path]) -> list[FunctionMetric]:
    executable = os.environ.get("LIZARD") or shutil.which("lizard")
    if not executable:
        raise SystemExit(
            "lizard is required; install it or set LIZARD=/path/to/lizard"
        )
    command = [executable, "-l", "cpp", "--csv", *(str(path) for path in files)]
    result = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=True)
    metrics = []
    for row in csv.reader(result.stdout.splitlines()):
        if len(row) < 11 or not row[0].isdigit():
            continue
        metrics.append(
            FunctionMetric(
                file=ROOT / row[6],
                function=row[7],
                signature=row[8],
                start=int(row[9]),
                end=int(row[10]),
                nloc=int(row[0]),
                ccn=int(row[1]),
            )
        )
    return metrics


def installed_header(package_root: Path, path: Path) -> bool:
    if "include" not in path.parts:
        return False
    cmake = package_root / "CMakeLists.txt"
    if not cmake.exists():
        return False
    text = cmake.read_text(errors="ignore")
    return bool(re.search(r"install\s*\(\s*DIRECTORY\s+include/", text)) or bool(
        re.search(r"install\s*\(\s*FILES[^)]*" + re.escape(path.name), text, re.S)
    )


def tier_for(package: str) -> str:
    if package in {"navigation_common", "navigation_contracts", "navigation_mission", "navigation_planning", "navigation_world_model"}:
        return "L1"
    if package in {"fast_lio_ros", "navigation_runtime", "px4_navigation_external_mode", "px4_odometry_bridge"}:
        return "L3"
    if package in {"fast_lio_tools", "uav_simulation"}:
        return "L4"
    return "L2"


def target_for(symbol: Symbol) -> str:
    package = symbol.package
    path = symbol.rel_file
    name = symbol.name.lower()
    if package == "navigation_common":
        return "nav_core_types"
    if package == "navigation_contracts":
        return "navigation_contracts"
    if package == "navigation_mission":
        return "nav_mission_contract"
    if package == "navigation_world_model":
        return "nav_world_contract"
    if package == "navigation_planning":
        return "nav_plan_contract"
    if package == "fast_lio_core":
        return "lio_core"
    if package == "fast_lio_ros":
        return "lio_node"
    if package == "fast_lio_tools":
        return "nav_judge"
    if package == "navigation_mapping":
        return "nav_world"
    if package == "navigation_execution":
        return "nav_execution"
    if package == "px4_odometry_bridge":
        return "odom_bridge_node" if "node.cpp" in path or symbol.name.endswith("Node") else "odom_bridge_core"
    if package == "px4_navigation_external_mode":
        if any(word in path or word in name for word in ("navigation_mode", "navigationmode", "navigation_mode_node", "paired_node_lifetime")):
            return "px4_adapter_node"
        return "nav_mission" if "mission" in path or "mission" in name else "px4_setpoint_core"
    if package == "uav_simulation":
        return "sitl_harness"
    if package == "navigation_runtime":
        if symbol.name == "NavigationRuntimeNode":
            return "nav_core_node"
        if any(word in path or word in name for word in ("planner_fsm", "planning_supervisor", "desired_planning", "planning_worker")):
            return "nav_planning_policy"
        if "mission_progress" in path or "missionprogress" in name:
            return "nav_mission"
        if any(word in path or word in name for word in ("execution_trace", "execution_lifecycle", "command")):
            return "nav_execution"
        return "nav_core_node"
    if package == "navigation_planning_backend":
        if any(word in path for word in ("trajectory_world_validator", "corridor_plane_validation", "backup_braking", "candidate", "certificate", "evidence_speed_governor")):
            return "nav_certifier"
        return "nav_planner"
    return "DELETE"


def responsibility(symbol: Symbol) -> str:
    if symbol.kind == "enum":
        return f"Mã hóa các trạng thái của {symbol.name}."
    if symbol.kind == "free_fn_group":
        return f"Nhóm và thực thi logic hàm tự do trong {symbol.file.stem}."
    lower = symbol.name.lower()
    verb = "Quản lý"
    if any(x in lower for x in ("validator", "validate", "gate", "authoriz", "admission", "certif")):
        verb = "Kiểm chứng"
    elif any(x in lower for x in ("convert", "transform", "serializer", "adapter", "resolver")):
        verb = "Chuyển đổi"
    elif any(x in lower for x in ("planner", "optimizer", "search", "generator", "trajectory", "corridor")):
        verb = "Tạo hoặc điều phối"
    elif any(x in lower for x in ("worker", "pipeline", "actor", "manager", "store", "buffer")):
        verb = "Điều phối"
    elif any(x in lower for x in ("state", "status", "config", "policy", "result", "snapshot", "identity")):
        verb = "Mô tả hoặc lưu giữ"
    return f"{verb} {symbol.name}."


def rationale_action(symbol: Symbol) -> tuple[str, str]:
    if symbol.synthetic:
        return "MOVE", "File-level free-function group retained until ownership is assigned."
    if symbol.name in {"NavigationRuntimeNode", "Planner", "PlannerFacade"}:
        return "SPLIT", "Tách shell/điều phối khỏi reducer, generator và certification tương ứng."
    if symbol.package == "navigation_runtime" and ("planner_fsm" in symbol.rel_file or symbol.name in {"PlanningSupervisor", "PlanningWorker", "DesiredPlanningIntent"}):
        return "SPLIT", "Tách state reducer, scheduling policy và ROS shell theo ADR-013."
    if symbol.target_module == "nav_certifier":
        return "MOVE", "Validator/certificate hiện thuộc backend nhưng đích là nav_certifier độc lập theo ADR-014."
    if symbol.package in {"navigation_common", "navigation_contracts", "navigation_mission", "navigation_planning", "navigation_world_model"}:
        return "MOVE", "Đưa contract/data về module Tier A tương ứng."
    if symbol.package == "navigation_execution" and symbol.target_module == "nav_execution":
        return "KEEP", "Đã gần đúng execution owner; giữ logic và chỉ đổi namespace/package khi migration."
    return "MOVE", "Di chuyển theo mapping module đích; chưa thay đổi hành vi."


def function_matches_symbol(metric: FunctionMetric, symbol: Symbol) -> bool:
    if symbol.synthetic and metric.file != symbol.file:
        return False
    return bool(re.search(r"::" + re.escape(symbol.name) + r"(?:$|::)", metric.function))


def caller_counts(
    symbol: Symbol,
    product_masked: dict[Path, str],
    test_masked: dict[Path, str],
) -> tuple[int, int]:
    pattern = re.compile(r"\b" + re.escape(symbol.name) + r"\b")
    prod = 0
    for path, text in product_masked.items():
        count = len(pattern.findall(text))
        if path == symbol.file and count:
            count -= 1
        prod += max(0, count)
    tests = sum(bool(pattern.search(text)) for text in test_masked.values())
    return prod, tests


def dependency_map(symbols_by_file: dict[Path, list[Symbol]], product_files_list: list[Path]) -> dict[Path, str]:
    by_include: dict[str, Path] = {}
    for path in product_files_list:
        rel = path.relative_to(ROOT).as_posix()
        by_include[rel] = path
        if "include" in path.parts:
            idx = path.parts.index("include")
            by_include["/".join(path.parts[idx + 1:])] = path
    result: dict[Path, str] = {}
    for path in product_files_list:
        deps: list[str] = []
        text = mask_comments_and_strings(path.read_text(errors="ignore"))
        for include in re.findall(r"#\s*include\s*[<\"]([^>\"]+)[>\"]", text):
            target = by_include.get(include)
            if target is None:
                continue
            for sym in symbols_by_file.get(target, []):
                if not sym.synthetic:
                    short_name = sym.name.rsplit("::", 1)[-1]
                    if re.search(r"\b" + re.escape(short_name) + r"\b", text):
                        deps.append(sym.identifier)
        result[path] = ";".join(dict.fromkeys(deps))
    return result


def build_inventory(files: list[Path], metrics: list[FunctionMetric]) -> list[Symbol]:
    product_text = {path: path.read_text(errors="ignore") for path in files}
    tests = [path for path in ROOT.joinpath("src").rglob("*") if path.is_file() and path.suffix in CPP_EXTENSIONS and "test" in path.parts]
    test_text = {path: path.read_text(errors="ignore") for path in tests}
    product_masked = {path: mask_comments_and_strings(text) for path, text in product_text.items()}
    test_masked = {path: mask_comments_and_strings(text) for path, text in test_text.items()}
    symbols_by_file: dict[Path, list[Symbol]] = {}
    symbols: list[Symbol] = []
    for path in files:
        found = find_type_definitions(path, product_text[path])
        symbols_by_file[path] = found
        symbols.extend(found)
        if not found or top_level_function_group(path, product_text[path], found):
            package = package_for(path)[0]
            namespace = next((s.namespace for s in found if s.namespace), file_namespace(product_text[path]))
            stem = re.sub(r"[^A-Za-z0-9_]+", "_", path.stem)
            role = "header" if "/include/" in path.as_posix() else "source"
            name = f"{stem}_{role}_free_functions"
            symbols.append(
                Symbol(
                    package=package,
                    file=path,
                    line=1,
                    token_kind="free_fn_group",
                    name=name,
                    qualified_name=f"{namespace}::{name}" if namespace else name,
                    synthetic=True,
                    kind="free_fn_group",
                    namespace=namespace,
                    confidence="low",
                )
            )
    deps = dependency_map(symbols_by_file, files)
    for symbol in symbols:
        package, package_root = package_for(symbol.file)
        symbol.state_fields = len(field_names(symbol, product_text[symbol.file]))
        sync = sync_names(symbol, product_text[symbol.file])
        symbol.sync_primitives = ";".join(sync)
        body = product_text[symbol.file][symbol.body_start:symbol.body_end] if symbol.body_start is not None and symbol.body_end is not None else ""
        if symbol.name == "NavigationRuntimeNode":
            symbol.threads = "ROS callbacks/timers; MappingWorker; PlanningWorker; HeadingRebindWorker"
        elif any(x in body for x in ("std::thread", "std::jthread")):
            symbol.threads = "thread/worker"
        elif any(x in symbol.name.lower() for x in ("worker", "actor", "pipeline")):
            symbol.threads = "worker/callback"
        elif symbol.package in {"fast_lio_ros", "navigation_runtime", "px4_navigation_external_mode", "px4_odometry_bridge"}:
            symbol.threads = "ROS callback/timer"
        symbol.depends_on = deps.get(symbol.file, "")
        # Caller counts are intentionally left blank/zero here.  The final
        # inventory must be enriched by reference_graph.py using libclang;
        # lexical grep is not accepted as a caller oracle in R1.
        _lexical_prod, symbol.test_callers = caller_counts(symbol, product_masked, test_masked)
        symbol.prod_callers_other_tu = 0
        symbol.prod_callers_same_tu = 0
        symbol.definition_seen_in_ast = "false"
        symbol.max_ccn = max((m.ccn for m in metrics if function_matches_symbol(m, symbol)), default=0)
        symbol.current_tier = tier_for(package)
        symbol.target_module = target_for(symbol)
        symbol.action, symbol.rationale = rationale_action(symbol)
        if symbol.prod_callers_other_tu + symbol.prod_callers_same_tu == 0 and not symbol.synthetic and symbol.name not in {"main", "NavigationRuntimeNode", "NavigationMode", "Px4OdometryBridgeNode", "Px4ExternalOdometryBridgeNode"}:
            symbol.action = "DELETE"
            symbol.rationale = "0 direct production caller theo grep trên baseline; cần xác nhận trước khi xóa."
        elif symbol.action == "MOVE" and symbol.kind in {"class", "struct_logic", "template"} and symbol.max_ccn * symbol.state_fields >= 300:
            symbol.action = "SPLIT"
            symbol.rationale = "Tách state lớn hoặc logic phức tạp thành reducer, core và boundary theo module đích."
        symbol.responsibility = responsibility(symbol)
        symbol.confidence = "low"
    return symbols


def method_target(function: str, file: Path) -> str:
    if "NavigationRuntimeNode" in function:
        name = function.rsplit("::", 1)[-1].lower()
        if any(x in name for x in ("publishcommand", "admit", "commit", "clearcommand", "suspendcommand", "executionrecovery", "failclosed")):
            return "nav_execution"
        if any(x in name for x in ("goal", "mission", "mode", "commandadmission")):
            return "nav_mission"
        if any(x in name for x in ("planning", "heading", "cycle", "retained", "candidate", "rebind")):
            return "nav_planning_policy"
        return "nav_core_node"
    if "Planner" in function:
        if any(x in function.lower() for x in ("validate", "certificate", "emergency", "backup", "candidateexport")):
            return "nav_certifier"
        return "nav_planner"
    return "nav_core_node" if "navigation_runtime_node.cpp" in file.as_posix() else "nav_planner"


def member_rw(metric: FunctionMetric, fields: set[str], text: str) -> tuple[str, str]:
    if not fields:
        return "", ""
    lines = text.splitlines()[metric.start - 1:metric.end]
    body = "\n".join(lines)
    read = {name for name in fields if re.search(r"\b" + re.escape(name) + r"\b", body)}
    written = set()
    for name in fields:
        direct_write = re.search(
            r"\b" + re.escape(name) + r"\b\s*(?:\[[^\]]+\])?\s*(?:[+\-*/%]?=|\+\+|--)",
            body,
        )
        mutating_call = re.search(
            r"\b" + re.escape(name) + r"\b\s*(?:\.|->)\s*"
            r"(?:store|exchange|fetch_add|fetch_sub|reset|clear|push|emplace|insert|erase|assign|update|advance|record|commit|publish|transition|set[A-Z]\w*)\s*\(",
            body,
        )
        first_body = body.find("{")
        in_initializer = first_body >= 0 and bool(
            re.search(r"\b" + re.escape(name) + r"\b\s*[({]", body[:first_body])
        )
        if direct_write or mutating_call or in_initializer:
            written.add(name)
    return ";".join(sorted(written)), ";".join(sorted(read))


def build_method_split(files: list[Path], metrics: list[FunctionMetric], symbols: list[Symbol]) -> list[dict[str, str]]:
    wanted = [m for m in metrics if "NavigationRuntimeNode" in m.function or "::Planner::" in m.function]
    node_methods = [m for m in wanted if "NavigationRuntimeNode" in m.function]
    planner_methods = [m for m in wanted if "::Planner::" in m.function]
    node_symbol = next((s for s in symbols if s.name == "NavigationRuntimeNode"), None)
    planner_symbol = next((s for s in symbols if s.name == "Planner" and "planning_backend" in s.rel_file), None)
    node_fields = field_names(node_symbol, node_symbol.file.read_text(errors="ignore")) if node_symbol else set()
    planner_fields = field_names(planner_symbol, planner_symbol.file.read_text(errors="ignore")) if planner_symbol else set()
    rows = []
    for metric in wanted:
        text = metric.file.read_text(errors="ignore")
        is_node = "NavigationRuntimeNode" in metric.function
        fields = node_fields if is_node else planner_fields
        written, read = member_rw(metric, fields, text)
        label = metric.function
        rows.append(
            {
                "method": label,
                "line_start": str(metric.start),
                "line_end": str(metric.end),
                "nloc": str(metric.nloc),
                "ccn": str(metric.ccn),
                "members_written": written,
                "members_read": read,
                "target_module": method_target(metric.function, metric.file),
                "notes": "lizard CCN/NLOC; members read/write là static token scan theo hậu tố _.",
            }
        )
    # Keep a deterministic order and remove only exact duplicate metric entries.
    unique = []
    seen = set()
    for row in sorted(rows, key=lambda item: (item["line_start"], item["method"], item["line_end"])):
        key = (row["method"], row["line_start"], row["line_end"])
        if key not in seen:
            seen.add(key)
            unique.append(row)
    return unique


def write_inventory(symbols: list[Symbol]) -> None:
    with (OUT / "class_inventory.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=INVENTORY_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for symbol in sorted(symbols, key=lambda s: (s.package, s.rel_file, s.line, s.identifier)):
            writer.writerow(
                {
                    "id": symbol.identifier,
                    "package": symbol.package,
                    "file": symbol.rel_file,
                    "line": symbol.line,
                    "kind": symbol.kind,
                    "installed_public": str(installed_header(package_for(symbol.file)[1], symbol.file)).lower(),
                    "responsibility": symbol.responsibility,
                    "state_fields": symbol.state_fields,
                    "sync_primitives": symbol.sync_primitives,
                    "threads": symbol.threads,
                    "depends_on": symbol.depends_on,
                    "prod_callers_other_tu": symbol.prod_callers_other_tu,
                    "prod_callers_same_tu": symbol.prod_callers_same_tu,
                    "test_callers": symbol.test_callers,
                    "reference_sites": symbol.reference_sites,
                    "definition_seen_in_ast": symbol.definition_seen_in_ast,
                    "max_ccn": symbol.max_ccn,
                    "current_tier": symbol.current_tier,
                    "target_module": symbol.target_module,
                    "action": symbol.action,
                    "rationale": symbol.rationale,
                    "evidence": f"{symbol.rel_file}:{symbol.line}",
                    "confidence": symbol.confidence,
                }
            )


def write_methods(rows: list[dict[str, str]]) -> None:
    with (OUT / "method_split_navigation_runtime_node.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=METHOD_COLUMNS, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    files = product_files()
    metrics = lizard_metrics(files + [ROOT / "src/runtime/navigation_runtime/include/navigation_runtime/navigation_runtime_node.hpp", ROOT / "src/planning/navigation_planning_backend/include/planner_core/planner.hpp"])
    symbols = build_inventory(files, metrics)
    write_inventory(symbols)
    write_methods(build_method_split(files, metrics, symbols))
    print(f"product_files={len(files)} symbols={len(symbols)} methods={len(build_method_split(files, metrics, symbols))}")
    print("packages=" + ";".join(f"{k}:{v}" for k, v in sorted(Counter(s.package for s in symbols).items())))


if __name__ == "__main__":
    main()
