#!/usr/bin/env python3
"""Build a compiler-backed reference graph for the WP-A1 inventory.

The old inventory used token grep.  This script consumes CMake's compilation
database and libclang's AST instead.  A reference is counted only when clang
resolves it to an inventory symbol and the reference is outside that symbol's
definition scope.  The graph intentionally separates production references
in another translation unit, production references in the definition's own
translation unit, and references originating in tests.

This is still static evidence: unresolved macros, generated code that is not
present in the compilation database, and runtime/plugin discovery are reported
as limitations rather than silently treated as callers.
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import shlex
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

try:
    from clang import cindex
except ImportError as exc:  # pragma: no cover - exercised by the environment
    raise SystemExit(
        "clang.cindex is required; use the WP-A1 clang environment"
    ) from exc


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE_SUFFIXES = {".h", ".hpp", ".hh", ".cpp", ".cc", ".cxx"}
SKIP_PARTS = {"external", "ikd_tree_vendor", "ikfom_vendor", "rog_map_vendor", "livox_ros_driver2"}
REFERENCE_KINDS = {
    cindex.CursorKind.DECL_REF_EXPR,
    cindex.CursorKind.TYPE_REF,
    cindex.CursorKind.MEMBER_REF_EXPR,
    cindex.CursorKind.CALL_EXPR,
    cindex.CursorKind.TEMPLATE_REF,
}


@dataclass
class CompilationUnit:
    source: Path
    arguments: list[str]
    directory: Path
    is_test: bool


@dataclass
class SymbolInfo:
    row: dict[str, str]
    definition_ranges: list[tuple[Path, int, int, int, int]] = field(default_factory=list)
    member_ranges: list[tuple[Path, int, int, int, int]] = field(default_factory=list)
    usrs: set[str] = field(default_factory=set)

    @property
    def identifier(self) -> str:
        return self.row["id"]

    @property
    def file(self) -> Path:
        return ROOT / self.row["file"]

    @property
    def kind(self) -> str:
        return self.row["kind"]


def configure_libclang(path: str | None) -> None:
    if path:
        cindex.Config.set_library_file(path)
        return
    candidates = [
        Path("/usr/lib/x86_64-linux-gnu/libclang-18.so.1"),
        Path("/usr/lib/llvm-18/lib/libclang.so.1"),
        Path("/usr/lib/llvm-18/lib/libclang.so"),
    ]
    for candidate in candidates:
        if candidate.exists():
            cindex.Config.set_library_file(str(candidate))
            return


def source_in_scope(path: Path, repo: Path) -> bool:
    try:
        rel = path.relative_to(repo)
    except ValueError:
        return False
    return (
        "src" in rel.parts
        and path.suffix in SOURCE_SUFFIXES
        and not any(part in SKIP_PARTS or part == "external" for part in rel.parts)
    )


def is_test_path(path: Path) -> bool:
    return any(part in {"test", "tests"} or part.startswith("test_") for part in path.parts)


def command_arguments(entry: dict[str, object]) -> tuple[Path, list[str], Path]:
    source = Path(str(entry["file"])).resolve()
    directory = Path(str(entry.get("directory", source.parent))).resolve()
    raw = entry.get("arguments")
    if raw:
        command = [str(value) for value in raw]  # type: ignore[arg-type]
    else:
        command = shlex.split(str(entry.get("command", "")))
    args = command[1:]
    cleaned: list[str] = []
    skip_next = False
    source_strings = {str(source), str(Path(str(entry["file"]))), source.name}
    for arg in args:
        if skip_next:
            skip_next = False
            continue
        if arg in {"-c", "--compile"}:
            continue
        if arg in {"-o", "--output", "-MF", "-MT", "-MQ"}:
            skip_next = True
            continue
        if arg in source_strings:
            continue
        if arg.endswith(".o") and ("CMakeFiles" in arg or "/" not in arg):
            continue
        cleaned.append(arg)
    # clang needs the working directory explicitly when an entry contains
    # relative include paths.  This is accepted by both clang and libclang.
    cleaned.insert(0, f"-working-directory={directory}")
    if source.suffix in {".h", ".hpp", ".hh"}:
        cleaned.extend(["-x", "c++-header"])
    return source, cleaned, directory


def load_compilation_units(repo: Path, locations: list[Path]) -> list[CompilationUnit]:
    databases: list[Path] = []
    for location in locations:
        location = location.resolve()
        if location.is_dir():
            databases.extend(sorted(location.rglob("compile_commands.json")))
        elif location.name == "compile_commands.json":
            databases.append(location)
    units: dict[Path, CompilationUnit] = {}
    for database in sorted(set(databases)):
        try:
            entries = json.loads(database.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            print(f"warning: cannot read {database}: {exc}", file=sys.stderr)
            continue
        for entry in entries:
            source, arguments, directory = command_arguments(entry)
            if not source_in_scope(source, repo):
                continue
            # Prefer the aggregate database's command when duplicate package
            # databases exist; all valid entries for a source are equivalent.
            units.setdefault(
                source,
                CompilationUnit(source, arguments, directory, is_test_path(source)),
            )
    return [units[path] for path in sorted(units)]


def cursor_file(cursor: cindex.Cursor) -> Path | None:
    try:
        location = cursor.location
        if location.file is None:
            return None
        path = Path(location.file.name)
        return path if path.is_absolute() else ROOT / path
    except (AttributeError, cindex.LibclangError):
        return None


def cursor_name(cursor: cindex.Cursor) -> str:
    names: list[str] = []
    current = cursor
    seen: set[int] = set()
    while current is not None:
        try:
            identity = current.hash
        except AttributeError:
            identity = id(current)
        if identity in seen:
            break
        seen.add(identity)
        spelling = current.spelling or current.displayname.split("(", 1)[0]
        if spelling and spelling not in {"translation_unit", "::"}:
            names.append(spelling)
        try:
            current = current.semantic_parent
        except cindex.LibclangError:
            break
    return "::".join(reversed(names))


def cursor_usr(cursor: cindex.Cursor) -> str:
    try:
        return cursor.get_usr() or ""
    except cindex.LibclangError:
        return ""


def location_tuple(cursor: cindex.Cursor) -> tuple[Path | None, int, int, int]:
    try:
        location = cursor.location
        return (
            cursor_file(cursor),
            int(location.line or 0),
            int(location.column or 0),
            int(location.offset or 0),
        )
    except (AttributeError, cindex.LibclangError):
        return None, 0, 0, 0


def extent_tuple(cursor: cindex.Cursor) -> tuple[Path | None, int, int, int, int]:
    try:
        extent = cursor.extent
        start = extent.start
        end = extent.end
        return (
            cursor_file(cursor),
            int(start.offset or 0),
            int(end.offset or 0),
            int(start.line or 0),
            int(end.line or 0),
        )
    except (AttributeError, cindex.LibclangError):
        return None, 0, 0, 0, 0


def row_qualified_name(row: dict[str, str]) -> str:
    prefix = row["package"] + "::"
    return row["id"][len(prefix) :] if row["id"].startswith(prefix) else row["id"]


def load_inventory(path: Path) -> tuple[list[dict[str, str]], dict[str, SymbolInfo], dict[tuple[str, int], SymbolInfo]]:
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    infos = {row["id"]: SymbolInfo(row) for row in rows}
    anchors: dict[tuple[str, int], SymbolInfo] = {}
    for info in infos.values():
        anchors[(info.row["file"], int(info.row["line"]))] = info
    return rows, infos, anchors


def walk(cursor: cindex.Cursor):
    # Included system/ROS/vendor ASTs dominate the tree and cannot resolve to
    # an in-scope inventory row.  Prune them before descending; product
    # headers included by a product TU remain fully traversed.
    if cursor.kind != cindex.CursorKind.TRANSLATION_UNIT:
        path = cursor_file(cursor)
        if path is None or not source_in_scope(path, ROOT):
            return
    yield cursor
    try:
        children = list(cursor.get_children())
    except cindex.LibclangError:
        return
    for child in children:
        yield from walk(child)


def cursor_kind_is_type(cursor: cindex.Cursor) -> bool:
    return cursor.kind in {
        cindex.CursorKind.CLASS_DECL,
        cindex.CursorKind.STRUCT_DECL,
        cindex.CursorKind.ENUM_DECL,
        cindex.CursorKind.CLASS_TEMPLATE,
        cindex.CursorKind.TYPE_ALIAS_DECL,
    }


def build_ast_index(
    translation_unit: cindex.TranslationUnit,
    infos: dict[str, SymbolInfo],
    anchors: dict[tuple[str, int], SymbolInfo],
    class_usr_to_info: dict[str, SymbolInfo],
    usr_to_info: dict[str, SymbolInfo],
    free_groups: dict[Path, list[SymbolInfo]],
) -> None:
    for cursor in walk(translation_unit.cursor):
        path, line, _column, _offset = location_tuple(cursor)
        if path is None or not cursor.is_definition():
            continue
        try:
            rel = path.relative_to(ROOT).as_posix()
        except ValueError:
            continue
        extent = extent_tuple(cursor)
        if cursor.kind in {
            cindex.CursorKind.CXX_METHOD,
            cindex.CursorKind.CONSTRUCTOR,
            cindex.CursorKind.DESTRUCTOR,
            cindex.CursorKind.FUNCTION_TEMPLATE,
        }:
            owner = parent_type_info(cursor, class_usr_to_info)
            if owner is not None and extent[0] is not None:
                owner.member_ranges.append(extent)
        direct = anchors.get((rel, line))
        name = cursor_name(cursor)
        candidate = direct
        if candidate is None:
            for info in infos.values():
                if info.row["file"] == rel and row_qualified_name(info.row) == name:
                    candidate = info
                    break
        if candidate is None:
            continue
        usr = cursor_usr(cursor)
        if usr:
            usr_to_info.setdefault(usr, candidate)
            if cursor_kind_is_type(cursor):
                class_usr_to_info.setdefault(usr, candidate)
                candidate.usrs.add(usr)
        if extent[0] is not None:
            candidate.definition_ranges.append(extent)
        # A function declaration in a file-level synthetic row is resolved by
        # its source file, not by the synthetic name.
        if cursor.kind in {
            cindex.CursorKind.FUNCTION_DECL,
            cindex.CursorKind.FUNCTION_TEMPLATE,
            cindex.CursorKind.CXX_METHOD,
            cindex.CursorKind.CONSTRUCTOR,
            cindex.CursorKind.DESTRUCTOR,
        }:
            free_groups[path].append(candidate)


def parent_type_info(cursor: cindex.Cursor, class_usr_to_info: dict[str, SymbolInfo]) -> SymbolInfo | None:
    current = cursor
    for _ in range(12):
        try:
            current = current.semantic_parent
        except cindex.LibclangError:
            return None
        if current is None:
            return None
        usr = cursor_usr(current)
        if usr in class_usr_to_info:
            return class_usr_to_info[usr]
    return None


def referenced_cursor(cursor: cindex.Cursor) -> cindex.Cursor | None:
    try:
        referenced = cursor.referenced
        return referenced if referenced is not None and referenced.kind != cindex.CursorKind.NO_DECL_FOUND else None
    except cindex.LibclangError:
        return None


def resolve_target(
    cursor: cindex.Cursor,
    usr_to_info: dict[str, SymbolInfo],
    class_usr_to_info: dict[str, SymbolInfo],
    anchors: dict[tuple[str, int], SymbolInfo],
    free_groups: dict[Path, list[SymbolInfo]],
) -> SymbolInfo | None:
    target = referenced_cursor(cursor)
    if target is None:
        return None
    for candidate in (target, target.canonical):
        usr = cursor_usr(candidate)
        if usr and usr in usr_to_info:
            return usr_to_info[usr]
        path, line, _column, _offset = location_tuple(candidate)
        if path is not None:
            try:
                rel = path.relative_to(ROOT).as_posix()
            except ValueError:
                rel = ""
            if rel and (rel, line) in anchors:
                return anchors[(rel, line)]
    if target.kind == cindex.CursorKind.ENUM_CONSTANT_DECL:
        parent = parent_type_info(target, class_usr_to_info)
        if parent is not None:
            return parent
    target_file = cursor_file(target)
    if target_file is not None and target_file in free_groups:
        # Do not manufacture a caller for arbitrary system functions.  A
        # synthetic group is only a match when clang's declaration is in the
        # product file represented by that group.
        return free_groups[target_file][0] if free_groups[target_file] else None
    return None


def inside_definition_scope(cursor: cindex.Cursor, target: SymbolInfo) -> bool:
    path, line, _column, offset = location_tuple(cursor)
    if path is None:
        return False
    for file_path, begin, end, begin_line, end_line in target.definition_ranges:
        if file_path != path:
            continue
        if begin and end and offset and begin <= offset <= end:
            return True
        if begin_line and end_line and begin_line <= line <= end_line:
            return True
    for file_path, begin, end, begin_line, end_line in target.member_ranges:
        if file_path != path:
            continue
        if begin and end and offset and begin <= offset <= end:
            return True
        if begin_line and end_line and begin_line <= line <= end_line:
            return True
    return False


def inside_member_scope(cursor: cindex.Cursor, target: SymbolInfo, class_usr_to_info: dict[str, SymbolInfo]) -> bool:
    current = cursor
    for _ in range(16):
        try:
            current = current.semantic_parent
        except cindex.LibclangError:
            return False
        if current is None:
            return False
        parent = parent_type_info(current, class_usr_to_info)
        if parent is target:
            return True
        usr = cursor_usr(current)
        if usr in target.usrs:
            return True
    return False


def write_graph(path: Path, rows: list[dict[str, str]], counts: dict[str, dict[str, object]]) -> None:
    fields = [
        "id",
        "prod_callers_other_tu",
        "prod_callers_same_tu",
        "test_callers",
        "reference_sites",
        "definition_seen_in_ast",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            item = counts[row["id"]]
            output = {field: item.get(field, "") for field in fields}
            output["id"] = row["id"]
            writer.writerow(output)


def update_inventory(path: Path, rows: list[dict[str, str]], counts: dict[str, dict[str, object]]) -> None:
    old_fields = list(rows[0]) if rows else []
    fields = [
        field
        for field in old_fields
        if field not in {"prod_callers", "test_callers", "reference_sites", "definition_seen_in_ast"}
    ]
    insert_at = fields.index("depends_on") + 1 if "depends_on" in fields else len(fields)
    fields[insert_at:insert_at] = [
        "prod_callers_other_tu",
        "prod_callers_same_tu",
        "test_callers",
        "reference_sites",
        "definition_seen_in_ast",
    ]
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        for row in rows:
            item = counts[row["id"]]
            result = {field: row.get(field, "") for field in fields}
            for field in (
                "prod_callers_other_tu",
                "prod_callers_same_tu",
                "test_callers",
                "reference_sites",
                "definition_seen_in_ast",
            ):
                result[field] = str(item.get(field, ""))
            writer.writerow(result)


def main() -> int:
    global ROOT
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--inventory", type=Path, default=HERE / "class_inventory.csv")
    parser.add_argument(
        "--compile-commands",
        type=Path,
        action="append",
        default=[ROOT / "build"],
        help="compile_commands.json or directory; may be repeated",
    )
    parser.add_argument(
        "--source-prefix",
        action="append",
        default=[],
        help="restrict this invocation to a source subtree; repeat for isolated package runs",
    )
    parser.add_argument("--output", type=Path, default=HERE / "reference_graph.csv")
    parser.add_argument("--update-inventory", action="store_true")
    parser.add_argument("--libclang", type=str)
    args = parser.parse_args()
    repo = args.repo.resolve()
    ROOT = repo
    configure_libclang(args.libclang)

    rows, infos, anchors = load_inventory(args.inventory.resolve())
    units = load_compilation_units(repo, args.compile_commands)
    if args.source_prefix:
        prefixes = [(repo / prefix).resolve() for prefix in args.source_prefix]
        units = [
            unit
            for unit in units
            if any(unit.source == prefix or prefix in unit.source.parents for prefix in prefixes)
        ]
    class_usr_to_info: dict[str, SymbolInfo] = {}
    usr_to_info: dict[str, SymbolInfo] = {}
    free_groups: dict[Path, list[SymbolInfo]] = defaultdict(list)
    counts: dict[str, dict[str, object]] = {
        row["id"]: {
            "prod_callers_other_tu": 0,
            "prod_callers_same_tu": 0,
            "test_callers": 0,
            "reference_sites": [],
            "definition_seen_in_ast": False,
        }
        for row in rows
    }
    index = cindex.Index.create()
    parsed = 0
    failed: list[tuple[Path, str]] = []
    diagnostics = 0
    for unit in units:
        try:
            translation_unit = index.parse(
                str(unit.source),
                args=unit.arguments,
                # Macro expansion is not a caller category in R1.  Omitting
                # the detailed preprocessing record materially reduces AST
                # memory while preserving DeclRef/TypeRef/MemberRef/CallExpr.
                options=0,
            )
            parsed += 1
            diagnostics += len(translation_unit.diagnostics)
            build_ast_index(
                translation_unit,
                infos,
                anchors,
                class_usr_to_info,
                usr_to_info,
                free_groups,
            )
            # A source is parsed once, so de-duplication only needs to live for
            # this TU.  Retaining every site across hundreds of ASTs can
            # otherwise consume hundreds of MB without changing the counts.
            seen_sites: set[tuple[str, str, int, int, str]] = set()
            for cursor in walk(translation_unit.cursor):
                if cursor.kind not in REFERENCE_KINDS:
                    continue
                target = resolve_target(cursor, usr_to_info, class_usr_to_info, anchors, free_groups)
                if target is None:
                    continue
                if inside_definition_scope(cursor, target) or inside_member_scope(cursor, target, class_usr_to_info):
                    continue
                origin_path, line, _column, offset = location_tuple(cursor)
                if origin_path is None:
                    continue
                site_kind = str(cursor.kind).split(".")[-1]
                key = (target.identifier, str(origin_path), offset or line, line, site_kind)
                if key in seen_sites:
                    continue
                seen_sites.add(key)
                try:
                    origin_rel = origin_path.relative_to(repo).as_posix()
                except ValueError:
                    origin_rel = str(origin_path)
                site = f"{origin_rel}:{line}:{site_kind}"
                data = counts[target.identifier]
                sites = data["reference_sites"]
                if len(sites) < 40:  # type: ignore[arg-type]
                    sites.append(site)  # type: ignore[union-attr]
                if unit.is_test:
                    data["test_callers"] = int(data["test_callers"]) + 1
                elif origin_path == target.file:
                    data["prod_callers_same_tu"] = int(data["prod_callers_same_tu"]) + 1
                else:
                    data["prod_callers_other_tu"] = int(data["prod_callers_other_tu"]) + 1
            del translation_unit
        except Exception as exc:  # clang can reject one TU while others remain useful
            failed.append((unit.source, f"{type(exc).__name__}: {exc}"))

    for info in infos.values():
        if info.definition_ranges:
            counts[info.identifier]["definition_seen_in_ast"] = True
        sites = counts[info.identifier]["reference_sites"]
        counts[info.identifier]["reference_sites"] = ";".join(sites)  # type: ignore[arg-type]

    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_graph(args.output.resolve(), rows, counts)
    if args.update_inventory:
        update_inventory(args.inventory.resolve(), rows, counts)

    resolved = sum(
        int(item["prod_callers_other_tu"])
        + int(item["prod_callers_same_tu"])
        + int(item["test_callers"])
        for item in counts.values()
    )
    symbols_with_refs = sum(
        int(item["prod_callers_other_tu"])
        + int(item["prod_callers_same_tu"])
        + int(item["test_callers"]) > 0
        for item in counts.values()
    )
    definitions_seen = sum(bool(item["definition_seen_in_ast"]) for item in counts.values())
    print(f"compile_commands={len(units)}")
    print(f"translation_units_parsed={parsed}")
    print(f"translation_units_failed={len(failed)}")
    print(f"clang_diagnostics={diagnostics}")
    print(f"references_resolved_outside_definition_scope={resolved}")
    print(f"symbols_with_resolved_references={symbols_with_refs}")
    print(f"symbols_with_definition_seen_in_ast={definitions_seen}/{len(rows)}")
    print(f"graph_output={args.output.resolve()}")
    if failed:
        print("failed_translation_units:")
        for source, reason in failed[:20]:
            print(f"  {source}: {reason}")
        if len(failed) > 20:
            print(f"  ... {len(failed) - 20} more")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
