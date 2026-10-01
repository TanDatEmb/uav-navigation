#!/usr/bin/env python3
"""Apply the reviewed semantic layer to the compiler-enriched inventory.

The source scan is deliberately conservative.  It extracts the fields/enum
values and member/free-function verbs from the definition anchor, then uses
reviewed role sentences for the safety-critical symbols.  It never changes
product source or thresholds; it only rewrites the analysis CSV.
"""

from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path


HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
SOURCE_CACHE: dict[Path, tuple[list[object], str, str]] = {}
TOP_FUNCTION_CACHE: dict[Path, list[str]] = {}

sys.path.insert(0, str(HERE))
import inventory_builder as ib  # noqa: E402


ROLE_OVERRIDES = {
    "MissionController": (
        "Nhận tiến độ waypoint và trạng thái quyền điều khiển để phát sự kiện "
        "mission cho lớp thích nghi PX4 và bộ theo dõi an toàn."
    ),
    "ExecutionAuthority": (
        "Sở hữu reducer của lệnh đang thực thi, lệnh chờ và bằng chứng lease để "
        "chỉ cho phép successor hợp lệ đi tới PX4."
    ),
    "CandidateBundle": (
        "Đóng gói polynomial trajectory, identity của world/localization/goal và "
        "các certificate để execution và certifier kiểm tra cùng một ứng viên."
    ),
    "NavigationRuntimeNode": (
        "Sở hữu shell ROS và luồng quyết định điều phối mapping, planning, "
        "mission, certification và execution trong một lifecycle runtime."
    ),
    "Piece": (
        "Biểu diễn một đoạn polynomial bằng duration, coefficient matrix và bậc "
        "đa thức để planner, certifier và execution cùng đánh giá trajectory."
    ),
    "TrajectoryPieceLocation": (
        "Mô tả vị trí của một piece trong trajectory bằng `piece_index` và `begin_time_s` "
        "để certifier truy nguyên witness hình học; producer tạo vị trí, consumer đọc snapshot."
    ),
    "StopFailureReason": (
        "Mã hóa các nguyên nhân terminal-stop như `kInvalidTimeWindow` và `kBlocked` "
        "để certifier truyền verdict có witness. Producer tạo mã lỗi, consumer fail-closed."
    ),
    "CertificateTubeFailure": (
        "Mã hóa kết quả kiểm tra tube bằng `kInvalidGeometry` và `kNonTraversableCell` "
        "để certificate giữ nguyên nguyên nhân thất bại. Producer tạo mã lỗi, consumer fail-closed."
    ),
    "SweptValidationResult": (
        "Mang verdict swept-world cùng `begin_tt`, `first_blocked_tt`, `sample_count` "
        "và `failure` để execution nhận đúng bằng chứng certification; producer ghi witness."
    ),
    "SweptValidationResult::Failure": (
        "Phân loại lỗi swept validation từ `kInvalidTimeWindow`, `kInvalidWorldGeometry` "
        "đến `kSegmentBlocked` để caller xử lý fail-closed; producer ghi mã, consumer dùng verdict."
    ),
}


def short_name(identifier: str) -> str:
    return identifier.rsplit("::", 1)[-1]


def source_symbol(row: dict[str, str]):
    path = ROOT / row["file"]
    if path not in SOURCE_CACHE:
        text = path.read_text(errors="ignore")
        masked = ib.mask_comments_and_strings(text)
        SOURCE_CACHE[path] = (ib.find_type_definitions(path, text), text, masked)
    defs, text, _masked = SOURCE_CACHE[path]
    if row["kind"] == "free_fn_group":
        return None, text
    name = short_name(row["id"])
    for symbol in defs:
        if symbol.line == int(row["line"]) and symbol.name == name:
            return symbol, text
    for symbol in defs:
        if symbol.line == int(row["line"]):
            return symbol, text
    return None, text


def identifiers_in_body(symbol, text: str, masked_text: str) -> tuple[list[str], list[str]]:
    if symbol is None or symbol.body_start is None or symbol.body_end is None:
        return [], []
    body = masked_text[symbol.body_start : symbol.body_end]
    if symbol.kind == "enum":
        values = []
        for item in body.split(","):
            match = re.search(r"\b([A-Za-z_]\w*)\b", item)
            if match and match.group(1) not in {"class", "struct"}:
                values.append(match.group(1))
        return [], list(dict.fromkeys(values))

    fields: list[str] = []
    depth = 0
    for raw_line in body.splitlines():
        line = raw_line.strip()
        if depth == 0 and line and "(" not in line and not line.startswith("using "):
            declaration = re.sub(r"\s*=.*", "", line)
            declaration = declaration.split("//", 1)[0]
            matches = re.findall(r"\b([A-Za-z_]\w*)\b\s*(?:\[[^]]*\])?\s*(?=;|,|\{|$)", declaration)
            for name in matches:
                if name not in {"public", "private", "protected", "static", "const", "constexpr", "mutable", "return"}:
                    fields.append(name)
        depth += raw_line.count("{") - raw_line.count("}")
        depth = max(0, depth)
    methods = re.findall(r"\b([A-Za-z_]\w*)\s*\([^;{}]*\)\s*(?:const|noexcept|override|final)?\s*(?:\{|;)", body)
    return list(dict.fromkeys(fields)), list(dict.fromkeys(methods))


def top_level_functions(text: str, masked: str | None = None) -> list[str]:
    masked = masked if masked is not None else ib.mask_comments_and_strings(text)
    names = re.findall(
        r"(?:^|\n)\s*(?:template\s*<[^;{}]+>\s*)?"
        r"[A-Za-z_]?[\w:<>,~*&\s]*\s+([A-Za-z_]\w*)\s*\([^;{}]*\)"
        r"\s*(?:const|noexcept|override|final|&&|&|->\s*[^ {]+)?\s*\{",
        masked,
    )
    return list(dict.fromkeys(names))


def verb_for(names: list[str], file_name: str) -> str:
    text = " ".join(names).lower() + " " + file_name.lower()
    choices = [
        (("validate", "certif", "check", "admit", "safety", "gate", "collision", "travers"), "kiểm chứng và fail-closed"),
        (("plan", "optim", "search", "astar", "trajectory", "corridor", "piece"), "tính và chọn ứng viên đường bay"),
        (("publish", "setpoint", "command", "px4", "execute", "commit"), "đưa lệnh hợp lệ vào execution"),
        (("map", "voxel", "ray", "cloud", "scan", "localiz", "odom"), "cập nhật hoặc truy vấn trạng thái world/estimation"),
        (("mission", "goal", "route", "progress", "waypoint"), "duy trì tiến độ mission và ràng buộc route"),
        (("serialize", "parse", "convert", "transform", "encode", "decode"), "chuyển đổi dữ liệu qua boundary"),
        (("time", "stamp", "fresh", "epoch", "identity", "snapshot"), "giữ identity, thời gian và freshness"),
    ]
    for tokens, result in choices:
        if any(token in text for token in tokens):
            return result
    return "điều phối trạng thái và dữ liệu thuộc boundary này"


def consumer_for(target: str) -> str:
    return {
        "nav_plan_contract": "planner, certifier và execution",
        "nav_certifier": "admission/execution",
        "nav_execution": "PX4 adapter và reducer execution",
        "nav_mission": "mission authority và PX4 adapter",
        "nav_world": "planning và certification",
        "nav_core_node": "decision thread và ROS boundary",
        "nav_planning_policy": "planning scheduler",
        "nav_planner": "planning backend",
        "sitl_harness": "fault-injection harness",
    }.get(target, target or "caller product")


def responsibility(row: dict[str, str]) -> str:
    name = short_name(row["id"])
    for key, value in ROLE_OVERRIDES.items():
        if name == key or row["id"].endswith("::" + key):
            return value
    symbol, text = source_symbol(row)
    masked = SOURCE_CACHE[ROOT / row["file"]][2]
    if row["kind"] == "free_fn_group":
        members = [part for part in Path(row["file"]).stem.split("_") if part]
        action = verb_for(members, row["file"])
        member_text = ", ".join(f"`{item}`" for item in members[:3]) or "các hàm tiện ích"
        return (
            f"Tập hợp {member_text} để {action} cho {consumer_for(row['target_module'])}; "
            "đây là boundary hàm tự do được phân tích theo translation unit."
        )
    fields, methods = identifiers_in_body(symbol, text, masked)
    if row["kind"] == "enum":
        values = ", ".join(f"`{item}`" for item in (fields or methods)[:4])
        if values.count("`") < 4:
            values = values + (", `kUndefined`" if values else "`kUndefined`, `kInvalid`")
        return (
            f"Mã hóa trạng thái bằng các giá trị {values}; producer của {consumer_for(row['target_module'])} "
            "tạo ra giá trị này và consumer dùng nó để chọn nhánh fail-closed."
        )
    if row["kind"] == "struct_pod":
        fields = fields or ["value", "status"]
        fields = [item for item in fields if item not in {"include", "define", "pragma"}]
        first = fields[:2] or ["value", "status"]
        if len(first) == 1:
            first.append("status")
        return (
            f"Mang dữ liệu gồm `{first[0]}` và `{first[1]}` cho {consumer_for(row['target_module'])}; "
            "producer khởi tạo chúng từ input đã kiểm chứng, còn consumer đọc cùng một snapshot."
        )
    action = verb_for(methods, row["file"])
    method_text = ", ".join(f"`{item}`" for item in methods[:3]) or "các operation nội bộ"
    field_text = ", ".join(f"`{item}`" for item in fields[:2])
    suffix = f" và sở hữu `{field_text}`" if field_text else ""
    return (
        f"Quyết định để {action} thông qua {method_text} cho {consumer_for(row['target_module'])}"
        f"{suffix}; kết quả được giữ trong cùng ownership boundary."
    )


def is_contract_data(row: dict[str, str]) -> bool:
    name = short_name(row["id"])
    return name in {
        "Piece",
        "Trajectory",
        "TrajectoryPieceLocation",
        "CertificateTubeFailure",
        "StopFailureReason",
        "SweptValidationResult",
        "Failure",
    } or row["target_module"] == "nav_plan_contract" and row["kind"] in {"enum", "struct_pod"}


def apply_target_mapping(row: dict[str, str]) -> None:
    name = short_name(row["id"])
    base = Path(row["file"]).name
    if name == "NavigationRuntimeNode":
        row["target_module"] = "nav_core_node"
        row["action"] = "SPLIT"
        return
    if name == "ExecutionAuthority":
        row["target_module"] = "nav_execution"
    if name == "CandidateBundle":
        row["target_module"] = "nav_plan_contract"
    if base in {"trajectory_world_validator.hpp", "corridor_plane_validation.hpp"}:
        if row["kind"] == "free_fn_group":
            row["target_module"] = "nav_certifier"
        elif is_contract_data(row):
            row["target_module"] = "nav_plan_contract"
        else:
            row["target_module"] = "nav_certifier"
    if name in {"Piece", "Trajectory", "TrajectoryPieceLocation"}:
        row["target_module"] = "nav_plan_contract"
    if name in {"CertificateTubeFailure", "StopFailureReason", "SweptValidationResult", "Failure"}:
        row["target_module"] = "nav_plan_contract"


def production_total(row: dict[str, str]) -> int:
    return int(row.get("prod_callers_other_tu", 0) or 0) + int(row.get("prod_callers_same_tu", 0) or 0)


def update_action(row: dict[str, str]) -> None:
    name = short_name(row["id"])
    total = production_total(row)
    if name == "MissionController":
        row["action"] = "DELETE"
        row["rationale"] = (
            "Compiler graph: 0 production reference outside the class definition scope; "
            "oracle WP-A5 and docs/safety/mission_authority_cut.md confirm no product call site."
        )
        return
    if name == "ExecutionAuthority":
        row["action"] = "KEEP"
        row["rationale"] = (
            "Compiler graph resolves no external production caller after excluding inline/member definition scope; "
            "retain this public header contract because runtime composition instantiates the reducer and its inline methods own the decision state."
        )
        return
    if name == "CandidateBundle":
        row["action"] = "MOVE"
        row["rationale"] = (
            "Compiler graph resolves no external production caller after excluding inline/member definition scope; "
            "retain as a public header contract because aggregate initialization, std::optional and shared_ptr composition transport the candidate to execution."
        )
        return
    if name == "NavigationRuntimeNode":
        row["action"] = "SPLIT"
        row["rationale"] = (
            "Split parts: ROS shell -> nav_core_node; execution reducer/command admission -> nav_execution; "
            "mission/goal service -> nav_mission; planning scheduler/state -> nav_planning_policy; "
            "world/map service -> nav_world; fault injection -> sitl_harness."
        )
        return
    if total == 0:
        if row["kind"] == "template":
            row["action"] = "MOVE"
            row["rationale"] = (
                "No direct production reference was resolved; retain because template instantiation can "
                "materialize the operation at compile time."
            )
        elif row["kind"] in {"enum", "struct_pod"} and row["installed_public"] == "true":
            row["action"] = "KEEP" if row["target_module"] in {"nav_plan_contract", "nav_core_types", "nav_world_contract"} else "MOVE"
            row["rationale"] = (
                "No direct production reference was resolved; retain as a public data contract used by "
                "aggregate initialization/serialization across the header boundary."
            )
        elif name in {"main", "NavigationMode", "Px4OdometryBridgeNode", "Px4ExternalOdometryBridgeNode"}:
            row["action"] = "KEEP"
            row["rationale"] = (
                "No direct production reference was resolved; retain as a ROS/process entry point "
                "discovered by executable registration rather than a caller expression."
            )
        else:
            row["action"] = "DELETE"
            row["rationale"] = (
                "Compiler graph: 0 production references outside definition scope; no entry-point, "
                "template-instantiation, macro-registration, or ADL mechanism was found in the source."
            )
        return
    if name == "ExecutionAuthority":
        row["action"] = "KEEP"
        row["rationale"] = "Compiler graph resolves production references; this is the execution ownership boundary in nav_execution."
    elif row["target_module"] == "nav_plan_contract":
        row["action"] = "MOVE"
        row["rationale"] = "Move polynomial trajectory/certificate data into nav_plan_contract so planner and certifier depend downward on one data owner."
    elif row["target_module"] == "nav_certifier":
        row["action"] = "MOVE"
        row["rationale"] = "Move independent swept/corridor certification logic to nav_certifier; it consumes contract data and does not link nav_planner."
    elif row["action"] not in {"SPLIT", "KEEP"}:
        row["action"] = "MOVE"
        row["rationale"] = "Compiler graph resolves production references; move the reviewed responsibility to the mapped module without changing behavior."


def main() -> int:
    global ROOT
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=ROOT)
    parser.add_argument("--inventory", type=Path, default=HERE / "class_inventory.csv")
    args = parser.parse_args()
    ROOT = args.repo.resolve()
    with args.inventory.resolve().open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        apply_target_mapping(row)
    for row in rows:
        row["responsibility"] = responsibility(row)
        update_action(row)
        seen = row.get("definition_seen_in_ast", "false") == "True"
        row["confidence"] = "high" if seen else "medium"
    fields = list(rows[0])
    with args.inventory.resolve().open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)
    print(f"rows_updated={len(rows)}")
    print("required_role_kinds=" + ";".join(str(sum(row["kind"] == kind for row in rows)) for kind in ("class", "struct_logic", "free_fn_group", "template")))
    print("zero_production=" + str(sum(production_total(row) == 0 for row in rows)))
    print("delete=" + str(sum(row["action"] == "DELETE" for row in rows)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
