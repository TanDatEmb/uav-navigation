# W3-B1 — Piece/Trajectory polynomial contract

## 1. Tóm tắt

CONFIRMED: W3-B1 chạy trên branch `refactor/W3-B1`, baseline `origin/main=432dc94630fbc76ca670138228f2f616f6840bb0`.
CONFIRMED: `Piece`, `Trajectory`, `RootFinder` và implementation đã chuyển sang `navigation_planning/polynomial`.
CONFIRMED: backend không còn include path cũ; namespace lớp vẫn `geometry_utils`, root finder vẫn `math_utils`.
CONFIRMED: `navigation_planning::polynomial` là static target mới, backend link target này; package.xml không thêm dependency.
CONFIRMED: alias Eigen thuần nằm trong `types.hpp`; `trajectory.hpp` bỏ include `color_msg_utils.hpp` không dùng.
CONFIRMED: syntax compile các TU/header thuần, `git diff --check`, safety ledger và citation check đã PASS.
CONDITIONAL: graph command không báo cycle; dependency guard chưa có trên baseline A2 chưa merge.
NOT_MEASURED: build Release/CTest/gate all do canonical build lock của worktree R2.
Branch `refactor/W3-B1` sẽ được push để kiến trúc sư review; không merge.

## 2. Deliverables và changed paths

- Contract headers: `src/planning/navigation_planning/include/navigation_planning/polynomial/{piece,trajectory,root_finder,types}.hpp`.
- Implementations: `src/planning/navigation_planning/src/polynomial/{piece,trajectory,root_finder}.cpp`.
- CMake: `src/planning/navigation_planning/CMakeLists.txt`,
  `src/planning/navigation_planning_backend/CMakeLists.txt`.
- Backend callers: 25 include sites trong `data_structure`, `planner_core`,
  `planner_runtime_context`, `traj_opt`, `utils` và 2 test translation units.
- `src/planning/navigation_planning_backend/include/utils/header/type_utils.hpp`
  include `types.hpp` là include ngược duy nhất được giữ theo prompt.
- Không chuyển test `test_trajectory.cpp`: test này dùng backend/corridor/planner,
  không phải test thuần Piece/Trajectory/RootFinder.

## 3. Move diff/hunk table

Lệnh: `git diff origin/main..HEAD -M --color-moved=zebra --color-moved-ws=allow-indentation-change --stat`

```text
36 files changed, 106 insertions(+), 52 deletions(-)
Piece header R080; RootFinder header R100; Trajectory header R098
piece.cpp R099; root_finder.cpp R099; trajectory.cpp R098
```

| File/hunk | Nội dung không-moved | Lý do contract |
|---|---|---|
| `piece.hpp:33-50` | include mới và qualifier alias từ `navigation_math` sang `navigation_planning::polynomial_types` | include, namespace |
| `trajectory.hpp:34-37` | include `piece.hpp` mới; bỏ `color_msg_utils.hpp` | include, unused-include |
| `piece.cpp:7`, `trajectory.cpp:7`, `root_finder.cpp:1` | include header public mới | include |
| `trajectory.cpp:9-19` | include `<iostream>` và giữ `RESET/GREEN` bằng compatibility alias cục bộ, byte ANSI không đổi | include, alias |
| `navigation_planning/CMakeLists.txt:28-40` | target `navigation_planning::polynomial`, source/install/export | cmake |
| `navigation_planning_backend/CMakeLists.txt:21-43,60-66` | bỏ source đã move, link target mới | cmake |
| backend include/test callers | đổi include path, không đổi symbol/body | include |
| `type_utils.hpp:5` | thêm include `types.hpp`; giữ include ngược duy nhất | include |

`PLANNER_BACKEND_PORT_SHA256.json` còn key path legacy để giữ provenance manifest;
đây không phải source include hoặc shim và không được sửa trong WP này.

## 4. Dependency/type guard

- CONFIRMED: `navigation_planning/package.xml` vẫn chỉ có dependency cũ; `eigen`
  đã tồn tại, không thêm `rog_map_vendor`, ROS msg hay `rclcpp`.
- CONFIRMED: source/header mới không include `navigation_math/type_utils.hpp`,
  `color_msg_utils.hpp`, `std_msgs`, `rclcpp`, `rog_map_vendor` hoặc
  `navigation_planning_backend`.
- CONFIRMED: `types.hpp` chỉ chứa Eigen/vector aliases cần cho public contract.
- NOT_MEASURED: `python3 tools/check_dependency_direction.py` — file chưa tồn tại
  trên baseline `origin/main`; xem `OPEN_QUESTIONS.md` OQ-01.

## 5. Verification — output thật

| Kiểm tra | Kết quả |
|---|---|
| `g++ -std=c++20 -Isrc/planning/navigation_planning/include -I/usr/include/eigen3 -fsyntax-only` trên 3 TU mới | PASS; cả `piece.cpp`, `root_finder.cpp`, `trajectory.cpp` exit 0 |
| Header compile `piece.hpp`/`trajectory.hpp`/`root_finder.hpp` | PASS; exit 0 |
| Direct syntax compile of `optimization_utils.h` and `kinematic_state_boundary.hpp` with navigation-math closure | PASS after review fix |
| Old include guard trên `src` (loại trừ provenance JSON) | PASS; không còn match |
| Forbidden dependency grep trong package mới | PASS; không có match |
| `git diff --check` | PASS |
| `python3 tools/validate_runtime_safety_ledger.py` | `runtime safety ledger validation: PASS (current=482 lines, gates=34, bypasses=1)` |
| `python3 tools/refactor/check_citations.py . docs/refactor` | `checked=989 out_of_range=0 ambiguous_basenames=[...]` |
| `colcon graph --base-paths src --packages-select navigation_planning navigation_planning_backend navigation_runtime navigation_execution` | exit 0; liệt kê 4 package, không báo cycle |
| `python3 tools/check_dependency_direction.py` | NOT_MEASURED: `No such file or directory` |
| `make build` | NOT_MEASURED: canonical lock pid 17616 thuộc worktree R2 |
| CTest `navigation_planning navigation_planning_backend navigation_runtime navigation_execution` | NOT_MEASURED; chờ build Release |
| `tools/gate.sh all` | NOT_MEASURED; script chưa có trên baseline A2 |
| `git log --first-parent 7e0b850..432dc946 -- src config` | NOT_MEASURED: `fatal: bad revision` |

## 6. Lệch prompt / rủi ro còn lại

- A2 chưa merge trên baseline nên guard/gate v3 chưa thể chạy; không tự copy/sửa A2.
- Build/CTest chưa hoàn tất do lock ngoài worktree; không gọi đây là PASS.
- Compatibility alias màu diagnostic cần xác nhận kiến trúc sư; không đổi threshold,
  deadline, lease, budget, UNKNOWN policy, tolerance hay runtime authority.

## 7. Finding → trạng thái → commit

| Finding | Trạng thái | Commit |
|---|---|---|
| Piece/Trajectory/RootFinder còn nằm trong backend contract | FIXED | `fe39ee4` |
| Backend còn dùng include path cũ | FIXED | `fe39ee4` |
| Piece kéo `rog_map_vendor`/ROS qua type_utils | FIXED trong contract mới; old backend `type_utils.hpp` giữ nguyên closure cho caller cũ | `fe39ee4` |
| `trajectory.hpp` include `color_msg_utils.hpp` không dùng | FIXED | `fe39ee4` |
| Backend include/type closure after removing the old trajectory transitive include | FIXED | `f6ce417` |
| Dependency guard/gate v3 | NOT_FIXED trên baseline hiện tại; chờ A2 merge | — |
| Release build + affected/reverse CTest | NOT_MEASURED do build lock | — |

## 8. Commit

`git log --format='%h %s' origin/main..HEAD` ngay trước remote handoff:

```text
fe39ee4 refactor(planning): move polynomial contract into navigation_planning
efbe539 docs(planning): report W3-B1 move evidence
aca3347 docs(wave3): record B1 remote handoff
f6ce417 fix(planning): restore direct type includes
1360bc6 docs(wave3): record B1 review fix
```

Branch `refactor/W3-B1` được push để review; không merge. A1/A2 dependency và Release/CTest gate vẫn là điều kiện mở.
