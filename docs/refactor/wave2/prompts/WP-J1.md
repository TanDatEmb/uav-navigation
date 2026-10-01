# WP-J1: Sửa tính đúng đắn của judge SITL (chỉ Python)

> **Ghi chú baseline (2026-10-01):** các đường dẫn `artifacts/`, `runtime_evidence/`, `.artifacts/`, `docs/reports/`, `docs/validation/`, `docs/benchmarks/`, safety index/archive và CI workflow được nhắc trong tài liệu này đã được gỡ khỏi repo. Coi chúng là nguồn gốc lịch sử, không còn kiểm chứng lại được. Xem mục "Ghi chú baseline" trong `docs/refactor/README.md`.

**Loại:** mỗi hạng mục là **một commit riêng**. Commit loại "judge-behavior" có thể làm đổi verdict; commit loại "refactor" thì không được đổi verdict.
**Phụ thuộc:** baseline v2. Không cần ROS để build; cần Python 3.12, theo đúng phiên bản CI dùng.
**Vì sao làm trước P2:** mọi evidence của các phase sau đều đi qua judge. Judge sai thì mọi kết luận PASS hay FAIL đều vô nghĩa.
**Finding:** R7-04, R7-10, R7-11, R7-12, R7-17, R7-21, R7-23, R7-24, R7-25, R7-26, R7-27, R7-32, R7-35, R7-37, R7-39, R7-41, R-03, R-07. Chi tiết ở `docs/refactor/kb/data/findings.csv`; ngữ cảnh ở `areas/R7_layer.md`.

## Hạ tầng bắt buộc làm đầu tiên (commit J1.0, refactor)
Viết `tools/runtime/rejudge_all.py`:
- Với mỗi session đã lưu có `report.json` trong `runtime_evidence/**` và `artifacts/**`: copy sang thư mục tạm, chạy `report.build(...)` với đúng workflow và config của session (lấy từ `metadata.json` / `runtime.json`), rồi xuất `verdict`, `reasons` và các trục evaluation.
- Output: `docs/refactor/WP-J1/verdicts_<commit>.csv`.
- **Không ghi gì vào repo evidence.**
- Chạy trên baseline, lưu thành `verdicts_baseline.csv`.

Sau **mỗi** commit judge-behavior phải chạy lại script này và ghi bảng diff `session → verdict/reasons trước → sau → finding ID giải thích`. Nếu có một thay đổi verdict không giải thích được bằng finding của commit đó thì DỪNG.

## Hạng mục
| Commit | Loại | Finding | Việc | Test RED bắt buộc |
|---|---|---|---|---|
| J1.1 | judge-behavior | R7-25 | Twist của GT và của LIO là **body frame** (xem `report.py:2324-2327`). Quay sang world bằng `q_xyzw` của cùng sample **trước** khi áp transform lio←gazebo (`evaluation.py:1918-1929`, `2441-2453`; `html_report.py` tương ứng). Sai phân gia tốc phải tính trên vận tốc world | fixture yaw = 90°, v_body = (5,0,0) → sai số vận tốc bằng 0 khi command world = (0,5,0) |
| J1.2 | judge-behavior | R7-26 | ATE/RPE dùng frame witness T_L_G giống `evaluate_tracking` | fixture có offset yaw giữa lio và world |
| J1.3 | judge-behavior | R7-27 | Thống kê sai số dùng đúng `evaluation_window` **nếu** tài liệu policy nói vậy. Nếu policy không nói rõ thì DỪNG, ghi OPEN_QUESTIONS, không sửa | theo quyết định |
| J1.4 | judge-behavior | R7-23 | Gộp 10 hàm percentile thành **một** hàm trong `tools/runtime/stats.py`. Giữ định nghĩa mà gate acceptance hiện đang dùng, ghi rõ định nghĩa đó trong docstring. Các chỗ khác chuyển sang dùng hàm này. Verdict chỉ được đổi ở những chỗ trước đây dùng định nghĩa kia | dữ liệu có p95 lệch (repro `kb/data/repro/R7/pct_repro.py`) |
| J1.5 | judge-behavior | R-03, R7-32 | Collision truth: box SDF áp dụng rpy (OBB). Clearance giữa hai sample GT tính trên đoạn thẳng nối hai sample (swept segment đến OBB/cylinder), không chỉ tại điểm. Không đổi `vehicle_radius` hay các margin | OBB xoay 45° mà AABB báo không chạm; hai sample kẹp một cột |
| J1.6 | judge-behavior | R-07 | **Một** parser acceptance waypoint duy nhất, dùng chung cho `external_mode_scenario.py:2905`, `evaluation.py:3069,3277`, `report.py:751`. Thiếu field thì kết quả là `NOT_EVALUABLE`, không bao giờ là True (fail-closed) | 4 dạng input: thiếu field, `False`, `0`, `True` |
| J1.7 | judge-behavior | R7-39 | Stale/gap của monitor tính theo **stamp nguồn (sim time)** so với `/clock`, không theo wall arrival. Giữ cả số đo wall làm diagnostic riêng | sim chạy 0.5× real-time mà không có stale thật |
| J1.8 | judge-behavior | R7-41, R7-37 | `REPORT.html` chỉ **hiển thị** verdict và lý do từ `report.json`; bỏ overall tự tính. Không đổi logic trong `report.json` | HTML khớp JSON trên toàn bộ session của rejudge |
| J1.9 | judge-behavior | R7-17 | Mặc định waypoint behavior trong runner mirror đúng quy tắc của loader C++ (`navigation_mission/src/mission.cpp:110-114`: thiếu `behavior` thì dùng `stop` khi `hold_s > 0` hoặc là waypoint cuối, còn lại dùng `pass_through`). Thêm contract test đối chiếu với fixture của test C++ | mission không khai behavior |
| J1.10 | refactor | R7-10, R7-11, R7-12 | Guard: bỏ `assert` (dùng kiểm tường minh + exit code); quét cả `config/runtime/**`; ban publish `NavigationCommand` dùng `ast` thay regex | chạy guard với `python -O`; code vi phạm viết theo định dạng khác |
| J1.11 | refactor | R7-24 | `_bracket` dùng bisect trên stream đã sort một lần. **Output phải giống hệt từng byte** trên toàn bộ session của rejudge | benchmark trước/sau ghi vào REPORT |
| J1.12 | refactor | R7-21, R7-35 | Ghi `runtime.json` theo kiểu atomic (tmp + rename); signal handler chỉ set cờ, `finish()` chạy ở vòng chính | test mô phỏng SIGTERM giữa lúc ghi |

## Không làm (chờ chủ dự án, ADR-017 §6.3)
- **Q-TRK:** giữ nguyên runner default `relaxed`. Thêm cảnh báo rõ trong `metadata.json` khi mode khác `off`.
- **Q-XTRK:** cross-track **không** thành gate. Chỉ báo cáo con số.
- Không đổi bất kỳ ngưỡng nào trong `common.yaml`, `map_profiles.yaml` hay scenario.

## Ledger
Mỗi commit judge-behavior ghi một entry vào `docs/safety/runtime_safety_current.md`: Authority QUALIFICATION, Evidence UNIT_VERIFIED + rejudge diff. Nếu verdict của một run lịch sử đổi từ PASS sang FAIL: liệt kê run đó trong mục "Active qualification and safety debt".

## Nghiệm thu
- Gate v2: static và python PASS trên Python 3.12. Ghi thêm kết quả trên 3.11 để tham khảo.
- REPORT có `verdicts_baseline.csv` và bảng diff sau từng commit. Mọi thay đổi đều được giải thích bằng finding ID.
- Commit refactor (J1.10–J1.12) có diff verdict **rỗng**.

(Áp dụng `COMMON_CONTRACT_v2.md`.)
