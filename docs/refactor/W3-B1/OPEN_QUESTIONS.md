# W3-B1 — OPEN_QUESTIONS

## OQ-01 — gate v3 / dependency direction

- Trạng thái: CLOSED/CONFIRMED trên baseline `origin/main=24ec0fc8718bb8606e4e9e4a7eaa84773d384854`.
- Bằng chứng: `tools/gate.sh static` chạy được; dependency direction PASS với 4 exception baseline được contract cho phép.

## OQ-02 — `color_text`

- Trạng thái: CLOSED/FIXED bởi `50e008f`.
- Bằng chứng: `trajectory.cpp` chép nguyên văn block `namespace color_text` với `static const std::string` từ `src/mapping/rog_map_vendor/include/navigation_math/color_text.hpp`; không thêm dependency `rog_map_vendor` vào target `polynomial`.

## OQ-03 — build/CTest evidence

- Trạng thái: build Release và `make test` CONFIRMED; ROS gate đã chạy nhưng FAIL, không tự quy đổi thành PASS.
- Bằng chứng ROS gate: `1483 tests, 0 errors, 5 failures, 0 skipped`, `GATE_V3_RESULT=FAIL`. 5 failure thuộc attachment-only dependency checks của `px4_ros2_cpp` (cpplint/uncrustify); các test XML của W3-B1 không có suite failure.
- Quy tắc chạy: mọi ROS build/test dùng `flock /tmp/uavnav-build.lock` với `PARALLEL_WORKERS=2 MAKE_JOBS=2`.

## OQ-04 — legacy lineage

- Trạng thái: NOT_APPLICABLE/RESOLVED theo COMMON_CONTRACT_v3 §1; không truy hồi hoặc phục hồi SHA legacy.

## OQ-05 — cleanup list

- Trạng thái: OPEN only for architect instruction.
- Prompt W3-B1 không có path ở mục “Dọn dẹp và đóng”, nên agent không xoá file nào. Không được tự xoá `REPORT.md`, `OPEN_QUESTIONS.md`, hoặc tài liệu ngoài danh sách prompt.
