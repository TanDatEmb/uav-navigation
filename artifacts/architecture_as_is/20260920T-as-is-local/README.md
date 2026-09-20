# Khôi phục kiến trúc hành vi AS-IS

**Trạng thái:** `PARTIAL_AS_IS`. Đây là mô tả tĩnh có bằng chứng của nội dung checkout được chụp ngày 2026-09-20; không phải chứng nhận code đúng, an toàn hay flight-qualified.

## Bắt đầu đọc

1. Mở [index.html](index.html) để duyệt tài liệu, bảng, evidence excerpts và SVG offline.
2. Đọc [AS_IS.md](AS_IS.md) cho hành vi hiện tại.
3. Đọc [REDESIGN_INPUTS.md](REDESIGN_INPUTS.md) cho các điểm cần quyết định ở bước tiếp theo; tài liệu này không đưa ra kiến trúc thay thế.
4. Dùng [model/architecture.json](model/architecture.json) làm model canonical; bảng và DOT được sinh từ model này.

## Baseline

- Repo: `/home/letandat/Dev/uav-navigation`
- HEAD: `9534d8dc15920c8b3e80c8fa12f28ec972b8c6a2`
- Branch: `codex/close-proven-findings`, không detached.
- Baseline là local working tree, gồm dirty source/config hiện có. Lúc chụp có 32 status entries, 0 staged bytes và 1,815,760 unstaged diff bytes. Hash và path-level status ở [baseline/environment.json](baseline/environment.json), [baseline/worktree_status.txt](baseline/worktree_status.txt), [baseline/source_manifest.json](baseline/source_manifest.json).
- Snapshot giữ 596 source/config/build/interface text files theo relative path, SHA-256, kích thước và line count. Hai external submodule payload không được copy; revision của chúng được ghi riêng.
- Build profile đang chạy không xác định. `config/runtime/mapping.yaml` là source default (`deployment_profile: sitl`), không chứng minh profile/config thật của một process.
- Compile database có 57 entry nhưng chỉ thấy code của PX4 ROS2 submodule và ROS Jazzy `gtest_vendor`; thiếu `CMakeCache.txt` tương ứng và không có first-party runtime translation unit. AST/write-set analysis vì vậy chưa làm được.

## Phạm vi và giới hạn

Package inventory ở [tables/package_scope.md](tables/package_scope.md). View chi tiết bao phủ goal handoff, worker, candidate commit/activation, command exposure, MissionController, PX4 adapter, measured recovery và Hold. Field list là 30 field hành vi quan trọng; transition list có 16 record; 12 scenario được xếp status; 68 reference records dẫn tới excerpt có line number từ snapshot.

Không tuyên bố exhaustive writer/reader inventory: alias, assignment toàn object, lambda capture và mọi indirect mutation chưa được AST kiểm tra. `runCycle` lớn; các nhánh hiếm về cancellation/exception cần tiếp tục review. Không có runtime trace, build hoặc test nào chạy trong lượt audit. Test được đọc ở source để dẫn assertion, không được chạy.

## Render và kiểm tra

Chạy từ repository root của checkout được chụp, ở nhánh baseline và khi các thay đổi local lúc chụp còn nguyên:

```sh
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/build_views.py
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_views.py
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_snapshot.py
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_baseline.py
```

`build_views.py` sinh evidence excerpt, các bảng, DOT, Mermaid và render Graphviz SVG. Không có `mmdc`, `node` hoặc `npm` trong môi trường chụp; 4 Mermaid sequence source có sẵn nhưng chưa được render. `verify_views.py` kiểm tra cấu trúc model/artifact, không chứng minh semantics đầy đủ. `verify_snapshot.py` so SHA-256 của toàn bộ file trong manifest với snapshot và checkout hiện tại; nếu báo drift, ngừng dùng ref bị ảnh hưởng như bằng chứng của baseline.

Bản artifact được push lên nhánh audit có thể được mở trên checkout sạch ở HEAD thay vì dirty working tree đã chụp. Khi chỉ kiểm tra tính toàn vẹn của bản archive trên nhánh đó, dùng rõ `--archive-only`:

```sh
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_snapshot.py --archive-only
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_baseline.py --archive-only
```

Các lệnh archive-only xác minh hash snapshot và tính nhất quán nội bộ của metadata đã lưu; chúng không tuyên bố checkout hiện tại bằng baseline gốc.

DOT là view từ `model/architecture.json`; dotted order trong transition view là thứ tự đọc, không khẳng định happens-before. Sequence source cũng sinh từ `sequence_diagrams` trong model. Evidence href mở `evidence/refs.html`, nơi có excerpt, source path, line range và SHA-256.

## Provenance và an toàn

Không chỉnh production source, config, message, test, build manifest hoặc `.gitignore`; không cài tool toàn cục, chạy build/test, truy cập flight stack, publish command hay upload source. Toàn bộ artifact mới nằm trong thư mục run này. Chi tiết generated/dependency provenance và lý do profile/build chưa xác minh xem [baseline/provenance.md](baseline/provenance.md).
