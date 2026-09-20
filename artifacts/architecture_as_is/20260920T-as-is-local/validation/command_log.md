# Lệnh và kết quả kiểm chứng

Các lệnh baseline mặc định chạy từ checkout được chụp `/home/letandat/Dev/uav-navigation`; lệnh archive-only cũng được kiểm tra trong worktree nhánh audit. Exit code ghi đúng từ lần chạy.

| Lệnh | Exit | Kết quả / giới hạn |
|---|---:|---|
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/build_views.py` | 0 | Sinh 30 field rows, 16 transition rows, 68 evidence excerpts, 7 DOT/SVG và 4 Mermaid source; Graphviz rendering completed. |
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_views.py` | 1 (trước hoàn tất index) | Phát hiện duy nhất link `validation/command_log.md` chưa được tạo; file này sau đó được thêm vào output. |
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_views.py` | 0 | Kiểm tra ID/evidence, field writer-reader/write-set, source path/range/hash, transition coverage trong DOT, SVG XML/links, cùng local links trong index và Markdown. |
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_snapshot.py` | 0 trong checkout gốc; 1 trên worktree nhánh mới | Checkout gốc khớp 596/596. Worktree nhánh mới từ HEAD không chứa các dirty source files đã chụp; phép so sánh live thất bại đúng như dự kiến. Dùng archive-only để kiểm tra artifact đã push. |
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_baseline.py` | 0 | Đối chiếu HEAD, branch, porcelain v2, staged/unstaged diff hash và submodule revisions với baseline capture. |
| `python3 artifacts/architecture_as-is/20260920T-as-is-local/tools/verify_snapshot.py --archive-only` | 0 | Xác minh 596/596 hash trong source snapshot được commit; bỏ so sánh với live source tree theo chủ đích. |
| `python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/verify_baseline.py --archive-only` | 0 | Metadata HEAD/branch/diff/status trong archive khớp nội bộ; không so current Git checkout với baseline gốc. |
| `git diff --cached --check` | 2 | Có 380 whitespace/blank-at-EOF diagnostics trong source snapshot được chụp nguyên byte và excerpt số dòng. Không sửa source snapshot vì sẽ phá hash baseline; các diagnostics nằm trong artifact snapshot, không phải production source của nhánh audit. |
| `python3 -c 'from pathlib import Path; import subprocess; root=Path("artifacts/architecture_as_is/20260920T-as-is-local"); out=Path("/tmp/uav-as-is-svg-review-final"); out.mkdir(parents=True,exist_ok=True); [subprocess.run(["dot","-Tpng",str(p),"-o",str(out/(p.stem+".png"))],check=True) for p in sorted((root/"diagrams/src").glob("*.dot"))]'` | 0 | Tạo tạm 7 preview PNG trong `/tmp/uav-as-is-svg-review-final`; đã mở để kiểm tra clipping, mũi tên, nhãn và khả năng đọc. PNG preview không phải artifact bàn giao. |
| `PUPPETEER_SKIP_DOWNLOAD=1 pnpm add --save-exact @mermaid-js/mermaid-cli@11.17.0` trong `/tmp/uav-as-is-mmdc-20260920` | 1 | Package manager chặn Puppeteer lifecycle script trong lần cài đầu; không tải browser. Tiếp tục bằng install lockfile với scripts tắt. |
| `PUPPETEER_SKIP_DOWNLOAD=1 pnpm install --ignore-scripts` trong `/tmp/uav-as-is-mmdc-20260920` | 0 | Cài cô lập từ pnpm lockfile; không cài global hoặc chạy script tải browser. |
| `mmdc --version` với `node_modules/.bin` trong PATH | 0 | `11.17.0`. Node `v24.19.0`, pnpm `v11.19.0`; Chrome hiện có là `153.0.8010.52`. |
| Lần render Mermaid đầu với `build_views.py --mmdc ... --puppeteer-config ...` | 1 | `nominal_flow` render; parser phát hiện lỗi cú pháp trong nội dung `pass_through_handoff`. Không thay source runtime; sửa câu nguồn model rồi render lại. |
| `python3 .../tools/build_views.py --mmdc /tmp/uav-as-is-mmdc-20260920/node_modules/.bin/mmdc --puppeteer-config /tmp/uav-as-is-mmdc-20260920/puppeteer.json` | 0 | Sinh lại bảng/DOT/Mermaid; render đủ 7 Graphviz + 4 Mermaid SVG từ model canonical. |
| Mở bốn Mermaid PNG preview dưới `/tmp/uav-as-is-mermaid-review` | 0 | Kiểm tra bằng mắt tất cả bốn sequence; không phát hiện clipping; ghi nhận Mermaid participant footbox lặp mặc định. |
| `python3 .../tools/verify_views.py` sau Mermaid render | 0 | Unique IDs=159, fields=30, transitions=16, evidence refs=68, rendered SVGs=11 (7 DOT + 4 Mermaid); mọi link/index/source-range/hash trong scope checker resolve. |
| Cài lại từ artifact lockfile trong một thư mục tạm mới: `PUPPETEER_SKIP_DOWNLOAD=1 pnpm install --ignore-scripts && node_modules/.bin/mmdc --version` | 0 | Cài từ local content-addressed store, không tải thêm; lockfile reproduced, CLI báo `11.17.0`. |
| Final render từ `/tmp/uav-as-is-mmdc-repro.4ZNdLY` bằng `build_views.py --mmdc ... --puppeteer-config ...` | 0 | Render lại toàn bộ view sau khi xác minh lockfile; sinh đủ 7 Graphviz + 4 Mermaid SVG. |
| Tạo bốn Mermaid PNG preview trong `/tmp/uav-as-is-mermaid-review-final` bằng `mmdc -e png`; mở cả bốn hình | 0 | Visual review sau final render: labels/arrows đọc được; không phát hiện clipping. |
| `python3 .../tools/verify_views.py` final | 0 | Unique IDs=159, fields=30, transitions=16, evidence refs=68, SVG=11 (7 DOT + 4 Mermaid); mọi link và model reference trong scope checker resolve. |
| `python3 .../tools/verify_snapshot.py --archive-only` và `verify_baseline.py --archive-only` | 0, 0 | 596 source hashes và archived baseline metadata đều pass nội bộ; không so sánh live checkout với snapshot gốc. |
| `git diff --check` trong audit worktree sau cập nhật artifact | 0 | Không có whitespace error trong diff của follow-up; source snapshot vẫn nguyên byte. |

Không chạy build, unit test, launch, SITL hoặc runtime. `validation/test_review.md` chỉ ghi nhận các test/assertion được đọc trong source. Structural checker pass không chứng minh semantic completeness hoặc runtime correctness.
