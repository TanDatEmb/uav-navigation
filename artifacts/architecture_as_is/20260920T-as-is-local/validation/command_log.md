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

Không chạy build, unit test, launch, SITL hoặc runtime. `validation/test_review.md` chỉ ghi nhận các test/assertion được đọc trong source. Structural checker pass không chứng minh semantic completeness hoặc runtime correctness.
