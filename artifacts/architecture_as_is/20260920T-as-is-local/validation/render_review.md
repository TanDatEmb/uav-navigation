# Kiểm tra render và điều hướng offline

- Graphviz DOT: 7 SVG được tạo bằng Graphviz 2.43.0 và XML-parse trong `verify_views.py`.
- Mỗi SVG được mở dưới dạng PNG preview ở `/tmp/uav-as-is-svg-review` để kiểm tra cắt khung, chiều mũi tên, edge label và cỡ chữ. Các view deployment/ownership rộng, nhưng node/nhãn còn đọc được khi phóng; các view state/transition đã tách theo owner để tránh poster dày. Không thấy clipping hoặc arrow bị che.
- `ownership.svg` thể hiện phân biệt chiều ROS async giữa runtime và mode; lời gọi hold từ mode sang executor là callback/call path. Legend dùng nét và nhãn để phân biệt sync, async, read/write và layout-only edge.
- Các liên kết SVG trỏ tới ID evidence/table tương ứng; checker xác minh target và ID. `index.html`, bảng và evidence index dùng đường dẫn tương đối, không CDN/JS framework.
- Bốn sequence diagram Mermaid có source sinh từ `model/architecture.json`. `mmdc`, `node`, `npm` không có; Mermaid SVG không được render hoặc visual-QA, nên `PARTIAL_AS_IS` được giữ.

Preview là kiểm tra hình ảnh trên artifact render, không phải bằng chứng runtime hoặc semantic completeness. Mở [index.html](../index.html) trong browser để duyệt offline.
