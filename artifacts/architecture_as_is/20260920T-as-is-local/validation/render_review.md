# Kiểm tra render và điều hướng offline

- Graphviz DOT: 7 SVG được tạo bằng Graphviz 2.43.0 và XML-parse trong `verify_views.py`.
- Mermaid sequence: 4 SVG được tạo bằng `@mermaid-js/mermaid-cli` 11.17.0 với Chrome 153.0.8010.52 có sẵn trong môi trường. Preview PNG tạm nằm ở `/tmp/uav-as-is-mermaid-review-final`; từng hình đã được mở để kiểm tra cắt khung, nhãn, thứ tự mũi tên và khả năng đọc. Các participant footbox lặp ở cuối là mặc định của Mermaid; nội dung vẫn đọc được, không phát hiện clipping.
- Graphviz preview PNG tạm đã được mở để kiểm tra cắt khung, chiều mũi tên, edge label và cỡ chữ. Các view deployment/ownership rộng, nhưng node/nhãn còn đọc được khi phóng; các view state/transition đã tách theo owner để tránh poster dày. Không thấy clipping hoặc arrow bị che.
- `ownership.svg` thể hiện phân biệt chiều ROS async giữa runtime và mode; lời gọi hold từ mode sang executor là callback/call path. Legend dùng nét và nhãn để phân biệt sync, async, read/write và layout-only edge.
- Các liên kết Graphviz SVG trỏ tới ID evidence/table tương ứng; checker xác minh target và ID. `index.html` nhúng bốn sequence SVG, đồng thời liên kết source Mermaid và scenario table. Các trang dùng đường dẫn tương đối, không CDN/JS framework.
- Bốn Mermaid source và SVG đều được sinh từ `model/architecture.json`; nội dung model đã được chỉnh câu có dấu `;` gây lỗi parse, không thay đổi runtime source.

Preview là kiểm tra hình ảnh trên artifact render, không phải bằng chứng runtime hoặc semantic completeness. Mở [index.html](../index.html) trong browser để duyệt offline.
