# Kiểm soát tiến hóa kiến trúc (CHANGE_CONTROL)

Quyết định: D34 (2026-10-13). File này là **quy tắc làm việc** cho thay đổi kiến trúc. Nó không thay thế gì:
thiết kế vẫn ở [SYSTEM_DESIGN.md](architecture/SYSTEM_DESIGN.md), lý do ở [DECISIONS.md](architecture/DECISIONS.md), tiến độ và lệch ở
[TRACEABILITY.md](TRACEABILITY.md), quy tắc cấu trúc code ở [AGENTS.md](../AGENTS.md).
Đường dẫn, ngưỡng và quy tắc phụ thuộc mà script dùng nằm ở
[arch_rules.yaml](../tools/uavnav/arch_rules.yaml); mỗi dữ kiện chỉ có một nơi.

> Kiến trúc định hướng. Code kiểm chứng và phản biện bằng bằng chứng. Kiến trúc được phép tiến hóa,
> không được suy thoái âm thầm. Kiểm chứng quyết định mức hoàn thành thực tế.

## 1. Nguồn chuẩn

| Loại quyết định hoặc dữ kiện | Nguồn chuẩn duy nhất |
|---|---|
| Thiết kế đích (thành phần, state machine, contract, thời gian) | [SYSTEM_DESIGN.md](architecture/SYSTEM_DESIGN.md) |
| Lý do, lịch sử, câu hỏi mở, sự kiện đã kiểm chứng (D\*, O\*, F\*), Architecture Review (AR\*) | [DECISIONS.md](architecture/DECISIONS.md) |
| Hiện trạng: mức trạng thái, bằng chứng, lệch đang mở (DV\*) | [TRACEABILITY.md](TRACEABILITY.md) |
| Plan triển khai theo lát | `docs/plans/*.md` (kế hoạch, không phải nguồn của thiết kế) |
| Quy tắc cấu trúc code và cách làm việc | [AGENTS.md](../AGENTS.md) |
| Quy tắc kiểm soát thay đổi (file này) và phần máy đọc được | file này, [arch_rules.yaml](../tools/uavnav/arch_rules.yaml) |
| Kiến trúc **như đã xây** (sơ đồ và mô tả khớp code đang chạy, do người làm vẽ sau khi phần đó đã kiểm thử ổn định) | [AS_BUILT.md](architecture/AS_BUILT.md) |
| Bằng chứng chạy (SITL) | `log/sitl/<run>/` (không vào Git); tham chiếu bằng `run:<id>` cộng commit |

Plan không được mâu thuẫn spec. Khi plan và spec khác nhau, spec thắng và plan phải sửa.

## 2. Thang trạng thái (P2)

`DESIGNED` → `IMPLEMENTED` → `COMPONENT_VERIFIED` → `INTEGRATED` → `QUALIFIED`, cộng `DEVIATED` và `BLOCKED`.
Định nghĩa và bằng chứng tối thiểu của từng mức nằm ở đầu [TRACEABILITY.md](TRACEABILITY.md);
`tools/uavnav/arch_check.py` kiểm bằng máy rằng bằng chứng được nêu có thật.

- Mức ghi trên một dòng là mức cao nhất mà **mọi phần đã bắt đầu** của dòng đạt được.
- Một test mới không tự nâng mức. `INTEGRATED` cần một lần chạy tích hợp có `run:<id>`. `QUALIFIED` cần thêm
  `accept:<id>` trỏ tới quyết định owner đã AGREED hoặc một AR đã APPROVED.
- Hạ mức khi phát hiện báo cáo quá tay là việc đúng (xem tín hiệu S8): ghi lý do trong commit.

## 3. Mức thay đổi (Architecture Evolution Gate)

| Mức | Là gì | Yêu cầu |
|---|---|---|
| **L0** | Sửa lỗi, test, tối ưu nội bộ, observability có chủ sở hữu; không đổi contract công khai | Review thường. Không cần ADR. Ghi `Docs-Impact` nếu có |
| **L1** | Đổi interface, hành vi quan sát được hoặc điều kiện tương thích; invariant vẫn đúng | Spec và [TRACEABILITY.md](TRACEABILITY.md) cập nhật **trong cùng thay đổi** hoặc ghi `pending:` có lý do; test tương thích cho consumer; kế hoạch migration nếu cần |
| **L2** | Dấu hiệu tích luỹ độ phức tạp, hoặc thêm một **trạng thái** vào state machine (§2.2 của spec) | Architecture Review (AR\*, §5), kết quả `LIMITED_REFACTOR` hoặc `APPROVED` |
| **L3** | Đổi invariant, ranh giới trách nhiệm, safety contract, hoặc kiến trúc liên hệ con | AR có kết quả `APPROVED` **và** một quyết định D\* AGREED trước khi triển khai phần lớn; phân tích tác động, so sánh phương án, kế hoạch migration, xác định lại yêu cầu kiểm chứng |

Một tín hiệu stress **kích hoạt review, không tự động kích hoạt redesign**.

**Khai báo bằng trailer của commit** (không cần công cụ ngoài; PR lặp lại cùng hai dòng):

```text
Arch-Level: 0|1|2|3
Docs-Impact: none | docs/architecture/SYSTEM_DESIGN.md, docs/TRACEABILITY.md | pending:<lý do>   # có thể ghép: <đường dẫn>, pending:<còn nợ gì>
Review: AR<n>        # từ L2
Decision: D<n>       # L3
```

`tools/uavnav/check_change.py` chỉ **bắt buộc** khai báo khi commit chạm đường dẫn hợp đồng (message, srv, `config/`,
header công khai), file mang invariant, hoặc thêm một enumerator vào enum trạng thái. Commit khác không cần trailer
(coi như L0). Máy không đánh giá được mức khai báo có đúng không: đó là việc của reviewer.

Dừng và hỏi owner (AGENTS.md §1) vẫn áp dụng như trước: giao diện mới hay đổi, đổi chủ sở hữu quyết định, trạng thái
hay chuyển trạng thái mới, thread hay khoá mới, đổi timing hay budget, đổi hành vi an toàn.

## 4. Change envelope

### A. Invariant (chặn cho tới khi có AR được duyệt)

Rút ra từ spec và từ các lỗi đã kiểm chứng (F\*), không áp đặt thêm.

| ID | Invariant | Nguồn | Được ép bởi |
|---|---|---|---|
| SYS-1 | Fail-closed: thiếu hoặc mơ hồ về thời gian, frame, identity, epoch, world, certificate thì từ chối | §0, AGENTS §3 | test từng quyết định; review |
| SYS-2 | Mọi bàn giao cho PX4 và mọi huỷ đều có `Reason` trong event log; không bypass ẩn | §0, D11 | test; review |
| SYS-3 | Một quyết định, một event có component, trạng thái trước và sau, reason, giá trị | §6.1, AGENTS §2.3 | test; `arch_check` cấm in thẳng ra stdout |
| SYS-4 | Không publish và không gọi callback khi giữ khoá; mọi worker có đường fail-closed | §6.4, AGENTS §2.6 | review; TSan (`make sanitize`) chỉ bắt được data race trên các test của lớp có thread (`LioRunner`, ingest), không chứng minh quy tắc này |
| SYS-5 | Thư viện quyết định thuần, không phụ thuộc ROS; ROS chỉ ở lớp vỏ | §7.1 | `arch_check`: include và `package.xml` của các file ngoài danh sách `ros_allowed` trong `arch_rules.yaml` (danh sách này cho phép cả header của lớp vỏ như `lio_node/*`, nên trong đó chỉ review mới giữ được ranh giới) |
| SYS-6 | Mỗi giá trị cấu hình một nguồn; ngoài phạm vi thì từ chối khởi động | §6.2, AGENTS §2.4 | `test_config`, test config từng package |
| TIME-1 | Bốn miền thời gian là bốn kiểu; stamp ngoài được kiểm phạm vi; int64 ns | §6.3, D28 | `test_time`, `Px4Time.*`, `to_msg` tests |
| LIO-1 | `LioLifecycle` là nơi ghi duy nhất của trạng thái LIO | §3.5, D30 | `LioLifecycleTable.*`; file mang invariant: `lifecycle.*`, `frontend.cpp` |
| LIO-2 | Backend không với tới lifecycle và không publish | §3.5, D32 | compile-time (`#error` ở cuối `.cpp` của backend, macro đánh dấu của `lifecycle.hpp`) |
| LIO-3 | Hai thread, mỗi khối một thread sở hữu; hàng đợi giới hạn, dữ liệu bất biến, tràn thì có event và không chặn ingest | §3.5, D31 | `LioRunner.*`, TSan |
| LIO-4 | Mọi lối vào TRACKING qua xác nhận; không có cạnh LOST → TRACKING; chỉ correction được predictor áp dụng và không quá tuổi mới tính là scan có correction | §3.1, F27, D31, D33 | `LioLifecycleTable.NoEdgeFromLostToTracking`, `LioFrontend.Result*` |
| PX4-1 | `T` đo tại vị trí xe; mọi cạnh đổi trạng thái của `T` nằm trong bảng cạnh của spec; đầu vào xấu thì từ chối | §4.1, D30 | `Alignment.TransitionTableIsTheSpecDiagram`, `AlignmentFuzz.*` |
| PX4-2 | EV chỉ từ odometry TRACKING, nhãn FRD, `reset_counter` theo epoch của mẫu; reset của PX4 áp tất định | §4.1, F13, F14, F34 | `EvEncoder.*`, `Alignment.*Reset*` |
| SUP-1 | (kế hoạch, S2) Supervisor là nơi ghi duy nhất của điều khiển bay; thêm trạng thái phải qua review thiết kế | §2.2 | review; `check_change` bắt enumerator mới |

File mang từng invariant liệt kê trong `invariant_paths` của [arch_rules.yaml](../tools/uavnav/arch_rules.yaml);
khi một commit chạm chúng, `check_change.py` in tên invariant để reviewer nhìn vào.

### B. Tiến hóa được phép (không cần review toàn diện nếu không có tín hiệu bất thường)

Sửa lỗi; thêm test; đổi thuật toán nội bộ mà không đổi contract; tối ưu trong giới hạn đã ghi; thêm
telemetry hoặc observability có chủ sở hữu; mở rộng interface theo cách tương thích đã thiết kế; hoàn thiện
trường hợp biên mà kiến trúc đã cho phép. Hằng số tầng (a) vẫn theo AGENTS và §6.2 của spec (review thiết kế).

### C. Chỉ báo stress

Ngưỡng ban đầu nằm ở `stress:` trong [arch_rules.yaml](../tools/uavnav/arch_rules.yaml), hiệu chỉnh trên lịch sử thật của nhánh
này (xem §5). Chúng là ngưỡng **cảnh báo**, không phải luật. Sai lệch ngưỡng được sửa ở đó, kèm lý do trong commit.

| # | Chỉ báo | Phát hiện | Máy |
|---|---|---|---|
| S1 | Bùng nổ cờ và trạng thái | `bool` mới trong header đã có, enumerator mới của enum trạng thái | `arch_health.py` |
| S2 | Tích luỹ workaround và đường đặc biệt | dòng thêm có `workaround`/`hack`/`temporary`/`for now`/`FIXME` | `arch_health.py` |
| S3 | Sửa cấu trúc lặp lại | một file còn trong cây, bị ≥ 5 commit `fix`/`refactor` sửa | `arch_health.py` |
| S4 | Contract không ổn định, sửa nhiều consumer vì một producer | file message/config bị sửa ≥ 3 lần; một thay đổi contract kéo ≥ 3 package khác | `arch_health.py` |
| S5 | Lệch tạm thời quá hạn, hoặc nhiều lệch mở | `DV*` quá ngày xem lại; ≥ 3 lệch mở | `arch_check.py`, `arch_health.py` |
| S6 | Coupling quá mức | hai package cùng thay đổi trong ≥ 6 commit | `arch_health.py` |
| S7 | Tài liệu trôi khỏi bằng chứng | `arch_check.py` báo lỗi TRACEABILITY | `arch_health.py` |
| S8 | Việc đã xong bị mở lại | dòng TRACEABILITY bị hạ mức | `arch_health.py` |
| R1 | Rò trách nhiệm giữa hệ con | vi phạm quy tắc phụ thuộc: máy bắt được một phần; còn lại là checklist review | `arch_check.py`, review |
| R2 | Logic lặp giữa các hệ con | không phát hiện được bằng máy một cách tin cậy | review (mục trong PR template) |
| R3 | Tổ hợp trạng thái không hợp lệ tăng | đọc bảng cạnh và test fuzz của từng state machine | review; S1 làm tín hiệu |

Khi một hệ con vượt ngưỡng chỉ báo: **tạm ngừng mở rộng workaround liên quan**, làm Architecture Review (§5), rồi tiếp tục.
Việc độc lập không bị ảnh hưởng thì **không dừng**. Điều này làm rõ quy tắc dừng cũ ở §7.4 của spec (xem D34).

## 5. Architecture Review

**Khi nào.** Một chỉ báo vượt ngưỡng; một commit cần L2 hoặc L3; nhiều lệch chung nguyên nhân; hoặc reviewer yêu cầu.

**Làm gì.**

1. Đọc diff của các commit trong tín hiệu, không chỉ message hay số lần sửa. Phân loại từng nhóm thay đổi:
   *tiến hóa bình thường*, *rework dự kiến*, *lỗi lặp cùng nguyên nhân gốc*, *tài liệu trôi*, *nghi xói mòn kiến trúc*,
   *vi phạm đã xác nhận*.
2. Với state model: mỗi cờ hay trạng thái có phải một chiều độc lập? Có suy ra được từ dữ liệu khác không? Có nhiều cờ
   cùng biểu diễn một sự kiện, tổ hợp mâu thuẫn, chuyển trạng thái rải rác không? Ai sở hữu, ai được chuyển? Số cạnh và
   độ phức tạp test tăng bao nhiêu?
3. So sánh phương án: giữ nguyên và thêm invariant/test; bỏ trạng thái dư bằng trạng thái dẫn xuất; chia lại sở hữu;
   FSM; FSM phân cấp; tách chiều trực giao; thiết kế lại. FSM không mặc định tốt hơn `bool`; nhiều cờ không mặc định là sai.
4. Ghi một dòng AR\* vào bảng ở [DECISIONS.md](architecture/DECISIONS.md) §5: hệ con, tín hiệu, phân tích, `Window end` (commit
   cuối của lịch sử đã xét), `Owner xác nhận` (`có`/`chưa`), kết quả. Một AR do phiên thực thi lập luôn ghi `chưa` cho tới khi owner xác nhận; `check_change.py` chặn Arch-Level 3 nếu AR chưa được owner xác nhận, và `arch_health.py` ghi rõ AR nào chưa được xác nhận.

**Kết quả:** `KEEP` (giữ, bằng chứng chưa đủ để đổi), `LIMITED_REFACTOR`, `REDESIGN_PROPOSED` (đề xuất, chờ owner),
`APPROVED` (owner đã duyệt thay đổi), `REJECTED`. Chỉ owner chuyển `REDESIGN_PROPOSED` thành `APPROVED`.

`arch_health.py` chỉ đếm tín hiệu **sau `Window end` mới nhất** của hệ con đó. Một review `KEEP` làm im lặng lịch sử đã xét,
và tín hiệu quay lại khi có bằng chứng mới.

## 6. Quản lý lệch (deviation)

Sổ duy nhất là bảng "Lệch thiết kế đang mở" ở [TRACEABILITY.md](TRACEABILITY.md): ID (`DV<n>`), hệ con, thiết kế gốc,
thực tế, lý do, rủi ro và phạm vi, loại (`TEMPORARY` hoặc `PERMANENT`), owner, **ngày hoặc sự kiện xem lại**, kế hoạch đóng,
bằng chứng, trạng thái (`OPEN`, `CLOSED`, `ACCEPTED`).

- `OPEN` bắt buộc có owner, ngày hoặc sự kiện xem lại, và kế hoạch đóng. Quá ngày xem lại thì `arch_check.py` cảnh báo
  (lỗi khi chạy `--strict-dates` trong CI).
- `CLOSED` cần bằng chứng (commit, test, `run:`).
- `ACCEPTED` (chấp nhận vĩnh viễn) chỉ dành cho lệch `PERMANENT` và phải trỏ tới D\* AGREED hoặc AR\* APPROVED. Một workaround
  không thành thiết kế chỉ vì nó tồn tại lâu.
- Dòng TRACEABILITY ở trạng thái `DEVIATED`/`BLOCKED` phải trỏ tới `DV*` đang `OPEN`.
- Từ 3 lệch mở, hoặc nhiều lệch chung nguyên nhân: gom vào **một** AR. Lệch chạm §2 của spec hoặc một invariant là L3.
- Cần owner quyết thì mở O\* trong [DECISIONS.md](architecture/DECISIONS.md) như trước; `DV*` là bản ghi trạng thái, `O*` là câu hỏi.

## 7. Vòng làm việc

```text
Architecture → Task → Implementation → Discovery → Phân loại tác động → Review (nếu cần) → Verification → Đối soát tài liệu → Task kế
```

| Bước | Việc |
|---|---|
| Task | Nêu mục spec (§), quyết định (D\*), tiêu chí chấp nhận. Plan đã có sẵn các mục này |
| Implementation | Commit kèm trailer khi chạm hợp đồng hay invariant. Báo cáo task nêu: invariant đã chạm, cờ/trạng thái thêm, workaround thêm (và nguyên nhân gốc) |
| Discovery | Phát hiện lệch spec: dừng theo AGENTS.md §1; ghi O\* hoặc `DV*`; không sửa kiến trúc âm thầm |
| Phân loại | L0–L3 (§3). Chạm trạng thái mới hoặc invariant thì dừng và báo |
| Verification | Test theo mức: thành phần, tích hợp, nghiệm thu. Chỉ chạy mới tính; không suy diễn PASS |
| Đối soát | Docs → Code: dòng TRACEABILITY chỉ ở mức đã có bằng chứng. Code → Docs: đổi contract, ngữ nghĩa trạng thái, chủ sở hữu hay ranh giới thì cập nhật spec cùng thay đổi. Test → Docs: kết quả đổi nhận định thì đổi mức |
| Cuối WP | `make gate` (chạy kiểm tĩnh gồm `arch_check` và `check_change`); xem `arch_health.py` trước khi đóng lát |

**Quy tắc kiến trúc như đã xây (D45).** SYSTEM_DESIGN và plan là bản phác thảo và lộ trình; trong lúc làm, code chạy sẽ phát sinh và sửa đổi. Người trực tiếp làm là người duy nhất nắm đúng thực tại, nên khi một phần đã xong và đã kiểm thử ổn định (mức `COMPONENT_VERIFIED` trở lên, không còn sửa cấu trúc), họ **vừa làm vừa vẽ lại** phần đó vào [AS_BUILT.md](architecture/AS_BUILT.md), cùng lát với việc làm, chậm nhất ở task đóng lát:
- mỗi package sản phẩm có một mục `## <package>` với sơ đồ Mermaid (process, thread, topic và QoS, hàng đợi, state machine như trong code), mô tả ngắn thứ có thật, bảng trạng thái/cạnh lấy từ code, bằng chứng (test, `run:`), và chỗ lệch so với SYSTEM_DESIGN (trỏ tới D\*/O\*/DV\*). Dòng `Verified at: `<sha>`` ghi commit mà sơ đồ mô tả;
- chỉ vẽ cái đã chạy được và đã kiểm; phần chưa xây ghi "chưa xây", không vẽ theo thiết kế;
- khi lệch thiết kế, AS_BUILT thắng về hiện trạng và SYSTEM_DESIGN thắng về ý định; chỗ lệch thành `DV*` hoặc cập nhật spec qua D\*, không để hai bản nói khác nhau mà không ai ghi;
- `arch_check.py` bắt buộc mỗi package sản phẩm có mục, sơ đồ và commit có thật, và cảnh báo khi mục đã cũ hơn code quá `as_built_stale_commits` commit. Báo cáo task nêu thêm dòng `As-built:` (đã cập nhật mục nào, hay vì sao chưa cần).

Tiến độ báo cáo theo phạm vi: *dự kiến*, *đã code*, *đã kiểm chứng*, *đã nghiệm thu*, *rủi ro mở*, *thêm/bớt phạm vi*,
*lệch*. Không dùng số commit, số dòng hay tỷ lệ task làm tỷ lệ hoàn thành. Chưa có trọng số hay tiêu chí nghiệm thu rõ thì
ghi "chưa tính được".

## 8. Tự động và việc của người

| Kiểm tra | Cách | Chặn merge? |
|---|---|---|
| Link tài liệu, trích dẫn `file:dòng` | `tools/check_documentation.py` | có |
| Trạng thái đúng từ vựng; bằng chứng (test, commit, đường dẫn) có thật; QUALIFIED có quyết định AGREED | `arch_check.py` | có |
| Sổ lệch đủ trường; lệch quá hạn | `arch_check.py` | có (quá hạn: chỉ khi `--strict-dates` ở CI) |
| Tham chiếu D\*/O\*/AR\* và §spec tồn tại | `arch_check.py` | có |
| Hướng phụ thuộc: package ROS-free không include hay depend ROS; cạnh `package.xml`; không in thẳng stdout/stderr | `arch_check.py` | có |
| Commit chạm hợp đồng/invariant có trailer; L1 có docs; enumerator trạng thái mới cần L2 có AR; L3 có D\* | `check_change.py` | có |
| PR mô tả có `Arch-Level` và `Docs-Impact` không thấp hơn commit | `check_change.py --body-file` | có |
| Tín hiệu stress | `arch_health.py` | không (báo cáo trên PR; người quyết định) |
| Thiết kế còn đúng không; mức khai báo có đúng không; logic lặp, rò trách nhiệm | **Architecture Reviewer** | quyết định của người |
| Build ROS, test, TSan, SITL | `make gate`, `make sanitize`, SITL trên máy owner (CI công khai không chạy ROS và SITL) | ngoài CI |

## 9. Cơ chế này có đáng không (chống bộ máy cồng kềnh)

| Cơ chế | Giải quyết rủi ro | Dùng cái có sẵn được không | Chi phí duy trì | Tự động | Bỏ khi |
|---|---|---|---|---|---|
| Thang trạng thái | báo cáo "done" cho việc mới chạy unit test | mở rộng cột trạng thái cũ | thấp; máy kiểm | có | không bao giờ cần ngược lại |
| Trailer commit | thay đổi hợp đồng đi vào mà không ai nhìn | trailer Git, không cần công cụ | 2 dòng cho commit chạm hợp đồng | có | khi PR template đủ và reviewer ổn định |
| `arch_check.py` | tài liệu trôi khỏi bằng chứng; phình phụ thuộc | mở rộng `check_documentation.py` về ý tưởng, tách file vì phạm vi khác | thấp; luật nằm ở YAML | có | khi gate khác phủ cùng kiểm tra |
| `arch_health.py` | xói mòn âm thầm qua nhiều thay đổi nhỏ | Git history | thấp; ngưỡng hiệu chỉnh | có | khi không còn tín hiệu qua 3 lát liền |
| AR\* + sổ DV\* | quyết định kiến trúc và lệch tạm thời không có chủ | O\*/D\* và bảng lệch đã có | trung bình, chỉ khi vượt ngưỡng | một phần | khi dự án chuyển sang bảo trì |

Không thêm file quản lý, công cụ hay bảng trạng thái thứ hai cho cùng một thông tin. Cơ chế mới phải trả lời cả năm cột trên.

## 10. Áp dụng cho việc tiếp theo

Không phải chuyển đổi toàn repo. Lịch sử trước `baseline_commit` không bị đánh giá lại. Từ đó:

1. Commit chạm hợp đồng, invariant hoặc enum trạng thái kèm trailer (§3). Commit khác không đổi gì.
2. Báo cáo task nêu ba dòng ở §7. Subagent không tự khai L2/L3: dừng và báo.
3. Dòng TRACEABILITY mới dùng thang §2; dòng cũ đã chuyển sang thang này (ánh xạ: `todo`→`DESIGNED`, `doing`/`done` có test→`COMPONENT_VERIFIED`, có code chưa kiểm→`IMPLEMENTED`).
4. Chạy `python3 -m tools.uavnav.arch_health` ở cuối mỗi lát; vượt ngưỡng thì làm AR.
