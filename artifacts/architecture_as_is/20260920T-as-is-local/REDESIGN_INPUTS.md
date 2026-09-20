# Inputs cho buổi thảo luận thiết kế tiếp theo

Tài liệu này chỉ nêu câu hỏi và vấn đề cần quyết định dựa trên AS-IS; không đưa ra FSM, owner hay kiến trúc TO-BE.

## Ưu tiên điều khiển và an toàn

1. Cấu hình nào được xem là chuẩn cho SIM_TIME và tracking experiment? Code đặt `suppress_braking` cùng `suppress_estimator_health_response` true khi `use_sim_time=true` và các hệ số tracking bằng zero; YAML comments lại mô tả zero như vô hiệu hóa gate. Cần quyết định cách giữ alignment giữa source policy, config comment và profile được phép dùng. Hiệu ứng hiện tại là conditional theo launch parameter, chưa có trace profile thật. [FIND-04](tables/authority_findings.md) · [evidence](evidence/refs.html#E_TRACKING_POLICY).
2. `scheduleMode(AUTO_LOITER)` trả Success nhưng chưa có `VehicleStatus.AUTO_LOITER` thì hệ thống có nên tiếp tục retry/await không? Code hiện xóa pending ở API Success/Deactivated; xác nhận status là field khác. Cần test với thứ tự API callback/status, mất status và timeout, trước khi kết luận đây là failure runtime. [FIND-02](tables/authority_findings.md) · [executor evidence](evidence/refs.html#E_HOLD).
3. Biên nào được phép coi `NavigationCommand` đã được chấp nhận để progression mission? Hiện có runtime publish gate, mode message admission, mission witness và PX4 setpoint update riêng; source không chứng minh PX4 đã thực sự áp dụng command. Xác định evidence cần có cho mỗi boundary khi chuẩn bị thiết kế, không gộp các lifecycle fact. [D_EXPOSE/D_ADMIT/D_SETPOINT](tables/decisions.md).

## Ownership và liveness

4. Trong PASS_THROUGH, khi `MissionController` đã tăng waypoint/request nhưng runtime còn giữ `executing_goal_` và bundle cũ, identity/lease nào là căn cứ cho mỗi callback? Source có exact tuple, pending single-slot và timeline activation, nhưng chưa có paired-process test/trace cho reordered goal/result/command. [FIND-01](tables/authority_findings.md) · [S03](tables/scenarios.md).
5. Runtime `ExecutionEpisode::StoppedRecovery` và PX4 adapter `planner_recovery_pending_` phục vụ hai lifetime/clock khác nhau. Cần xác định quan hệ mong đợi khi một timer hết trước timer kia, nhất là khi command callback đến muộn hoặc identity đổi. [FIND-03](tables/authority_findings.md) · [S06](tables/scenarios.md).
6. Worker, timeline store, mission callback group và PX4 state-input thread có những linearization point riêng. Cần chỉ rõ callback order nào được test hoặc quan sát runtime trước khi mô tả cross-process sequence như invariant. [Concurrency refs](tables/transitions.md).

## Semantics cần khóa rõ

7. Comment `onActivate()` nói goal đầu được publish lúc activate, nhưng `MissionController::update` trả none khi chưa airborne; test xác nhận đợi `isArmed && z>0.5`. Cần chọn comment/current behavior nào phản ánh contract, và xác nhận activation trong profile mục tiêu. Audit này giữ nguyên cả hai chứng cứ. [S01](tables/scenarios.md) · [E_ACTIVATE_COMMENT](evidence/refs.html#E_ACTIVATE_COMMENT) · [E_MISSION_TEST](evidence/refs.html#E_MISSION_TEST).
8. Coincident PASS_THROUGH→STOP có semantics riêng. Test đó không xác lập mọi route zero-length hoặc các tổ hợp trùng tọa độ khác; cần thêm scenario review nếu mission input cho phép. [S08](tables/scenarios.md).
9. Candidate validation, commit, stage, activation, ROS publish, mode admission, setpoint update và PX4 status là các outcome khác nhau. Cần chọn bằng chứng nào bắt buộc ở mỗi boundary và identity nào liên kết chúng; không có flight/runtime evidence trong bộ tài liệu này.

## Finding và mức chứng cứ

Bảng [authority findings](tables/authority_findings.md) ghi classification, supporting evidence, cơ chế serialization/identity có thể làm representation hợp lệ, test coverage và gap cho bốn điểm. `CONDITIONAL` nghĩa là source path tồn tại nhưng effect phụ thuộc identity/config/order hoặc chưa được quan sát; không đồng nghĩa có lỗi runtime đã chứng minh.
