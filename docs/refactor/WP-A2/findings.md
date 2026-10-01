# WP-A2 findings

Phân loại: `CONFIRMED` là thấy trực tiếp trên baseline và suy ra được từ
source/config; `CONDITIONAL` là đường code có thật nhưng phụ thuộc launch,
parameter hoặc external endpoint; `NOT_EVALUABLE` là chưa có phép đo runtime.

## CONFIRMED

1. **QoS tài liệu/helper lệch QoS thực của `/lio/mapping_observation`.**
   `QosProfiles::mappingObservation()` khai báo best-effort, volatile, depth 1
   tại `src/estimation/fast_lio_ros/src/qos_profiles.cpp:25-27`, nhưng publisher
   thực tế dùng `QosProfiles::estimatorOutput()` (reliable, depth 10) tại
   `src/estimation/fast_lio_ros/src/ros_output_publisher.cpp:120-121`. Runtime
   subscriber là best-effort depth 1 tại
   `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:1726-1729`.
   Đây là mismatch của intended profile với code path đang chạy; DDS
   reliable-writer/best-effort-reader vẫn tương thích.

2. **`/lidar/free_space_endpoints` có subscriber product nhưng publisher SITL-only.**
   FAST-LIO tạo subscription khi `input.visibility_points_topic` không rỗng tại
   `src/estimation/fast_lio_ros/src/fast_lio_node.cpp:212-218`; publisher duy nhất
   là `gz_visibility_bridge` tại `src/uav_simulation/src/visibility_bridge.cpp:41-47`.
   Runner chỉ khởi động bridge khi không ở characterization tại
   `tools/runtime/runner.py:3692-3706`. Product config `config/runtime/sim.yaml`
   vẫn bật topic này.

3. **Cặp interface mission có điều kiện tạo không đối xứng.**
   `navigation_runtime_node` chỉ tạo `/navigation/command_admission`,
   `/navigation/mission_progress` và `/navigation/mission_complete` khi
   `mission_progress_` tồn tại tại
   `src/runtime/navigation_runtime/src/navigation_runtime_node.cpp:1753-1763`.
   Mode node lại luôn tạo subscriber `/navigation/mission_progress` tại
   `src/px4/px4_navigation_external_mode/src/navigation_mode_node.cpp:291-296`.
   Do đó subscriber có thể tồn tại mà không có publisher khi mission không được
   tạo; đây là `CONFIRMED` về topology tạo interface, còn tác động runtime
   trong từng launch là `CONDITIONAL`.

4. **Có publisher không có subscriber product tương ứng.**
   Các topic được source inventory ghi nhận là `/lio/registered_points`,
   `/lio/odometry_transport_trace`, `/navigation/odometry_ingress_trace`,
   `/px4/estimator_odometry`, và `/px4/diagnostics`; các vị trí publisher và
   `subscribers: []` nằm trong `icd_topics.yaml`. Đây là đường observability/
   SITL, không được coi là product data path.

5. **Product dùng diagnostics chuỗi để safety gate.**
   `px4_external_odometry_bridge_node` chỉ chấp nhận health khi
   `DiagnosticStatus.name == "fast_lio/estimator"`, timestamp tăng, generation
   hợp lệ, `navigation_valid`, `corrected_estimate_valid`, `status=TRACKING`,
   và covariance keys thỏa điều kiện tại
   `src/px4/px4_odometry_bridge/src/px4_external_odometry_bridge_node.cpp:156-203`.
   Kênh typed `/lio/health` cũng được publish tại
   `src/estimation/fast_lio_ros/src/ros_output_publisher.cpp:122-123`, nhưng không
   thay thế parser diagnostics này trên bridge path. ICD có 17 channel entries;
   `navigation_planning/planner` là compatibility alias chỉ có parser, không có
   publisher baseline.

6. **Một số subscriber product chỉ có publisher external/PX4.**
   `/fmu/in/vehicle_visual_odometry`, `/fmu/out/*`, và các endpoint mode của
   `px4_ros2_interface_lib` được tạo/tiêu thụ ở ranh giới PX4 hoặc thư viện
   pinned, không có publisher/subscriber đối xứng trong clone này. ICD đánh dấu
   endpoint bằng `px4_fmu` hoặc `px4_ros2_interface_lib` và không giả định QoS
   wire của PX4 khi source không công bố.

## CONDITIONAL / evidence gap

7. **Rate thực chưa được đo.** Các giá trị 50 Hz, 20 Hz, 2 Hz trong ICD là
   timer/config expectations, không phải rate observed. `icd_topics.yaml` giữ
   `NOT_MEASURED` cho stream chưa có trace/bag rate calculation.

8. **QoS bridge `ros_gz` và PX4 transport không thể chứng minh đầy đủ bằng clone.**
   `tools/runtime/runner.py:3677-3690` khởi động parameter bridge declaratively;
   các publisher PX4 do firmware/DDS endpoint sở hữu. Vì vậy ICD ghi
   `NOT_STATICALLY_DECLARED` thay vì bịa giá trị offered QoS.

9. **`/fmu/in/*` scenario streams có nhiều publisher SITL.**
   `external_mode_scenario`, `closed_loop_characterization`, và
   `offboard_scenario` cùng tạo `VehicleCommand`/`OffboardControlMode`/
   `TrajectorySetpoint` ở các source anchor trong `icd_topics.yaml`; đây là
   mutually-exclusive runner modes, không phải một topology đồng thời đã được
   chứng minh.

## NOT_EVALUABLE

10. **Không kết luận runtime message loss, callback starvation hoặc negotiated
    QoS từ static ICD.** Cần rosbag/runtime graph ở từng profile và đối chiếu
    expected/emitted/dropped trước khi dùng làm acceptance evidence.

11. **Không có field navigation-contract nào được gắn `unused: true` chỉ vì
    không thấy một consumer trong product.** Các field evidence-only vẫn được
    ghi rõ reader observer/judge; còn PX4 schema fields không chạm tới được đánh
    dấu `unused: true` trong `icd_msgs.yaml`, với source schema của pinned
    submodule.
