# PX4 Autopilot Extras — SITL với Livox MID-360

Thư mục này lưu các file mô phỏng CTUAV cần để chạy `uav-navigation` với PX4
`release/1.17`, Gazebo Harmonic và Livox MID-360. Các file không thuộc upstream
PX4; `vendor/` là source of truth, còn `Tools/simulation/gz/` chỉ là bản triển
khai để chạy local.

## Cấu trúc

```text
vendor/px4_autopilot_extras/
├── airframes/
│   └── 4022_gz_x500_lidar_360
├── airframes_CMakeLists.patch
├── gz_models/
│   ├── lidar_mid360/
│   ├── map_loop_arena/                  # Arena vuông 80 m
│   ├── map_tree/                        # Cây primitive có collision
│   ├── wire_test_gate/                  # Dây/rod theo cấp đường kính
│   └── x500_lidar_360/
└── gz_worlds/
    └── obstacle_course.sdf
```

## Naming convention

Entity trong world dùng thứ tự:

```text
<zone>_<type>_<station>_<profile-or-side>_<shape>
```

Ví dụ:

- `corridor_wall_east`
- `corridor_pillar_route_040m`
- `corridor_tree_west_015m`
- `corridor_mover_025m_slow_crate`
- `corridor_mover_105m_medium_barrel`
- `corridor_mover_185m_fast_panel`
- `loop_arena_square_080m`
- `loop_wire_gate_222m`

Tên station luôn zero-pad ba chữ số để Scene Tree sắp xếp theo khoảng cách.

## Áp dụng tự động

```bash
cd ~/Dev/uav-navigation
bash tools/apply_px4_extras.sh [PX4_DIR]
```

Mặc định `PX4_DIR=${HOME}/Dev/Autopilot`. Script sẽ:

1. Copy airframe `4022_gz_x500_lidar_360`.
2. Patch `ROMFS/.../airframes/CMakeLists.txt`.
3. Checkout Gazebo submodule về baseline thử nghiệm nếu commit có sẵn.
4. Copy `lidar_mid360`, `map_loop_arena`, `map_tree`, `wire_test_gate` và
   `x500_lidar_360`.
5. Copy world `obstacle_course.sdf`.

## Áp dụng thủ công

```bash
cd ~/Dev/Autopilot
EXTRAS=~/Dev/uav-navigation/vendor/px4_autopilot_extras

cp "${EXTRAS}/airframes/4022_gz_x500_lidar_360" \
   ROMFS/px4fmu_common/init.d-posix/airframes/
patch -p1 < "${EXTRAS}/airframes_CMakeLists.patch"

cp -r "${EXTRAS}/gz_models/lidar_mid360" Tools/simulation/gz/models/
cp -r "${EXTRAS}/gz_models/map_loop_arena" Tools/simulation/gz/models/
cp -r "${EXTRAS}/gz_models/map_tree" Tools/simulation/gz/models/
cp -r "${EXTRAS}/gz_models/wire_test_gate" Tools/simulation/gz/models/
cp -r "${EXTRAS}/gz_models/x500_lidar_360" Tools/simulation/gz/models/
cp "${EXTRAS}/gz_worlds/obstacle_course.sdf" Tools/simulation/gz/worlds/
```

## Hình học test

- Corridor dài 220 m, dẫn vào arena tại `y=210 m`.
- Arena ngoài `80 x 80 m`: `x=[-40,40]`, `y=[210,290] m`.
- Island giữa `30 x 30 m`; hành lang vòng rộng tối thiểu `24.5 m`.
- Nominal square lap dài khoảng `220 m` và quay lại cùng điểm.
- Mover cao `8/10/12 m`; force/mass profile chậm/vừa/nhanh giữ nguyên.
- Mover là free rigid body với `<gravity>false>`: `TrajectoryFollower` vẫn điều
  khiển được, trong khi không có trọng lực/contact torque làm vật cao bị lật.
- Cổng dây tại `y=222 m`: dây `20/50/100 mm` ở cao `5/9/13 m`; rod đứng
  `40/80/150 mm`, cao 4 m.

Quy trình đo chi tiết:
[`docs/simulation_validation_course.md`](../../docs/simulation_validation_course.md).

## Validation

```bash
cd ~/Dev/uav-navigation
python3 tools/validate_obstacle_course.py

SDF_PATH="$PWD/vendor/px4_autopilot_extras/gz_models" \
  gz sdf -k vendor/px4_autopilot_extras/gz_worlds/obstacle_course.sdf
```

Sau khi copy vào PX4:

```bash
cmp vendor/px4_autopilot_extras/gz_worlds/obstacle_course.sdf \
    ~/Dev/Autopilot/Tools/simulation/gz/worlds/obstacle_course.sdf
```

## Lưu ý

- Gazebo không hot-reload world; phải restart để nhận tên và mover model mới.
- Không commit file custom trực tiếp vào upstream `PX4-gazebo-models`.
- `Tools/simulation/gz` là submodule; `git submodule update` có thể xóa các file
  untracked đã triển khai. Luôn giữ bản canonical trong `vendor/`.
- `TrajectoryFollower` là force-driven; tốc độ và overshoot phải đo runtime.
- FAST-LIO hiện không có loop-closure correction; arena chỉ đo revisit drift và
  map overlap.
