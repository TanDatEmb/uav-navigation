# PX4 Autopilot Extras — Backup for SITL with Livox MID-360

Thư mục này lưu trữ các file mô phỏng cần thiết để chạy SITL `uav-navigation` với PX4 `release/1.17` + Gazebo Harmonic + Livox MID-360.

Các file này **không thuộc upstream PX4** — đây là patch thử nghiệm của CTUAV. Khi cần triển khai lại trên một bản PX4 sạch, dùng script tự động hoặc copy/patch thủ công theo hướng dẫn bên dưới.

## Files trong thư mục này

```text
vendor/px4_autopilot_extras/
├── README.md                              # Hướng dẫn này
├── airframes/
│   └── 4022_gz_x500_lidar_360           # Airframe x500 + Livox MID-360
├── airframes_CMakeLists.patch           # Diff thêm airframe vào CMakeLists
├── gz_models/
│   ├── lidar_mid360/
│   │   ├── model.config                 # Mô tả Livox MID-360
│   │   └── model.sdf                    # Sensor standalone
│   └── x500_lidar_360/
│       ├── model.config                 # Mô tả x500 + MID-360
│       └── model.sdf                    # Include x500 + lidar_mid360
└── gz_worlds/
    └── obstacle_course.sdf              # World cho CP testing
```

## Cách áp dụng tự động (khuyến nghị)

Từ thư mục `uav-navigation`:

```bash
bash tools/apply_px4_extras.sh [PX4_DIR]
```

Mặc định `PX4_DIR=${HOME}/Dev/Autopilot`. Script sẽ:

1. Copy airframe `4022_gz_x500_lidar_360`.
2. Patch `ROMFS/.../airframes/CMakeLists.txt`.
3. Copy Gazebo models `lidar_mid360` và `x500_lidar_360`.
4. Copy world `obstacle_course.sdf`.
5. Checkout submodule `Tools/simulation/gz` về commit thử nghiệm `606c099` (nếu có sẵn).

## Cách áp dụng thủ công

Giả sử PX4 repo ở `~/Dev/Autopilot` và đang ở branch `release/1.17`:

```bash
cd ~/Dev/Autopilot

EXTRAS=~/Dev/uav-navigation/vendor/px4_autopilot_extras

# 1. Copy airframe
cp "${EXTRAS}/airframes/4022_gz_x500_lidar_360" \
   ROMFS/px4fmu_common/init.d-posix/airframes/

# 2. Patch CMakeLists để build system nhận airframe mới
patch -p1 < "${EXTRAS}/airframes_CMakeLists.patch"

# 3. Copy Gazebo models
cp -r "${EXTRAS}/gz_models/lidar_mid360" Tools/simulation/gz/models/
cp -r "${EXTRAS}/gz_models/x500_lidar_360" Tools/simulation/gz/models/

# 4. Copy Gazebo world
cp "${EXTRAS}/gz_worlds/obstacle_course.sdf" Tools/simulation/gz/worlds/

# 5. Checkout submodule Gazebo về commit chứa obstacle_course world
#    Commit này là phiên bản thử nghiệm, không phải upstream.
cd Tools/simulation/gz
git checkout 606c099   # hoặc commit tương đương nếu đã thay đổi
```

## Kiểm tra sau khi áp dụng

```bash
cd ~/Dev/Autopilot
git status --short

# Kỳ vọng:
# A  ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_x500_lidar_360
# M  ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
#  M Tools/simulation/gz
```

Sau đó build PX4 SITL. Vì submodule `Tools/simulation/gz` đang ở commit thử nghiệm
khác với upstream, bạn cần cho phép build sử dụng submodule hiện tại:

```bash
cd ~/Dev/Autopilot
GIT_SUBMODULES_ARE_EVIL=1 make px4_sitl_default -j$(nproc)
```

Hoặc nếu build hỏi `y/n`, chọn `y` để tiếp tục với commit thử nghiệm.

## Chạy SITL từ uav-navigation

```bash
cd ~/Dev/uav-navigation
make sim        # hoặc make sim-headless
```

## Lưu ý

- **Không commit các file này vào PX4 upstream** nếu chưa được review.
- `Tools/simulation/gz` là submodule. Nếu chạy `git submodule update --recursive`, các thay đổi trong submodule sẽ bị mất. Hãy đảm bảo đã checkout đúng commit thử nghiệm trước khi chạy SITL.
- Nếu muốn lưu dài hạn hơn, hãy tạo branch trên fork riêng thay vì để ở working tree.
- Nếu commit hash `606c099` của submodule bị thay đổi trong tương lai, cập nhật `GZ_COMMIT` trong `tools/apply_px4_extras.sh`.
