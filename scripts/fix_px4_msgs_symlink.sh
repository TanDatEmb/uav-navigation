#!/bin/bash
# fix_px4_msgs_symlink.sh - Fix px4_msgs symlink error
#
# Lỗi: "failed to create symbolic link ... Is a directory"
# Nguyên nhân: Thư mục Python cũ tồn tại, CMake không xóa được
#
# Usage: ./fix_px4_msgs_symlink.sh

set -e

echo "Fixing px4_msgs symlink error..."
echo ""

# Xóa build cũ
echo "1. Removing old build artifacts..."
rm -rf build/px4_msgs install/px4_msgs log/build_px4_msgs* 2>/dev/null || true

# Xóa thư mục Python cũ nếu tồn tại
if [ -d "build/px4_msgs/ament_cmake_python" ]; then
    echo "2. Removing old Python directory..."
    rm -rf build/px4_msgs/ament_cmake_python
fi

echo "3. Rebuilding px4_msgs..."
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "✅ px4_msgs rebuilt successfully!"
echo ""
echo "Bạn có thể chạy 'make build' tiếp theo:"
echo "  cd ~/Dev/uav-navigation && make build"
