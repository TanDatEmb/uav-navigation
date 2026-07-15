# ikd-Tree Architecture Review & UAV Integration Proposal

**Date:** 2026-07-14
**Expert:** FAST-LIO2 / ikd-Tree Specialist
**Context:** Real-time UAV Navigation with MID-360 LiDAR

---

## Executive Summary

Đánh giá kiến trúc FAST-LIO2 hiện tại và đề xuất tích hợp ikd-Tree cho UAV navigation.
- **Target:** 10-20Hz real-time performance với MID-360 LiDAR
- **Constraint:** Không dùng PCL KD-tree (không incremental), bắt buộc dùng ikd-Tree

---

## 1. Tại sao ikd-Tree cần thiết cho FAST-LIO2 Real-time?

### 1.1 Vấn đề với PCL KD-tree (Non-incremental)

| Aspect | PCL KD-tree | ikd-Tree |
|--------|-------------|----------|
| **Update mechanism** | Rebuild entire tree | Incremental add/delete |
| **Time complexity (add N points)** | O(N log N) rebuild | O(log N) per point |
| **Memory allocation** | Frequent large allocations | Minimal, incremental |
| **Thread safety** | None | Multi-threaded rebuild |
| **UAV suitability** | ❌ Unacceptable latency | ✅ Real-time capable |

### 1.2 UAV Scenario Requirements

```
MID-360 LiDAR characteristics:
├── Point rate: ~100,000 points/sec (10Hz)
├── Detection range: 0.1-100m
├── FOV: 360° horizontal × 59° vertical
└── UAV dynamics: High acceleration, agile maneuvers

Real-time constraints:
├── Target latency: <50ms per frame (20Hz)
├── Max acceptable: <100ms per frame (10Hz)
├── Memory: Limited on embedded (Jetson/Jetson Orin)
└── CPU: ARM Cortex-A78AE (octa-core)
```

**Key insight:** Với PCL KD-tree, mỗi frame cần rebuild tree với ~10,000 điểm mới → **O(10,000 × log 10,000) ≈ 130,000 operations** liên tục. Với ikd-Tree, mỗi điểm chỉ cần **O(log N)** và **lazy rebuild** khi tree unbalanced.

---

## 2. Cấu trúc ikd-Tree (Phân tích từ HKU Implementation)

### 2.1 Core Data Structure

```cpp
// From ikd_Tree.h - HKU implementation
template <typename PointType>
class KD_TREE {
    struct KD_TREE_NODE {
        PointType point;
        int division_axis;           // 0=x, 1=y, 2=z
        int TreeSize = 1;            // Total nodes in subtree
        int invalid_point_num = 0;   // Marked for deletion
        int down_del_num = 0;        // Downsample deletion count

        // Flags for lazy operations
        bool point_deleted = false;  // Soft delete flag
        bool tree_deleted = false;   // Subtree marked for delete
        bool need_push_down_to_left = false;
        bool need_push_down_to_right = false;
        bool working_flag = false;   // Lock during rebuild

        pthread_mutex_t push_down_mutex_lock;

        // Bounding box for subtree
        float node_range_x[2], node_range_y[2], node_range_z[2];
        float radius_sq;

        KD_TREE_NODE *left_son_ptr = nullptr;
        KD_TREE_NODE *right_son_ptr = nullptr;
        KD_TREE_NODE *father_ptr = nullptr;

        // Balance criteria (from paper)
        float alpha_bal;  // Balance threshold
        float alpha_del;  // Deletion threshold
    };
};
```

### 2.2 Three-Phase Operation Model

```
ikd-Tree Operation Flow:

┌─────────────────────────────────────────────────────────────┐
│  PHASE 1: NORMAL OPERATION                                  │
│  - Add_Points(): O(log N) per point                         │
│  - Delete_Point_Boxes(): Lazy marking                       │
│  - Nearest_Search(): O(log N) query                         │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼ (When α_del or α_bal exceeded)
┌─────────────────────────────────────────────────────────────┐
│  PHASE 2: REBUILD DETECTION                                 │
│  - Criterion_Check():                                        │
│    ├── α_bal > balance_criterion_param (0.7) → unbalanced  │
│    └── α_del > delete_criterion_param (0.5) → too many deleted│
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  PHASE 3: PARALLEL REBUILD (Background Thread)              │
│  - Spawn rebuild_thread via pthread_create                  │
│  - Flatten valid points to Rebuild_PCL_Storage              │
│  - BuildTree(): O(N log N) but offline                      │
│  - Apply Rebuild_Logger operations to new tree              │
│  - Atomic pointer swap: Rebuild_Ptr                         │
└─────────────────────────────────────────────────────────────┘
```

### 2.3 Key Parameters (Tuned for UAV)

```cpp
// From HKU implementation - should be adjusted for UAV
#define Minimal_Unbalanced_Tree_Size 10      // Min size to trigger rebuild
#define Multi_Thread_Rebuild_Point_Num 1500    // Threshold for parallel rebuild
#define DOWNSAMPLE_SWITCH true                 // Enable voxel downsampling
#define ForceRebuildPercentage 0.2             // Force rebuild at 20% invalid
#define Q_LEN 1000000                          // Operation queue length

// UAV-optimized tuning:
constexpr float UAV_DELETE_CRITERION = 0.4f;   // More aggressive cleanup
constexpr float UAV_BALANCE_CRITERION = 0.6f;  // Keep tree tighter
constexpr float UAV_MAP_RESOLUTION = 0.15f;    // 15cm for UAV precision
constexpr float UAV_CUBE_LEN = 100.0f;        // 100m local map
```

---

## 3. Multi-threading trong ikd-Tree

### 3.1 Thread Model Analysis

```
ikd-Tree Thread Architecture:

Main Thread (Real-time loop):
├── Nearest_Search() ─────┐
├── Add_Points() ─────────┤
├── Delete_Point_Boxes() ─┤ → pthread_mutex_lock(&working_flag_mutex)
└── Process callbacks ─────┘

Background Thread (Rebuild):
├── multi_thread_rebuild() ───────┐
│   ├── pthread_mutex_lock(&rebuild_ptr_mutex_lock)
│   ├── flatten() ──────────────┤
│   ├── BuildTree() ────────────┤ → Offline, doesn't block main
│   ├── Apply Rebuild_Logger ───┤
│   └── Atomic tree swap ───────┘
└── pthread_join() on destruction
```

### 3.2 Mutex Hierarchy (Critical for Deadlock Prevention)

```cpp
// From ikd_Tree.cpp - mutex acquisition order MATTERS

// Correct order (must be maintained):
1. search_flag_mutex      - Shortest hold time
2. working_flag_mutex     - Protects tree structure
3. rebuild_ptr_mutex_lock - Rebuild coordination
4. rebuild_logger_mutex_lock - Operation logging
5. points_deleted_rebuild_mutex_lock - Deleted points storage

// Deadlock scenario to avoid:
// Thread A: lock(m1) → wait for m2
// Thread B: lock(m2) → wait for m1
// Solution: Global ordering enforced in code
```

### 3.3 Search Counter Pattern (Lock-free reads)

```cpp
// From Nearest_Search() implementation:
void KD_TREE<PointType>::Nearest_Search(...) {
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node) {
        // Fast path: No rebuild in progress
        Search(Root_Node, k_nearest, point, q, max_dist);
    } else {
        // Slow path: Rebuild in progress - use counter
        pthread_mutex_lock(&search_flag_mutex);
        while (search_mutex_counter == -1) {
            // Wait for rebuild to release
            pthread_mutex_unlock(&search_flag_mutex);
            usleep(1);  // Yield
            pthread_mutex_lock(&search_flag_mutex);
        }
        search_mutex_counter += 1;  // Register reader
        pthread_mutex_unlock(&search_flag_mutex);

        Search(...);  // Safe to search

        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;  // Unregister reader
        pthread_mutex_unlock(&search_flag_mutex);
    }
}
```

---

## 4. Cách Integrate vào ROS2 Package

### 4.1 Current Architecture Analysis (FASTLIO2_ROS2)

```
Current FASTLIO2_ROS2 Architecture:

lio_node.cpp (ROS2 Node)
├── MapBuilder (map_builder.h/cpp)
│   ├── IMUProcessor (imu_processor.h/cpp) ───┐
│   │   └── Motion undistortion              │
│   └── LidarProcessor (lidar_processor.h/cpp)
│       └── m_ikdtree: KD_TREE<PointType>    │
│           ├── Build()                      │
│           ├── Nearest_Search()           │
│           ├── Add_Points()               │
│           └── Delete_Point_Boxes()       │
└── IESKF (ieskf.h/cpp) ◄────────────────────┘
    └── Iterated ESKF for state estimation
```

### 4.2 Architecture Problems Identified

| Problem | Severity | Description |
|---------|----------|-------------|
| **Template instantiation overhead** | Medium | `KD_TREE<PointType>` template in header |
| **No memory pool** | High | Dynamic allocation per node insertion |
| **pthread instead of std::thread** | Medium | Not C++17 idiomatic, harder to integrate |
| **No ROS2 logging** | Low | printf() instead of RCLCPP macros |
| **No lifecycle management** | High | No proper destruction on node shutdown |
| **Lock contention risk** | Critical | Search and update in same thread |

### 4.3 Proposed Integration Architecture

```
UAV-Optimized ikd-Tree Architecture:

┌──────────────────────────────────────────────────────────────┐
│                    LIONode (ROS2 Lifecycle)                  │
│  ┌────────────────────────────────────────────────────────┐  │
│  │                   MapBuilder                            │  │
│  │  ┌──────────────────────────────────────────────────┐  │  │
│  │  │          IKDTreeWrapper (NEW)                   │  │  │
│  │  │  ┌────────────────────────────────────────┐   │  │  │
│  │  │  │        KD_TREE<PointType>             │   │  │  │
│  │  │  │  (Original HKU implementation)          │   │  │  │
│  │  │  └────────────────────────────────────────┘   │  │  │
│  │  │                                                 │  │  │
│  │  │  ┌─────────────┐  ┌─────────────┐  ┌─────────┐ │  │  │
│  │  │  │ Stats       │  │ Profiling   │  │ Memory  │ │  │  │
│  │  │  │ Monitor     │  │ Timer       │  │ Pool    │ │  │  │
│  │  │  └─────────────┘  └─────────────┘  └─────────┘ │  │  │
│  │  └──────────────────────────────────────────────────┘  │  │
│  │                                                         │  │
│  │  ┌──────────────┐  ┌──────────────┐  ┌─────────────┐  │  │
│  │  │ IMUProcessor │  │LidarProcessor│  │   IESKF     │  │  │
│  │  └──────────────┘  └──────────────┘  └─────────────┘  │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘

Key improvements:
1. RAII wrapper cho proper lifecycle
2. Memory pool pre-allocation
3. Real-time statistics
4. Integration với ROS2 executor
```

---

## 5. Trade-offs: Incremental vs Batch KD-tree

### 5.1 Quantitative Comparison

| Metric | Batch KD-tree (PCL) | ikd-Tree (Incremental) | Winner |
|--------|---------------------|------------------------|--------|
| **Add 1000 points** | ~5-10ms rebuild | ~1-2ms incremental | ikd-Tree |
| **Delete 500 points** | ~5-10ms rebuild | ~0.5ms lazy delete | ikd-Tree |
| **Query nearest 5** | ~0.05ms | ~0.05-0.1ms (with locking) | Comparable |
| **Memory overhead** | Minimal | ~20% for metadata | Batch |
| **Code complexity** | Low (PCL) | High (custom) | Batch |
| **Thread safety** | None | Full | ikd-Tree |
| **UAV 20Hz capable** | ❌ | ✅ | ikd-Tree |

### 5.2 When to Use Which

```
Decision Matrix:

Batch KD-tree appropriate for:
├── Offline mapping (SLAM without real-time constraint)
├── Small point clouds (<1000 points)
├── Single-threaded applications
└── When PCL compatibility is required

ikd-Tree required for:
├── Real-time UAV navigation (✓ Our case)
├── Large point clouds (>10,000 points)
├── Multi-threaded environments
├── Dynamic environments (add/delete frequent)
└── Embedded systems with limited CPU
```

---

## 6. Đánh giá Kiến trúc Hiện tại

### 6.1 Pros ✅

1. **Correct algorithmic implementation** - Follows FAST-LIO2 paper exactly
2. **Thread-safe rebuild** - Proper mutex hierarchy
3. **Lazy deletion** - Efficient for UAV's moving local map
4. **Voxel downsampling** - Built-in, reduces point count
5. **Template-based** - Type-safe for different point formats

### 6.2 Cons ❌

| Issue | Impact | Recommended Fix |
|-------|--------|---------------|
| `pthread` instead of `std::thread` | Portability | C++17 `std::thread` + `std::mutex` |
| No move semantics | Performance | Add `noexcept` move ctor/assignment |
| Raw pointers for nodes | Memory safety | Consider `std::unique_ptr` or pool |
| `printf` logging | ROS2 integration | Replace with `RCLCPP_*` macros |
| Fixed-size operation queue | Scalability | Dynamic queue with overflow handling |
| No real-time metrics | Debugging | Add timing hooks |
| Template in .cpp file | Build time | Explicit instantiation in header |

---

## 7. Checklist Implementation Cụ thể

### Phase 1: Foundation (Week 1)

```
□ Create ikd_tree_wrapper package
□ Wrap KD_TREE in RAII class
□ Add proper exception handling
□ Implement move semantics
□ Add ROS2 logging integration
□ Create unit tests for basic operations
□ Benchmark vs original implementation
```

### Phase 2: Optimization (Week 2)

```
□ Implement memory pool for nodes
□ Replace pthread with std::thread
□ Add lock-free statistics collection
□ Implement dynamic operation queue
□ Profile with MID-360 dataset
□ Optimize for ARM NEON (Jetson)
□ Add SIMD distance calculations
```

### Phase 3: Integration (Week 3)

```
□ Integrate with MapBuilder
□ Add ROS2 parameter server support
□ Implement dynamic_reconfigure equivalent
□ Add diagnostic topics (/diagnostics)
□ Create visualization markers
□ Test with UAV flight data
□ Validate 20Hz performance
```

### Phase 4: Hardening (Week 4)

```
□ Stress test with high-rate LiDAR
□ Test edge cases (empty tree, single point)
□ Validate memory usage over long runs
□ Add recovery from failed rebuilds
□ Document API and usage patterns
□ Create example launch files
□ Write integration tests
```

---

## 8. Code Skeleton cho ikd-Tree Wrapper

### 8.1 Header: `ikd_tree_wrapper.hpp`

```cpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <vector>

// Forward declare original ikd-Tree
namespace ikd {
    template<typename PointType>
    class KD_TREE;
}

namespace uav_navigation {

using PointType = pcl::PointXYZINormal;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

/**
 * @brief Configuration for ikd-Tree wrapper
 */
struct IKDTreeConfig {
    double delete_criterion_param = 0.5;
    double balance_criterion_param = 0.7;
    double downsample_size = 0.2;
    size_t max_queue_size = 1000000;
    bool enable_rebuild_thread = true;
    bool enable_profiling = true;
};

/**
 * @brief Real-time statistics for monitoring
 */
struct IKDTreeStats {
    std::atomic<size_t> total_points{0};
    std::atomic<size_t> valid_points{0};
    std::atomic<size_t> deleted_points{0};
    std::atomic<double> last_query_time_ms{0.0};
    std::atomic<double> last_update_time_ms{0.0};
    std::atomic<bool> rebuild_in_progress{false};
    std::atomic<size_t> rebuild_count{0};
};

/**
 * @brief RAII wrapper for ikd-Tree with ROS2 integration
 *
 * Design goals:
 * - Thread-safe operations
 * - Real-time performance monitoring
 * - Proper lifecycle management
 * - Memory pool optimization
 */
class IKDTreeWrapper {
public:
    explicit IKDTreeWrapper(const IKDTreeConfig& config = {});
    ~IKDTreeWrapper();

    // Non-copyable, movable
    IKDTreeWrapper(const IKDTreeWrapper&) = delete;
    IKDTreeWrapper& operator=(const IKDTreeWrapper&) = delete;
    IKDTreeWrapper(IKDTreeWrapper&&) noexcept;
    IKDTreeWrapper& operator=(IKDTreeWrapper&&) noexcept;

    // Lifecycle
    void initialize(rclcpp::Node::SharedPtr node);
    void shutdown();
    bool is_initialized() const { return initialized_; }

    // Core operations (thread-safe)
    void build(const PointVector& points);
    void addPoints(const PointVector& points, bool enable_downsample = true);
    void deletePoints(const PointVector& points);
    void deletePointBoxes(const std::vector<BoxPointType>& boxes);

    // Queries (thread-safe, lock-free when possible)
    void nearestSearch(const PointType& point, int k,
                       PointVector& nearest_points,
                       std::vector<float>& distances,
                       float max_dist = std::numeric_limits<float>::max());

    void boxSearch(const BoxPointType& box, PointVector& result);
    void radiusSearch(const PointType& point, float radius, PointVector& result);

    // Statistics
    size_t size() const;
    size_t validSize() const;
    const IKDTreeStats& getStats() const { return stats_; }

    // Diagnostics
    void publishDiagnostics();
    double getAverageQueryTime() const;
    double getAverageUpdateTime() const;

    // Configuration
    void setDownsampleParam(float size);
    void setDeleteCriterion(float param);
    void setBalanceCriterion(float param);

private:
    // Internal implementation
    std::unique_ptr<ikd::KD_TREE<PointType>> tree_;
    IKDTreeConfig config_;
    IKDTreeStats stats_;

    // ROS2 integration
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_pub_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // Threading
    std::thread profiler_thread_;
    std::atomic<bool> stop_profiler_{false};

    // State
    std::atomic<bool> initialized_{false};
    mutable std::shared_mutex state_mutex_;

    // Profiling data
    mutable std::mutex profiling_mutex_;
    std::queue<double> query_times_;
    std::queue<double> update_times_;
    static constexpr size_t MAX_PROFILING_SAMPLES = 100;

    // Helper methods
    void profilerLoop();
    void recordQueryTime(double ms);
    void recordUpdateTime(double ms);
};

/**
 * @brief Factory function for creating wrapper
 */
std::unique_ptr<IKDTreeWrapper> createIKDTreeWrapper(
    const IKDTreeConfig& config = {});

} // namespace uav_navigation
```

### 8.2 Implementation: `ikd_tree_wrapper.cpp`

```cpp
#include "ikd_tree_wrapper.hpp"
#include "ikd_Tree.h"  // Original HKU implementation
#include <pcl_conversions/pcl_conversions.hpp>
#include <chrono>

namespace uav_navigation {

// Explicit instantiation for common point types
template class IKDTreeWrapper::KD_TREE<PointType>;

IKDTreeWrapper::IKDTreeWrapper(const IKDTreeConfig& config)
    : config_(config)
{
    RCLCPP_INFO(rclcpp::get_logger("ikd_tree"),
                "Creating ikd-Tree wrapper");
}

IKDTreeWrapper::~IKDTreeWrapper() {
    shutdown();
}

IKDTreeWrapper::IKDTreeWrapper(IKDTreeWrapper&& other) noexcept
    : tree_(std::move(other.tree_))
    , config_(other.config_)
    , stats_(other.stats_.load())
    , node_(std::move(other.node_))
    , stop_profiler_(other.stop_profiler_.load())
    , initialized_(other.initialized_.load())
{
    other.initialized_ = false;
}

IKDTreeWrapper& IKDTreeWrapper::operator=(IKDTreeWrapper&& other) noexcept {
    if (this != &other) {
        shutdown();
        tree_ = std::move(other.tree_);
        config_ = other.config_;
        stats_ = other.stats_.load();
        node_ = std::move(other.node_);
        stop_profiler_ = other.stop_profiler_.load();
        initialized_ = other.initialized_.load();
        other.initialized_ = false;
    }
    return *this;
}

void IKDTreeWrapper::initialize(rclcpp::Node::SharedPtr node) {
    std::unique_lock lock(state_mutex_);

    if (initialized_) {
        RCLCPP_WARN(node->get_logger(), "Already initialized");
        return;
    }

    node_ = node;

    // Create underlying tree
    tree_ = std::make_unique<ikd::KD_TREE<PointType>>(
        config_.delete_criterion_param,
        config_.balance_criterion_param,
        config_.downsample_size
    );

    // Setup ROS2 publishers
    debug_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>(
        "ikd_tree/debug_cloud", 10);

    diag_timer_ = node->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { publishDiagnostics(); });

    // Start profiler thread if enabled
    if (config_.enable_profiling) {
        profiler_thread_ = std::thread(&IKDTreeWrapper::profilerLoop, this);
    }

    initialized_ = true;
    RCLCPP_INFO(node->get_logger(), "ikd-Tree wrapper initialized");
}

void IKDTreeWrapper::shutdown() {
    {
        std::unique_lock lock(state_mutex_);
        if (!initialized_) return;

        stop_profiler_ = true;
        initialized_ = false;
    }

    if (profiler_thread_.joinable()) {
        profiler_thread_.join();
    }

    // tree_ destructor handles thread cleanup
    tree_.reset();

    RCLCPP_INFO(node_->get_logger(), "ikd-Tree wrapper shut down");
}

void IKDTreeWrapper::build(const PointVector& points) {
    auto start = std::chrono::high_resolution_clock::now();

    tree_->Build(points);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    recordUpdateTime(duration.count() / 1000.0);

    // Update stats
    stats_.total_points = tree_->size();
    stats_.valid_points = tree_->validnum();

    RCLCPP_DEBUG(node_->get_logger(),
                 "Built tree with %zu points in %.2f ms",
                 points.size(), duration.count() / 1000.0);
}

void IKDTreeWrapper::addPoints(const PointVector& points, bool enable_downsample) {
    auto start = std::chrono::high_resolution_clock::now();

    int added = tree_->Add_Points(
        const_cast<PointVector&>(points),  // ikd-Tree API quirk
        enable_downsample
    );

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    recordUpdateTime(duration.count() / 1000.0);

    // Update stats
    stats_.total_points = tree_->size();
    stats_.valid_points = tree_->validnum();

    RCLCPP_DEBUG(node_->get_logger(),
                 "Added %d points in %.2f ms",
                 added, duration.count() / 1000.0);
}

void IKDTreeWrapper::nearestSearch(const PointType& point, int k,
                                   PointVector& nearest_points,
                                   std::vector<float>& distances,
                                   float max_dist) {
    auto start = std::chrono::high_resolution_clock::now();

    tree_->Nearest_Search(point, k, nearest_points, distances, max_dist);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    recordQueryTime(duration.count() / 1000.0);

    stats_.last_query_time_ms = duration.count() / 1000.0;
}

void IKDTreeWrapper::profilerLoop() {
    while (!stop_profiler_) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        if (!initialized_) continue;

        // Update rebuild status
        // Note: This requires exposing rebuild flag from ikd-Tree
        // stats_.rebuild_in_progress = tree_->isRebuilding();
    }
}

void IKDTreeWrapper::recordQueryTime(double ms) {
    std::lock_guard lock(profiling_mutex_);
    query_times_.push(ms);
    if (query_times_.size() > MAX_PROFILING_SAMPLES) {
        query_times_.pop();
    }
}

void IKDTreeWrapper::recordUpdateTime(double ms) {
    std::lock_guard lock(profiling_mutex_);
    update_times_.push(ms);
    if (update_times_.size() > MAX_PROFILING_SAMPLES) {
        update_times_.pop();
    }
}

double IKDTreeWrapper::getAverageQueryTime() const {
    std::lock_guard lock(profiling_mutex_);
    if (query_times_.empty()) return 0.0;

    double sum = 0.0;
    auto temp = query_times_;
    while (!temp.empty()) {
        sum += temp.front();
        temp.pop();
    }
    return sum / query_times_.size();
}

double IKDTreeWrapper::getAverageUpdateTime() const {
    std::lock_guard lock(profiling_mutex_);
    if (update_times_.empty()) return 0.0;

    double sum = 0.0;
    auto temp = update_times_;
    while (!temp.empty()) {
        sum += temp.front();
        temp.pop();
    }
    return sum / update_times_.size();
}

void IKDTreeWrapper::publishDiagnostics() {
    if (!node_ || !debug_pub_) return;

    RCLCPP_DEBUG(node_->get_logger(),
                 "ikd-Tree stats: total=%zu, valid=%zu, "
                 "avg_query=%.2fms, avg_update=%.2fms",
                 stats_.total_points.load(),
                 stats_.valid_points.load(),
                 getAverageQueryTime(),
                 getAverageUpdateTime());
}

std::unique_ptr<IKDTreeWrapper> createIKDTreeWrapper(const IKDTreeConfig& config) {
    return std::make_unique<IKDTreeWrapper>(config);
}

} // namespace uav_navigation
```

### 8.3 CMakeLists.txt Integration

```cmake
cmake_minimum_required(VERSION 3.8)
project(uav_ikd_tree)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# C++17 required for std::shared_mutex
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

# Original ikd-Tree implementation
add_library(ikd_tree_core STATIC
  third_party/ikd-Tree/ikd_Tree.cpp
)
target_include_directories(ikd_tree_core PUBLIC
  third_party/ikd-Tree
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(ikd_tree_core
  ${PCL_LIBRARIES}
  pthread
)

# ROS2 wrapper
add_library(${PROJECT_NAME} SHARED
  src/ikd_tree_wrapper.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}
  ikd_tree_core
  ${PCL_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

# Tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ikd_tree_wrapper test/test_wrapper.cpp)
  target_link_libraries(test_ikd_tree_wrapper ${PROJECT_NAME})
endif()

ament_package()
```

---

## 9. Integration với LidarProcessor

### 9.1 Modified `lidar_processor.hpp`

```cpp
#pragma once
#include "commons.h"
#include "ieskf.h"
#include "ikd_tree_wrapper.hpp"  // NEW: Use wrapper instead of raw ikd-Tree
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

class LidarProcessor {
public:
    LidarProcessor(Config &config, std::shared_ptr<IESKF> kf,
                   rclcpp::Node::SharedPtr node);  // NEW: Add node

    // ... existing methods ...

    // NEW: Get wrapper for diagnostics
    std::shared_ptr<IKDTreeWrapper> getIKDTree() { return m_ikdtree; }

private:
    Config m_config;
    LocalMap m_local_map;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<IKDTreeWrapper> m_ikdtree;  // CHANGED: Use wrapper
    // ... rest unchanged ...
};
```

### 9.2 Modified `lidar_processor.cpp` (Key Changes)

```cpp
// Constructor changes
LidarProcessor::LidarProcessor(Config &config, std::shared_ptr<IESKF> kf,
                               rclcpp::Node::SharedPtr node)
    : m_config(config), m_kf(kf)
{
    // Create wrapper with UAV-optimized config
    IKDTreeConfig ikd_config;
    ikd_config.delete_criterion_param = 0.4f;   // More aggressive
    ikd_config.balance_criterion_param = 0.6f;  // Tighter balance
    ikd_config.downsample_size = m_config.map_resolution;
    ikd_config.enable_profiling = true;

    m_ikdtree = createIKDTreeWrapper(ikd_config);
    m_ikdtree->initialize(node);

    // ... rest of initialization ...
}

// initCloudMap uses wrapper
void LidarProcessor::initCloudMap(PointVec &point_vec) {
    m_ikdtree->build(point_vec);  // Wrapper handles timing/stats
}

// incrCloudMap uses wrapper
void LidarProcessor::incrCloudMap() {
    // ... point processing logic ...
    m_ikdtree->addPoints(point_to_add, true);
    m_ikdtree->addPoints(point_no_need_downsample, false);
}

// trimCloudMap uses wrapper
void LidarProcessor::trimCloudMap() {
    // ... local map boundary logic ...
    PointVec points_history;
    m_ikdtree->acquire_removed_points(points_history);  // May need wrapper method

    if (!m_local_map.cub_to_rm.empty()) {
        m_ikdtree->deletePointBoxes(m_local_map.cub_to_rm);
    }
}

// updateLossFunc uses wrapper for nearest search
void LidarProcessor::updateLossFunc(State &state, SharedState &share_data) {
    // ... for each point ...
    std::vector<float> point_sq_dist(m_config.near_search_num);
    auto &points_near = m_nearest_points[i];

    // Wrapper provides timing and stats
    m_ikdtree->nearestSearch(point_world, m_config.near_search_num,
                             points_near, point_sq_dist);
    // ... rest unchanged ...
}
```

---

## 10. Performance Targets & Validation

### 10.1 Benchmark Criteria

| Metric | Target | Acceptable | Measurement Method |
|--------|--------|------------|-------------------|
| **Frame processing time** | <30ms | <50ms | `std::chrono` in timerCB |
| **IKDTree query latency** | <0.1ms | <0.5ms | Wrapper profiling |
| **IKDTree update latency** | <2ms | <5ms | Wrapper profiling |
| **Memory usage** | <500MB | <1GB | `/proc/self/status` |
| **Rebuild frequency** | <1Hz | <5Hz | Stats counter |
| **UAV pose jitter** | <1cm | <5cm | Ground truth comparison |

### 10.2 Validation Test Suite

```cpp
// test_ikd_tree_performance.cpp
TEST(IKDTreePerformance, Mid360Workload) {
    auto wrapper = createIKDTreeWrapper();

    // Simulate MID-360: 10,000 points @ 10Hz
    const size_t NUM_POINTS = 10000;
    const size_t NUM_FRAMES = 100;

    PointVector initial_cloud = generateRandomPoints(NUM_POINTS);
    wrapper->build(initial_cloud);

    std::vector<double> query_times;
    std::vector<double> update_times;

    for (size_t frame = 0; frame < NUM_FRAMES; ++frame) {
        // Simulate new scan points
        PointVector new_points = generateRandomPoints(NUM_POINTS / 10);

        auto t1 = std::chrono::high_resolution_clock::now();
        wrapper->addPoints(new_points, true);
        auto t2 = std::chrono::high_resolution_clock::now();

        // Simulate nearest neighbor queries
        PointVector result;
        std::vector<float> dists;
        auto t3 = std::chrono::high_resolution_clock::now();
        wrapper->nearestSearch(new_points[0], 5, result, dists);
        auto t4 = std::chrono::high_resolution_clock::now();

        update_times.push_back(
            std::chrono::duration<double, std::milli>(t2 - t1).count());
        query_times.push_back(
            std::chrono::duration<double, std::milli>(t4 - t3).count());
    }

    // Validate targets
    double avg_update = std::accumulate(update_times.begin(),
                                        update_times.end(), 0.0) / update_times.size();
    double avg_query = std::accumulate(query_times.begin(),
                                       query_times.end(), 0.0) / query_times.size();

    EXPECT_LT(avg_update, 2.0);  // < 2ms update
    EXPECT_LT(avg_query, 0.1);   // < 0.1ms query
}
```

---

## 11. Conclusion & Recommendations

### 11.1 Kiến trúc Được Đề xuất

```
Recommended UAV Navigation Stack:

┌─────────────────────────────────────────────────────────────┐
│                    LIONode (ROS2)                           │
│  ┌───────────────────────────────────────────────────────┐  │
│  │              MapBuilder (Thread-safe)                  │  │
│  │  ┌─────────────────────────────────────────────────┐   │  │
│  │  │         IKDTreeWrapper (RAII + Stats)         │   │  │
│  │  │  ┌───────────────────────────────────────┐    │   │  │
│  │  │  │      ikd-Tree (Original HKU)        │    │   │  │
│  │  │  │  • pthread rebuild thread             │    │   │  │
│  │  │  │  • Lazy delete/add                  │    │   │  │
│  │  │  │  • Lock-free counters                 │    │   │  │
│  │  │  └───────────────────────────────────────┘    │   │  │
│  │  │                                                 │   │  │
│  │  │  Features:                                      │   │  │
│  │  │  • ROS2 lifecycle                             │   │  │
│  │  │  • Real-time profiling                        │   │  │
│  │  │  • Memory pool (future)                       │   │  │
│  │  │  • std::thread migration (future)           │   │  │
│  │  └─────────────────────────────────────────────────┘   │  │
│  │                                                         │  │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐   │  │
│  │  │ IMUProcessor│  │LidarProcessor│  │   IESKF     │   │  │
│  │  └─────────────┘  └─────────────┘  └─────────────┘   │  │
│  └───────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────┘
```

### 11.2 Priority Action Items

| Priority | Task | Effort | Impact |
|----------|------|--------|--------|
| **P0** | Create IKDTreeWrapper class | 2 days | Critical - RAII + ROS2 integration |
| **P0** | Integrate with LidarProcessor | 1 day | Critical - Core functionality |
| **P1** | Add profiling hooks | 1 day | High - Debugging & optimization |
| **P1** | Create performance benchmarks | 2 days | High - Validate 20Hz target |
| **P2** | Migrate pthread → std::thread | 3 days | Medium - Code quality |
| **P2** | Implement memory pool | 3 days | Medium - Embedded optimization |
| **P3** | ARM NEON optimizations | 5 days | Low - Performance boost |

### 11.3 Risk Mitigation

| Risk | Likelihood | Mitigation |
|------|------------|------------|
| Lock contention in tight loop | Medium | Profile with `perf`, optimize mutex hierarchy |
| Memory fragmentation | Low | Pre-allocate point vectors, use memory pool |
| Rebuild starvation | Low | Monitor rebuild queue, add backpressure |
| PCL compatibility issues | Low | Maintain template interface, test with MID-360 |

---

## Appendix: References

1. **Original ikd-Tree Paper**: Cai et al., "ikd-Tree: An Incremental K-D Tree for Robotic Applications", 2021
2. **FAST-LIO2 Paper**: Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry", IEEE T-RO 2022
3. **HKU Implementation**: https://github.com/hku-mars/ikd-Tree
4. **FAST-LIO2 ROS2**: https://github.com/hku-mars/FAST_LIO

---

*Document generated for UAV Navigation Project - MID-360 Integration*
