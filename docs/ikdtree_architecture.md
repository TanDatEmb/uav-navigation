# ikd-Tree Spatial Index Architecture

## Overview

The `fast_lio` mapping subsystem uses an incremental k-d tree (ikd-Tree) as
its active spatial index for LiDAR scan matching and map updates. An abstract
interface separates the LIO algorithm from the concrete backend.

## Component Ownership and Data Flow

```
MapBuilder (owns)
├── IESKF (shared state estimate)
├── IMUProcessor (IMU integration)
├── LidarProcessor (point cloud processing)
│   └── MapTreeInterface (shared pointer)
│       └── IKDTreeBackend (or PCLTreeBackend fallback)
```

The `MapBuilder` constructs the tree via `createMapTree()`, which instantiates `IKDTreeBackend` by default. The `LidarProcessor` queries the tree for nearest neighbors during scan matching and incrementally updates it with new world-frame points.

## MapTreeInterface Contract

The abstract interface defines the minimal surface for LIO operations:

- **Point addition**: `addPoints(PointVec, bool downsample)` — incremental insertion with optional voxel downsampling
- **Point deletion**: `deletePoints(vector<BoxPointType>)` — axis-aligned box deletion for local map management
- **Nearest search**: `nearestKSearch()` returns indices; `nearestKSearchPoints()` returns actual points (ikd-Tree does not expose stable indices)
- **Tree management**: `build()`, `rebuild()`, `clear()`
- **State queries**: `size()`, `empty()`, `valid()`, `rebuilding()`
- **Configuration**: `setDownsampleParam()`, `setLocalMapRange()`

Factory methods create implementations:

- `createIKDTree()` — incremental ikd-Tree backend
- `createPCLTree()` — non-incremental PCL KD-tree backend

`MapBuilder::createMapTree()` currently selects ikd-Tree unconditionally; there
is no ROS parameter that switches to the PCL backend.

## IKDTreeBackend Implementation

`IKDTreeBackend` wraps the bundled HKU ikd-Tree implementation with:

- an external `std::shared_mutex` around selected wrapper operations
- wrapper defaults of `0.4` deletion criterion, `0.6` balance criterion, and
  `0.15 m` downsampling

`MapBuilder` immediately overrides the downsampling size with the active
`map_resolution` parameter (`0.5 m` in the canonical YAML).

### Incremental Operations

**Add:** `Add_Points()` optionally performs voxel downsampling. For each voxel,
it retains the existing or incoming point closest to the voxel center; it does
not compute a centroid.

**Delete**: `Delete_Point_Boxes()` marks points within axis-aligned boxes as deleted (lazy deletion). Actual removal occurs during background rebuild.

**Search**: `Nearest_Search()` traverses the tree with a manual heap for k-best selection. Thread-safe against concurrent rebuild via `search_flag_mutex`.

## Threading and Locking Reality

The ikd-Tree uses internal pthread mutexes; `IKDTreeBackend` adds a `std::shared_mutex` for external synchronization:

- **Write operations** (add/delete/clear): Acquire unique_lock on external mutex; ikd-Tree handles internal locking
- **Read operations** (search): ikd-Tree acquires `search_flag_mutex` internally; external callers do not hold locks during search to avoid deadlock with the rebuild thread

**Background rebuild thread**: Automatically started on construction. Triggers when:
- Tree size exceeds threshold with imbalance ratio > `balance_criterion_param` (0.6)
- Invalid/deleted points ratio exceeds `delete_criterion_param` (0.4)

During rebuild:
1. The subtree is flattened to a point vector
2. A new balanced tree is constructed
3. Operations logged during rebuild are replayed
4. The old subtree is atomically replaced

## Local Map Handling

`LidarProcessor` maintains a sliding `LocalMap` to bound memory and search time:

- **Initialization**: First scan defines the cube center; `build()` populates the tree
- **Sliding update**: `LocalMap::update()` checks if the UAV moved beyond `move_thresh * det_range` of cube edges
- **Slab deletion**: When the cube moves, only the non-overlapping slabs are queued in `boxes_to_remove` and deleted from the tree
- **Configuration**: `cube_len` defines edge length; `move_thresh` (typically 0.5) controls sensitivity

## Fallback Status

- **PCLTreeBackend:** compiled as an alternative implementation, but not selected
  by the current `MapBuilder`. It rebuilds the PCL index after additions or
  deletions.
- **Validity checks:** `valid()` checks `Root_Node != nullptr`; `size()` falls
  back to total node count if `validnum()` returns `-1` during rebuild.

## Configuration

| Setting | Active value | Source | Description |
|---------|--------------|--------|-------------|
| deletion criterion | 0.4 | wrapper constructor | Rebuild trigger for deleted-point ratio |
| balance criterion | 0.6 | wrapper constructor | Rebuild trigger for tree imbalance |
| `map_resolution` | 0.5 m | canonical ROS YAML | ikd-Tree voxel downsampling size |
| `cube_len` | 50.0 m | canonical ROS YAML | Local map cube edge length |
| `det_range` | 40.0 m | canonical ROS YAML | Detection range used by the move threshold |
| `move_thresh` | 0.5 | canonical ROS YAML | Ratio used to decide when the cube moves |
| `near_search_num` | 5 | canonical ROS YAML | Neighbors requested for plane fitting |

The wrapper constructor also accepts `rebuild_threshold=1500`, but does not pass
it into the bundled tree. The implementation uses its compile-time
`Multi_Thread_Rebuild_Point_Num` value of 1500.

## Build Configuration

CMakeLists.txt selects the backend and dependencies:
- Core library: `ikd_tree.cpp`, `ikd_tree_backend.cpp`, `pcl_tree_backend.cpp`
- Link: `pthread` (required for ikd-Tree)
- Optional: `OpenMP` for parallel processing (`MP_EN`)
- Compile flags: `-O3`, `-Wall -Wextra -Wpedantic`

## Tests

`test_ieskf.cpp` validates:
- **IKDTreeBackendTest**: Incremental addition, deletion, and `size()`/`getAllPoints()` consistency
- **LocalMapTest**: Sliding cube removes only departed slabs
- **LidarProcessorTest**: World-frame transform and map population

## Known Limitations

- **Rebuild detection**: `rebuilding()` always returns false; ikd-Tree does not expose background thread status through the wrapper
- **Local map range**: `setLocalMapRange()` is a no-op; callers must use `deletePoints()` directly
- **Rebuild threshold:** the constructor accepts but does not apply
  `rebuild_threshold`; the bundled tree uses its compile-time constant.
- **Point indices:** `nearestKSearch()` returns synthetic sequential indices
  because ikd-Tree does not expose stable point IDs.
- **Operation counts:** wrapper `addPoints()` returns the input count, and
  `deletePoints()` returns the box count rather than the number of points changed.
- **Thread safety:** search relies on ikd-Tree internal locking; the wrapper does
  not hold its external mutex during search to avoid deadlock with rebuild.
