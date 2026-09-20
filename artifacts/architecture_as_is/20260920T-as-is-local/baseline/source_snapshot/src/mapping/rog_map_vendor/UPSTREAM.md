# Upstream provenance: ROG-Map

## Source

- Upstream repository: https://github.com/hku-mars/SUPER
- Pinned commit: `2ad3419c127a617c6d7df6925e81a14175a9c096` (upstream `master` HEAD
  at the time of import; also matches the `uav-navigation` HEAD used to start
  this integration).
- Imported subtree: `rog_map/` (upstream package root). No other SUPER
  package (`navigation_planning_backend/`, `mission_planner/`, `mars_uav_sim/`) is imported.

## Imported files

Copied verbatim from upstream `rog_map/`, license headers preserved:

```
include/rog_map/rog_map.h
include/rog_map/prob_map.h
include/rog_map/inf_map.h
include/rog_map/esdf_map.h
include/rog_map/free_cnt_map.h
include/rog_map/rog_map_core/common_lib.hpp
include/rog_map/rog_map_core/config.hpp
include/rog_map/rog_map_core/counter_map.h
include/rog_map/rog_map_core/raycaster.h
include/rog_map/rog_map_core/sliding_map.h
include/navigation_math/eigen_alias.hpp
include/navigation_math/fmt_eigen.hpp
include/navigation_math/scope_timer.hpp
include/navigation_math/type_utils.hpp
include/navigation_math/yaml_loader.hpp
include/navigation_math/color_text.hpp
src/rog_map/counter_map.cpp
src/rog_map/esdf_map.cpp
src/rog_map/inf_map.cpp
src/rog_map/prob_map.cpp
src/rog_map/rog_map.cpp
src/rog_map/sliding_map.cpp
```

`include/rog_map/rog_map_core/raycaster.cpp` is upstream source relocated to
`src/rog_map_core/raycaster.cpp` in this vendor package purely for build
layout consistency (headers in `include/`, translation units in `src/`); its
contents are otherwise unmodified.

## Deliberately excluded

- `include/rog_map_ros/rog_map_ros1.hpp`, `include/rog_map_ros/rog_map_ros2.hpp`,
  `src/rog_map_ros/rog_map_ros1.cpp`: upstream's own ROS1/ROS2 automatic
  subscription/callback wrapper. P1 requires exactly one product-owned
  subscription path into ROG (`navigation_mapping::NavigationMappingNode`)
  using the manual `ROGMap::updateMap()` API, so the upstream wrapper is not
  vendored and must never become the production integration boundary.
- `include/navigation_math/backward.hpp` (stack-trace pretty-printer) and
  `include/navigation_math/tinycolormap.hpp` (visualization colormap table):
  neither is included by any file under `include/rog_map/` or `src/rog_map/`
  (verified by `grep`), so they fall outside the minimum required
  dependency closure.
- `include/navigation_math/color_msg_utils.hpp`: only used by the excluded ROS
  wrappers.
- `include/fmt/*` (bundled fmt 9.x source, ~13 headers): replaced by the
  system `libfmt-dev` package (see "Local modifications").
- ROS1 `CMakeLists.txt`/`package.xml`, `ros/ros1.*`, `ros/ros2.*`, and
  `config/visualization.cfg`: build/integration files, superseded by this
  package's own `CMakeLists.txt`/`package.xml`.
- `log/plot_performance_log.py`: offline plotting helper, not required to run
  ROG-Map; not part of the runtime dependency closure.

## Local modifications

All modifications are scoped to the smallest possible diff and are
individually justified below.

### 0. Aggregate ray diagnostics without hash-set hot-path state (`src/rog_map/prob_map.cpp`, `include/rog_map/prob_map.h`)

The product diagnostics layer previously kept per-update `std::unordered_set`
instances for unique hit/miss voxel counts. Those sets added hashing and
allocation work to every ray candidate. Unique counts now use the existing
`operation_cnt` and `hit_cnt` arrays while preserving the ray/update-cache
operations and map update semantics.

### 1. Lifecycle fix: per-instance init guard (`src/rog_map/prob_map.cpp`, `include/rog_map/prob_map.h`)

**Problem.** `ProbMap::initProbMap()` guarded double-initialization with a
function-local `static bool init_once`. A function-local `static` is
process-wide, not per-instance: the *first* `ROGMap`/`ProbMap` object
constructed in a process sets it `true` forever, so constructing a *second*
`ROGMap` instance later in the same process — even after the first one is
fully destroyed — throws `std::runtime_error("ProbMap can only init once.")`.

This directly blocks the P1 requirement that a public-frame-generation
discontinuity trigger a complete map reset that continues operating in the
same mapper process (see `docs/architecture/navigation_layers.md` and
P1 acceptance criterion "public-frame generation changes reset map
correctly").

**Fix.** Replaced the function-local `static bool init_once` with a regular
protected member `bool initialized_once_{false}` on `ProbMap`. Each new
`ProbMap`/`ROGMap` instance gets its own guard, initialized to `false`, so
repeated destroy-then-reconstruct cycles work. The original single-init
safety check for *one* instance is preserved unchanged (still throws if the
same instance's `initProbMap()` is called twice).

**Regression test.** `test/test_rog_map_lifecycle.cpp` constructs, initializes,
destroys, and reconstructs multiple `ROGMap`-derived instances in the same
process and asserts each succeeds and produces a usable map.

### 2. Build/ROS-integration layer replaced for ROS 2 Jazzy/ament (`CMakeLists.txt`, `package.xml`)

Upstream ships a catkin (`ros/ros1.CMakeLists.txt`) and an ament
(`ros/ros2.CMakeLists.txt`) build file that both recursively glob
`include/*.h`, `include/*.hpp`, `include/*.cpp`, and `src/*.cpp`, hard-code
`CMAKE_BUILD_TYPE Release`, and set `CMAKE_CXX_FLAGS` globally. This
repository's conventions forbid recursive globbing, global compiler-flag
overwrites, and forcing a repository-wide `Release` build. This package's
`CMakeLists.txt` is a from-scratch `ament_cmake` file that:

- lists every compiled source file explicitly;
- does not set `CMAKE_BUILD_TYPE` or global `CMAKE_CXX_FLAGS`;
- applies `-Wall` only to the `rog_map_vendor` target, not globally;
- uses the top-level project's C++ standard (20) instead of upstream's C++17.

This is a build/integration-layer replacement only; no algorithmic file
content is affected by this change.

### 3. System `fmt`/`yaml-cpp` instead of a bundled copy (`CMakeLists.txt`)

Upstream vendors its own copy of the `fmt` library (~13 headers, ~14k lines)
under `include/fmt/` purely to provide `fmt::print`/`fmt::format` used in 3
files (`rog_map.h`, `rog_map.cpp`, `fmt_eigen.hpp`) and links against
`yaml-cpp` for `Config`'s YAML loader. Ubuntu 24.04 ships first-class CMake
config packages for both (`libfmt-dev` 9.1, `libyaml-cpp-dev` 0.8). Vendoring
a second, divergent copy of `fmt` would risk ODR/version conflicts with any
other package in this workspace that also links system `fmt` or `spdlog`, and
adds ~14k lines of unrelated third-party source to this repository for no
functional benefit. This package therefore depends on the system packages
instead of importing `include/fmt/`. No ROG-Map source file needed any
change to build against system `fmt`/`yaml-cpp` (its use is limited to the
standard `fmt::print`, `fmt::format`, `fmt::color` API surface).

### 4. `raycaster.cpp` include path fixup (`src/rog_map_core/raycaster.cpp`)

Upstream's `raycaster.cpp` lives next to `raycaster.h` and includes it with a
bare, same-directory `#include "raycaster.h"`. This vendor package moves the
`.cpp` under `src/` while the header stays under `include/rog_map/rog_map_core/`
(headers-in-`include/`, translation-units-in-`src/` layout), so the include was
rewritten to `#include <rog_map/rog_map_core/raycaster.h>`. No other line in
the file changed.

### 5. Removed unused `pcl_conversions` include (`include/rog_map/rog_map_core/common_lib.hpp`)

`common_lib.hpp` included `<pcl_conversions/pcl_conversions.h>` but no
vendored source file (after excluding the ROS wrapper, which was the only
caller of `pcl::fromROSMsg`/`toROSMsg`) actually calls any `pcl_conversions`
function; verified with a repository-wide `grep` for `pcl::fromROSMsg`,
`pcl::toROSMsg`, and `pcl_conversions::` across `include/` and `src/`, which
returned no matches. `pcl_conversions` is a ROS package whose own CMake
config depends on `rclcpp`, and linking against it (even only for its include
path) pulled in `rclcpp`'s include tree without its own transitive
`rcl_interfaces` include path resolved correctly in this build, breaking the
build for every consumer of this package for no functional benefit. The
unused include line was removed and the `pcl_conversions` build/exec
dependency was dropped entirely from `CMakeLists.txt`/`package.xml`. This is
the only line removed from any vendored file; no ROG-Map behavior is
affected.

### 6. Restored `pcl/io/pcd_io.h` include (`src/rog_map/rog_map.cpp`)

`ROGMap::init()`'s optional `load_pcd_en` path (P1: disabled) calls
`pcl::io::loadPCDFile`. That declaration was previously reached transitively
through the now-removed `pcl_conversions.h` include (modification 5). This
file now includes `<pcl/io/pcd_io.h>` directly instead, restoring the
declaration without reintroducing the `pcl_conversions` dependency.

### 7. Disabled research timing output in product runtime (`src/rog_map/prob_map.cpp`, `src/rog_map/rog_map.cpp`)

The product already records aggregate ROG timing in `MappingDiagnostics` and
offline reports. The upstream `ROGMap::updateMap()` console timer and
per-update `rm_performance_log.csv` write therefore provide no product
semantic value and add work to every accepted observation. The product path
keeps the upstream timing code and `writeTimeConsumingToLog()` available for
comparison, but does not enable or invoke those outputs.

- `ProbMap::raycastProcess` (`src/rog_map/prob_map.cpp`) originally used a
  function-local `static bool first` to clear unknown cells around the very
  first robot position. Measurement/lifecycle testing showed that this was
  process-wide and therefore skipped the bootstrap after a map-generation
  reset. It is now an instance member reset by `resetLocalMap()`, so each map
  generation receives the same bootstrap behavior.
- `ROGMap::updateMap` (`src/rog_map/rog_map.cpp`) uses a function-local
  `static int local_cnt` purely to throttle an empty-cloud warning log line.
  Also process-wide, also non-blocking, also left unmodified.
- `ROGMap::init()` opens `log/rm_info_log.csv` relative to `ROOT_DIR` (this
  package's source directory at build time) via `std::ofstream::open`. The
  product performance stream is deliberately not opened; the remaining info
  log is upstream debug behavior, not part of the ROG-Map algorithm. If the
  path does not exist at runtime, the stream fails silently and cannot crash
  the mapper process.

### 8. Exposed the ROG config by const reference (`include/rog_map/rog_map.h`)

`ROGMap::getMapConfig()` now returns `const rog_map::Config&` instead of a
full `Config` copy. `Config` owns large precomputed neighbor lattices, so a
value-returning accessor made every product bounds query copy those vectors.
The product `WorldModel` also avoids calling bounds from its per-cell query
path.

### 9. Exposed coarse inflated-cell knowledge (`include/rog_map/inf_map.h`, `include/rog_map/prob_map.h`, `src/rog_map/inf_map.cpp`, `src/rog_map/prob_map.cpp`)

`WorldModel::Inflated` needs both ROG's inflated occupancy/unknown counters
and the underlying `CounterMap` aggregate for the represented coarse cell.
The narrow `InfMap::getBaseGridType()` query reuses the existing protected
`CounterMap::getGridType()` logic without duplicating its thresholds. The
`ProbMap` forwarding accessor keeps this vendor detail out of the product
facade. It prevents a coarse inflated cell from being inferred from one
probability voxel at its center; no mapping update or threshold semantics
were changed.

### 10. Integer ray traversal API (`src/rog_map_core/raycaster.cpp`, `include/rog_map/rog_map_core/raycaster.h`, `src/rog_map/prob_map.cpp`)

**Upstream behavior.** `RayCaster::step(Eigen::Vector3d&)` advances the DDA in
integer voxel state, converts the current index to a metric voxel center, and
the probability-map raycast path converts that metric center back to a global
voxel index.

**Local modification.** Added the narrow `RayCaster::stepIndex(Vec3i&)` API.
It emits the existing integer state directly; the legacy metric `step()` API
remains available and delegates to the same traversal logic. The production
probability-map free-ray loop now uses `stepIndex()` and no longer performs the
index-to-center-to-index round trip.

**Reason.** Remove avoidable metric Eigen conversion work from the ROG ray
traversal inner loop while limiting the vendor patch to an API addition.

**Semantic effect.** None intended: tie-breaking, traversal state, termination,
and voxel order are unchanged. The index API is covered by an exact ordered
sequence comparison against the legacy API over axis, diagonal, signed,
boundary, same-voxel, and deterministic random rays.

### Deterministic-origin patch

`SlidingMap::initSlidingMap` now initializes the local origin before the first
sliding-enabled `mapSliding()` call. The upstream order left the origin index
uninitialized and made fresh-map state nondeterministic; this was found by the
deterministic digest benchmark.

### 11. Detached planning-grid export

Added `ROGMap::exportPlanningGrid()` and the corresponding inflated-layer
export. The mapping owner converts circular mutable probability/counter
storage into compact semantic byte arrays in logical global-index order. Each
returned value owns its dynamic cell arrays. The configured nearest-neighbor
table is copied once after initialization into a separately owned immutable
allocation and safely shared by exports; it never aliases mutable config.
The export exposes neither probability values nor mutable buffers.

Export precomputes the three circular-index axes once and then performs
cache-local scalar traversal. This preserves logical output order and exact
public cell semantics while avoiding millions of repeated modulo/index
conversions per observation. This is representation-only: mapping state,
thresholds and query results are unchanged.

This is the staging boundary for a genuinely immutable product WorldSnapshot.
It does not change mapping updates or query thresholds, and it is never called
by SUPER. Exhaustive fixture tests compare exported base/inflated cells with
the live map away from separately represented virtual planes and prove an
earlier export does not alias a later update.

### 12. Explicit no-return visibility rays (`include/rog_map/prob_map.h`, `include/rog_map/rog_map.h`, `src/rog_map/prob_map.cpp`, `src/rog_map/rog_map.cpp`)

The product API adds a separate optional point cloud for explicit no-return
endpoints. Unlike hit endpoints, these points are processed as miss-only rays:
the traversed voxels receive the existing miss update, while the endpoint is
never inserted as occupied. The original two-argument `updateMap` and
`updateProbMap` overloads delegate to an empty no-return cloud, preserving the
upstream call contract when no visibility source is available.

This distinction is required by the typed `RegisteredScan` contract. A point
cloud contains returns only and cannot prove that the unreturned space beyond
the last return is free. The optional field therefore remains empty unless a
sensor/bridge explicitly supplies no-return evidence with matching frame and
timestamp. UNKNOWN and OUT_OF_MAP semantics remain unchanged; no endpoint-only
or blanket-free fallback was added.

## License metadata inconsistency found upstream

Upstream `rog_map/package.xml` (ROS 2/ament) declares `<license>BSD</license>`,
while `rog_map/ros/ros1.package.xml` (ROS1/catkin) declares
`<license>TODO</license>` for the same source tree, and several source files'
own header comments state GNU LGPL/GPL terms (see the header block reproduced
verbatim at the top of each vendored file, e.g. `rog_map/rog_map_core/config.hpp`).
This repository does not resolve or adjudicate that inconsistency. This
package's `package.xml` declares `BSD` to match the ROS 2 package manifest we
imported from, but this is **not** a claim that the final licensing terms are
resolved or that BSD is authoritative over the per-file LGPL/GPL header text.
Anyone redistributing this package must consult upstream directly.
