# Third-party notices and distribution boundary

This repository contains project-owned MIT material, vendored source, a Git
submodule, and runtime dependencies. It must not be represented as an
MIT-only or Apache-2.0-only distributable work. Before any binary/source
distribution, obtain a legal review of the combined-work and linking
obligations, provide corresponding source and notices as required, and ensure
package metadata and release artifacts reflect the review. This file is an
engineering inventory, not legal advice.

| Component | Use in this repository | License | Provenance / notice |
| --- | --- | --- | --- |
| IKFoM | Vendored headers used by estimator integration | GPL-2.0-only upstream | [UPSTREAM.md](src/estimation/ikfom_vendor/UPSTREAM.md); license text at `src/estimation/ikfom_vendor/LICENSES/IKFoM-GPL-2.0.txt` |
| ikd-Tree | Vendored source used by registration-map integration and tests | GPL-2.0-only upstream | [UPSTREAM.md](src/estimation/ikd_tree_vendor/UPSTREAM.md); license text at `src/estimation/ikd_tree_vendor/LICENSES/ikd-Tree-GPL-2.0.txt` |
| ROG-Map / planner backend subset | Vendored navigation world-model source | Upstream metadata inconsistent: BSD package manifest and LGPL/GPL file headers | [UPSTREAM.md](src/mapping/rog_map_vendor/UPSTREAM.md); do not treat the package manifest alone as a resolved license determination |
| FAST-LIO | Algorithmic/reference provenance; no FAST-LIO source copied | GPL-2.0-only upstream | [IKFoM/FAST-LIO provenance](src/estimation/ikfom_vendor/UPSTREAM.md) |
| Livox ROS Driver 2 | Complete pinned driver and message package copied into `src/external/livox_ros_driver2` | MIT | [UPSTREAM.md](src/external/livox_ros_driver2/UPSTREAM.md) and local [LICENSE](src/external/livox_ros_driver2/LICENSE) |
| PX4 `px4_msgs` | Git submodule at `src/external/px4_msgs`, pinned to v1.17.0 | BSD 3-Clause | [submodule README](src/external/README.md) and `src/external/px4_msgs/LICENSE` |
| PX4 ROS 2 Interface Library | Git submodule at `src/external/px4_ros2_interface_lib`, `release/1.17` | BSD 3-Clause | [submodule README](src/external/README.md) and `src/external/px4_ros2_interface_lib/LICENSE` |
| Livox bundled RapidJSON | Third-party headers included inside the Livox package | MIT, with BSD/JSON notices for listed components | `src/external/livox_ros_driver2/3rdparty/rapidjson/license.txt` |
| ROS 2 Jazzy / Gazebo Harmonic | Runtime platform dependencies; no source copied | per upstream packages | distribution package notices apply |
| Eigen / PCL / yaml-cpp / fmt | Build dependencies; no source copied | per upstream packages | distribution package notices apply |

The root `LICENSE` applies to project-owned material only. It does not replace
the notices and license texts shipped with vendored or external components.
Individual ROS package `<license>` declarations are not a substitute for the
required release licensing analysis, especially for the ROG-Map/planner backend subset
whose upstream metadata is internally inconsistent.
