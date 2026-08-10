# Third-party notices and distribution boundary

This repository vendors GPL-2.0-only source code and therefore must not be
represented as an MIT-only or Apache-2.0-only distributable work. Before any
binary/source distribution, obtain a legal review of the combined-work and
linking obligations, provide corresponding source and notices as required, and
ensure package metadata and release artifacts reflect the review. This file is
an engineering notice, not legal advice.

| Component | Use in this repository | License | Provenance / notice |
| --- | --- | --- | --- |
| IKFoM | Vendored headers used by estimator integration | GPL-2.0-only | [UPSTREAM.md](src/navigation_estimator/ikfom_vendor/UPSTREAM.md); full text at `ikfom_vendor/LICENSES/IKFoM-GPL-2.0.txt` |
| ikd-Tree | Vendored source used by registration-map integration and benchmark | GPL-2.0-only | [UPSTREAM.md](src/navigation_estimator/ikd_tree_vendor/UPSTREAM.md); full text at `ikd_tree_vendor/LICENSES/ikd-Tree-GPL-2.0.txt` |
| FAST-LIO | Algorithmic reference only; no source copied | GPL-2.0-only upstream | `ikfom_vendor/UPSTREAM.md` |
| Livox ROS Driver 2 | Hardware-driver/dataset contract reference; no source copied | MIT upstream | [official license](https://github.com/Livox-SDK/livox_ros_driver2/blob/master/LICENSE.txt) |
| ROS 2 Jazzy / Gazebo Harmonic | Runtime platform dependencies; no source copied | per upstream packages | distribution package notices apply |
| Eigen / PCL | Build dependencies; no source copied | per upstream packages | distribution package notices apply |

The root `LICENSE` applies only to project-owned material to the extent it is
legally separable from the GPL-licensed vendored/integrated work. Individual ROS
package `<license>` declarations are not a substitute for the required release
licensing analysis.
## ROG-Map reference (not imported)

The ROG-Map reference repository was inspected at
`https://github.com/hku-mars/ROG-Map`, main SHA
`df59c21304579a13fb3875100f8ce9523ba379a0`. Its `LICENSE` is GPL-3.0.
No source from ROG-Map or SUPER is distributed in this repository. The P1
`rog_map_core` implementation is original Apache-2.0 project code because
importing GPL-3.0 source into this Apache-2.0 workspace was not compatible.
