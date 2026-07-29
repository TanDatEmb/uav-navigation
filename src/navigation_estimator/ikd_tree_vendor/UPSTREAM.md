# ikd-Tree provenance

- Upstream repository: <https://github.com/hku-mars/ikd-Tree>
- Pinned commit: `c0e36a16b6e4d557d3783b16911207f6398dd478`
- Commit verification: `git ls-remote https://github.com/hku-mars/ikd-Tree.git HEAD`
  on 2026-07-28 returned this object ID.
- Integration date: 2026-07-28.
- Upstream license: GNU GPL version 2.
- Vendored files used:
  - `ikd-Tree/ikd_Tree.h`
  - `ikd-Tree/ikd_Tree.cpp`
  - upstream `LICENSE` and `README.md`
- Excluded content: examples, papers, images, Git metadata and build files.
- Source differences: see `PATCHES.md`. The project adds an opt-in
  asynchronous-rebuild constructor switch and minimal rebuild replacement
  safety fixes while retaining the pinned upstream algorithms and API shape.
- Build behavior: compiles the actual upstream implementation, exports
  `ikd_tree_vendor::ikd_tree_vendor`, runs an API smoke test, and fails CMake
  configuration if the pinned source is absent.
