# Patches

The pinned upstream source has the following project-owned hardening patches:

- Upstream base: `c0e36a16b6e4d557d3783b16911207f6398dd478`.

- `KD_TREE` accepts an `enable_asynchronous_rebuild` constructor argument.
  Its default remains `true` for compatibility, and the production FAST-LIO
  wrapper explicitly keeps asynchronous rebuild enabled. Tests may select
  synchronous rebuild for deterministic fixtures.
- `Nearest_Search_Into()` is a project-owned allocation-free query API. It
  writes up to `k` neighbors and squared distances into caller-owned buffers
  and uses caller-owned `MANUAL_HEAP` storage. It retains the upstream
  search/rebuild synchronization path.
- `asynchronous_rebuild_in_progress()` is a non-blocking project-owned
  observability hook used to prove that concurrent-query tests overlap an
  active vendor rebuild rather than merely enabling async mode.
- The asynchronous replacement path handles an empty rebuilt tree without
  dereferencing a null `new_root_node`.
- The synchronous rebuild path records whether it is replacing the root before
  deleting the old nodes, then updates both root pointers after reconstruction.
- Rebuilding or destroying a tree releases the separately allocated static-root
  sentinel after detaching its already-released child tree.

Root cause addressed by these source changes: the synchronous root test was
performed after deleting/rebuilding the old root, so pointer identity could no
longer reliably identify a root replacement. The asynchronous replacement also
unconditionally dereferenced `new_root_node` after rebuilding an empty subtree.
The vendor smoke tests cover empty/synchronous replacement, repeated
construction/destruction, allocation-free async queries, and serial-equivalent
multi-reader `Nearest_Search_Into()` with async rebuild enabled. Project
registration-map tests cover stable signed counts and lifecycle behavior.
AddressSanitizer lifecycle coverage protects the static-root sentinel ownership
fix.

Package CMake and tests are project-owned integration files outside
`vendor/ikd-Tree`.
