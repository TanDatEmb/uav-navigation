# Estimator upstream provenance

- Upstream repository: <https://github.com/hku-mars/IKFoM>
- Pinned commit: `59cfc095ca74425f9b330c7c04a5d74f68c6dd62`
- Commit verification: `git ls-remote https://github.com/hku-mars/IKFoM.git HEAD`
  on 2026-07-28 returned this object ID.
- Integration date: 2026-07-28.
- Upstream license: GNU GPL version 2, as declared by upstream `LICENSE`.
- Files used from upstream: none in this revision.
- Differences from upstream: this package intentionally does not vendor, patch,
  compile, or download IKFoM source. It exports the explicit
  `ikfom_vendor::ikfom_vendor` interface target and provenance only.

Before any IKFoM header or source is introduced, copy the applicable upstream
license text into `LICENSES/`, record every imported file here, and record each
local patch in `PATCHES.md`. Do not replace the pinned commit with a branch name.

## FAST-LIO algorithm reference

- Upstream repository: <https://github.com/hku-mars/FAST_LIO>
- Pinned commit: `7cc4175de6f8ba2edf34bab02a42195b141027e9`
- Commit verification: `git ls-remote https://github.com/hku-mars/FAST_LIO.git HEAD`
  on 2026-07-28 returned this object ID.
- Upstream license: GNU GPL version 2, as declared by upstream `LICENSE`.
- Files used from upstream: none in this revision.

This entry supplies algorithmic traceability only. It does not make this vendor
package a FAST-LIO source distribution; any future source import requires its
own file inventory, preserved license text, and patch record.
