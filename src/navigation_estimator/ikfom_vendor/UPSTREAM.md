# Estimator upstream provenance

- Upstream repository: <https://github.com/hku-mars/IKFoM>
- Pinned commit: `59cfc095ca74425f9b330c7c04a5d74f68c6dd62`
- Commit verification: `git ls-remote https://github.com/hku-mars/IKFoM.git HEAD`
  on 2026-07-28 returned this object ID.
- Integration date: 2026-07-28.
- Upstream license: GNU GPL version 2, as declared by upstream `LICENSE`.
- Files vendored and used:
  - `IKFoM_toolkit/esekfom/esekfom.hpp`
  - `IKFoM_toolkit/esekfom/util.hpp`
  - `IKFoM_toolkit/mtk/**`
  - upstream `LICENSE` and `README.md`
- Excluded upstream content: sample applications, PDFs, editor metadata and
  Git metadata. The samples are algorithm references, not package dependencies.
- Differences from upstream source: none. Project state/process definitions
  live outside the vendor tree and are recorded separately when introduced.
- Build behavior: exports the real toolkit include directory through
  `ikfom_vendor::ikfom_vendor`, compiles an upstream API smoke test, and fails
  CMake configuration when the pinned source is absent.

The upstream license is preserved at `vendor/IKFoM/LICENSE` and copied to
`LICENSES/IKFoM-GPL-2.0.txt`. Do not replace the pinned commit with a branch
name.

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
