# Mermaid render tool provenance

This is a documentation-only follow-up to the source baseline captured in `baseline/environment.json`. Mermaid CLI was not on `PATH` during the initial capture. For the follow-up render, the tool was installed under `/tmp/uav-as-is-mmdc-20260920`; no global package install, `sudo`, browser download, or external visualization service was used.

| Component | Observed version / identity |
|---|---|
| `@mermaid-js/mermaid-cli` | 11.17.0 |
| Node.js | v24.19.0 at `/home/letandat/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node` |
| pnpm | v11.19.0 at `/home/letandat/.cache/codex-runtimes/codex-primary-runtime/dependencies/bin/fallback/pnpm` |
| Chrome | `/usr/bin/google-chrome`, 153.0.8010.52 |
| Puppeteer browser download | disabled with `PUPPETEER_SKIP_DOWNLOAD=1`; existing Chrome used |

`package.json` and `pnpm-lock.yaml` preserve the exact CLI dependency resolution. Their SHA-256 values are recorded below. The isolated installation succeeded with `pnpm install --ignore-scripts` after pnpm refused the initial install attempt that would run Puppeteer's lifecycle script. The Mermaid CLI binary itself ran successfully after the scripts were skipped.

The checked-in manifest and lockfile were copied to a second fresh temporary directory and installed again with the same flags. pnpm resolved the lockfile without downloading packages (using the local content-addressed store); `mmdc --version` again reported `11.17.0`. The final four-diagram render used that clean reproduction directory.

```text
package.json     4fec07003a6ed49b7a5f2be536187f4b51c5bf9b8ff03266e6b7257c1974775a
pnpm-lock.yaml   14f3f850708c77a071dd6b6a1ffbd4e018502aaa0f43c97e530c536a1f477d5f
```

The render used this Puppeteer configuration (adapt `executablePath` to an installed browser on another host):

```json
{"executablePath":"/usr/bin/google-chrome","headless":true}
```

From repository root, the Mermaid step was:

```sh
python3 artifacts/architecture_as_is/20260920T-as-is-local/tools/build_views.py \
  --mmdc /tmp/uav-as-is-mmdc-20260920/node_modules/.bin/mmdc \
  --puppeteer-config /tmp/uav-as-is-mmdc-20260920/puppeteer.json
```

It rendered `nominal_flow.svg`, `pass_through_handoff.svg`, `backup_recovery.svg`, and `terminal_and_handover.svg`. The first attempt stopped at a Mermaid parse error in `pass_through_handoff`; the canonical model text was rewritten to remove parser-sensitive semicolon separators, then all four diagrams rendered successfully. The corresponding `.mmd` remains generated from `model/architecture.json`.

These are static sequence views generated from inspected source behavior. Rendering does not validate model completeness, scheduling order at runtime, PX4 acceptance, or flight safety.
