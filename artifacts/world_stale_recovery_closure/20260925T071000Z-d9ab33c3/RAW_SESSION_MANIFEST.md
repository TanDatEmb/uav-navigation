# Raw session manifest

`RAW_SESSION_MANIFEST.csv` contains a row for every regular raw file in the six counted sessions, with absolute path, byte size, and SHA256. It covers 411 files totaling 4,946,635,445 bytes (4.61 GiB). All source data remains outside Git under `/home/letandat/Dev/uav-navigation/.artifacts/runtime/`.

Every run records navigation Git HEAD `d9ab33c3e2321ac60794da4128863bc7933458ea`, its committed tree `5d6bfbdcea29d19d370267cba4667700cee73958`, PX4 checkout `deaff86ee335dd697677bcfc2415a23878e1b895`, and PX4 binary SHA256 `b69d990830625c54de458282ff109286c5a35597680675bcad0a163ea8640edc`. The runtime build manifest also supplies a source-content SHA256 per session; most cohort sessions use `d87cdabb103084659a30a3f166adb46d3151ba0fcbfcca48d6a03922adb44f76`. The supplemental 420 ms run uses `c3bb8eb5d33a56e9ea2fa286d0226e37b4aa053dee281705a4f5f541da32858b`, so it is reported separately and not pooled into same-build repeatability counts.

The CSV hashes all files within each retained session directory, including logs, timelines, maps, reports, scenario manifests, PX4 input snapshots, and gate trace. Symlink rows, if present, hash the link target bytes; this current manifest contains only regular files.
