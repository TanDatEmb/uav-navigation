# PX4 v1.17 dependency lock

P0.7 supports only the audited PX4 v1.17 `px4_msgs` commit recorded in
`px4_msgs.repos`. Import it into this workspace with:

```bash
make deps-px4-sync
make deps-px4-verify
```

The manifest is consumed by `vcs import`; it is not a Git submodule, vendor
copy, or source mirror. A normal repository build never downloads this
dependency.
