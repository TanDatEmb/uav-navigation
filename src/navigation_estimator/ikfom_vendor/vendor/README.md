# Vendor source boundary

This directory is intentionally empty of IKFoM source. Network fetching during
`colcon build` is prohibited so builds stay reproducible and offline-capable.
An explicit import must use the commit pinned in `../UPSTREAM.md` and preserve
the upstream license and patch record.
