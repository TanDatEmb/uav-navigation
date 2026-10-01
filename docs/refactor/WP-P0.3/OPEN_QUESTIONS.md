# WP-P0.3 — Open questions

1. **Historical baseline mapping:** current git has `origin/main=432dc94` and no `7e0b850`, `2543b0b4`, `74054ac` or usable historical R2 branches. Should the next wave branch from this squashed baseline, or should the project restore a canonical history mirror before accepting PR sequence evidence?

2. **I2 planner API:** owner decision 2026-09-30 P03-I2 defers ambient API removal to P3. The current branch intentionally leaves it untouched; a future wave must first verify current `PlanningRequest` ownership and port product-path tests before deleting symbols.

3. **Build acceptance:** the supplied 23-package build is not bound to this branch head. A fresh build/test on `89c80cd` is required before merge; the attempted run was interrupted during `px4_msgs` generation, with no compile failure observed in the processed packages.

4. **SITL evidence:** no new SITL/replay evidence was generated. This package must not be used to claim runtime qualification.
