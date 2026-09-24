# Motion acceptance policy evidence

`tools/runtime/evaluation.py:2154-2158` explicitly leaves motion metrics descriptive and emits `MOTION_ACCEPTANCE_POLICY_UNAVAILABLE`; the source comment says motion remains descriptive until W9 pins an explicit acceptance contract. The existing A1/A2/A3 report status is therefore `NOT_EVALUABLE` for motion even when descriptive motion metrics exist. No motion threshold was inferred from these runs or introduced here.
