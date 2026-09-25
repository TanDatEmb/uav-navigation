# Steady braking numerical contract

For steady input (`a0=j0=0`, speed `v`), preserve the existing minimum-snap family and its bounds:

- `v_peak = v`
- `a_peak = 15 v / (8 T)`
- `j_peak = 10 v / (sqrt(3) T^2)`
- `T >= 15v/(8 Amax)`, `T >= sqrt(10v/(sqrt(3)Jmax))`, and the existing sample-duration floor.

The implementation now checks the physical steady acceleration and jerk inequalities separately from polynomial evaluation tolerance. It constructs and evaluates the actual `Piece`, checks continuous extrema and finite support, and applies at most 32 upward representable-duration corrections (`nextafter(T,+infinity)`). Cancellation is checked during the bounded correction. An unresolved or materially invalid result fails synthesis. The existing numerical helper and its ULP allowance are unchanged and do not relax physical limits.

Non-steady PVAJ synthesis retains its prior bounded duration search and full PVAJ checks. Velocity/overspeed policy, dynamic limits, feasibility margins, planner deadlines, abort policy and terminal-altitude search policy are unchanged.

The integration exposed sub-ULP overage in the speed-support hull after duration correction. The bounded representable speed correction is at most 8 steps, with concrete support rechecked; it does not widen a threshold. See the second C1 commit and tests.
