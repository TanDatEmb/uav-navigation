# Local patches

## Compact FAST-LIO measurement update

The upstream dynamic update forms and explicitly inverts the dense
measurement-space matrix `H P H^T + R`. For Mid-360 registration, the
measurement dimension is commonly much larger than the 23-DoF error state.

For the FAST-LIO contract `h_v = I` with positive diagonal `R`, the production
path uses the equivalent information-form expression:

```text
K = (H^T R^-1 H + P^-1)^-1 H^T R^-1
```

The covariance system is solved with Eigen `LLT` (the covariance is required
to be SPD), while the information system is solved with Eigen `LDLT`.
The implementation checks finite inputs, positive covariance/information
factorizations, solver status, and finite gain output. A failed solve rejects
the measurement update, restores the propagated state and covariance, and is
reported as a numerical failure; it never silently falls back to a dense
inverse.

Project-owned differential tests compare the compact path with the original
dense expression across measurement dimensions below, equal to, and above 23,
multiple fixed seeds, multi-scale noise, near-rank-deficient Jacobians, and
wide-spectrum covariance. They compare gain action, `K H`, tangent correction,
manifold state, covariance, convergence, and final increment norm.

This patch does not change residual selection, measurement noise, convergence
limits, or the maximum iteration policy.
