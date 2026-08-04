# PX4--LIO alignment mathematics

For exact paired epochs `t_k`, the supervisor estimates only

\[
 ^L p = R_z(\psi)\,^P p + t,\qquad
 ^L R_B = R_z(\psi)\,^P R_B.
\]

The yaw estimate is the weighted circular mean

\[
 \psi=\operatorname{atan2}(\sum w_k\sin\psi_k,
                          \sum w_k\cos\psi_k),
\]

and translation is the weighted mean of `p_L - R_z(psi) p_P`. The estimator
uses a bounded window, median/MAD outlier rejection, translation/yaw
dispersion gates, residual-trend gates, covariance floors, and a horizontal
excitation gate when no authoritative yaw exists.

Roll/pitch disagreement is measured from body-Z vectors. It is a rejection
condition; it is never absorbed into `^L T_P`. Samples must have equal
nanosecond timestamps and remain within one LIO generation, PX4 reset
generation, and PX4 time generation. A missing pair leaves alignment invalid;
there is no identity fallback.
