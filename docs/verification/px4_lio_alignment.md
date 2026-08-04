# PX4--LIO alignment verification

The mathematical estimator is covered by
`test_odometry_alignment_estimator`: known XYZ+yaw recovery, quaternion-sign
invariance, generation clearing, stationary-yaw rejection, and roll/pitch
rejection. The supervisor node pairs only equal timestamp epochs and publishes
alignment covariance, dispersion, excitation, sample count, generations, and
rejection reason.

Runtime qualification still requires a fresh SITL run after the implementation
commits. The historical baseline is not evidence for this phase.
