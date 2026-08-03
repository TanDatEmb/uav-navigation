#!/bin/sh
# Project-owned PX4 SITL startup wrapper. Keep the pinned PX4 rcS intact and
# apply only the simulation harness contract after the airframe is initialized.
set +e
. "${R}etc/init.d-posix/rcS"

# This harness controls PX4 through XRCE-DDS and intentionally has no GCS or
# MAVLink ground station. The x500 default NAV_DLL_ACT=2 otherwise rejects
# every arm request even when the OFFBOARD signal and estimator are healthy.
param set NAV_DLL_ACT "${PX4_NAV_DLL_ACT:-0}"
