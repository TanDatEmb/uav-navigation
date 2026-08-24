#!/bin/sh
# Project-owned PX4 SITL startup wrapper. Keep the pinned PX4 rcS intact and
# apply only the simulation harness contract after the airframe is initialized.
set +e
. "${R}etc/init.d-posix/rcS"

# This harness controls PX4 through XRCE-DDS and intentionally has no GCS or
# MAVLink ground station. The x500 default NAV_DLL_ACT=2 otherwise rejects
# every arm request even when the OFFBOARD signal and estimator are healthy.
param set NAV_DLL_ACT "${PX4_NAV_DLL_ACT:-0}"

# PX4 SITL v1.17 does not publish the board-only system_power topic in this
# standalone profile. Keep the power circuit breaker scoped to simulation;
# real hardware must use the authoritative board power checks.
param set CBRK_SUPPLY_CHK "${PX4_PARAM_CBRK_SUPPLY_CHK:-894281}"

# The launcher exports these before rcS, so its parameter override loop applies
# them before Gazebo bridge and EKF2 start.  Print the effective values into
# px4_gazebo.log for every run; they are an audit trail, not a new data source.
echo "PX4 multisensor + external-vision contract (ground truth odometry disabled):"
param show SIM_GZ_EN_ODOM
param show SIM_GZ_EN_GPS
param show SIM_GZ_EN_BARO
param show EKF2_EV_CTRL
param show EKF2_HGT_REF
param show EKF2_GPS_CTRL
param show EKF2_BARO_CTRL
param show EKF2_RNG_CTRL
param show EKF2_MAG_TYPE
param show COM_RC_IN_MODE
