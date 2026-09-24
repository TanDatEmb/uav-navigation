# Producer timing boundary

The IMU worker records `worker_estimate_ready_steady_ns` immediately before invoking its publication callback. The ROS publisher records entry, primary `publisher_->publish()` call and return using steady time. `(localization_epoch, sequence, source_stamp_ros_ns)` joins producer to adapter and monitor. Expected source deadline and prior published source stamp are included to distinguish a no-new-source interval from downstream delivery delay. A worker skip or serializer failure has no published message/trace and must be interpreted with IMU ingress, health, and publication counters. The sideband is emitted after primary publication and is best effort. A late sideband alone cannot establish late primary delivery.

Current product producer QoS: reliable KeepLast(10), from `QosProfiles::estimatorOutput()`. No QoS change in this branch. Producer interval distributions await instrumented SITL.
