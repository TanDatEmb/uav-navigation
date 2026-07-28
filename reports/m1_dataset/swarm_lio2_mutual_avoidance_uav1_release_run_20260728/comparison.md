# Reference comparison

The project production run did not sustain tracking, so trajectory/map distance
metrics would be misleading. Only one corrected pose and one inserted scan were
produced.

The upstream ROS1 reference could not be executed on this host: Ubuntu 24.04 /
ROS 2 Jazzy is installed, while ROS1 Noetic and Docker/Podman are absent. No
ROS1 packages were injected into the production workspace. The exact intended
reference is Swarm-LIO2 commit
`a5f751a797bb92baa3104cdd384a312d3c8e7744`, `swarm_lio/config/mid360.yaml`.

This is an explicit missing acceptance gate, not an upstream-equivalence claim.
