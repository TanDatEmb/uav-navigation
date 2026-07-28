# Local build patch

The complete pinned upstream source is present and the `.msg` files are
unmodified. One CMake option, `LIVOX_BUILD_HARDWARE_DRIVER`, gates SDK discovery
and compilation of the hardware node. It defaults off for deterministic
offline dataset validation on hosts without Livox-SDK2. Setting it on restores
the upstream driver build path. `package.xml` is the upstream ROS 2 manifest.
