# Interface packaging differences

The `.msg` files are unmodified.

The upstream driver CMake and package manifests are not copied because they
require Livox SDK2 and build a hardware driver. The local CMake/package files
only generate the two official ROS interfaces and clearly identify this as an
interface-only package.
