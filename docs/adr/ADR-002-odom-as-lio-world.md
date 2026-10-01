# ADR-002: `odom` as the LIO world

**Status:** accepted. The LIO state and registration map use continuous local
`odom`, which may drift. M1 does not define a global map or publish `map -> odom`.
This makes the registration-map frame unambiguous and avoids claiming global
localization without loop closure.
