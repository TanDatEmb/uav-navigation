# Pre-beta stability debt

1. Improve repeatability of final terminal tracking/bridge usability; two of five fresh nominal trials safely stopped at waypoint 3 under the existing gate. Do not relax the 0.25 m terminal tracking gate or 0.75 m adapter anchor contract.
2. Establish the policy meaning of braking setpoints relative to `requested_cruise_speed_mps`; one complete mission is marked runtime FAIL because the generic runner maximum includes a BRAKING command of 1.669 m/s against 1.5 m/s cruise. Keep raw outcome and check unchanged until a separately approved contract defines this.
3. Historical N1 lacks independent sensor-generation sequence before bridge; current evidence locates sparse update at RegisteredScan input but cannot attribute upstream to simulator vs sensor producer.
4. World producer CPU and queue high-water unavailable.
5. Full EMERGENCY/PX4 authority handover and live-FMU integration are not closed.

These items are not resolved and do not claim beta readiness.
