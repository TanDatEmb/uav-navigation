# Focused timing comparison

Method: `analyze_sitl.py` deserializes the versioned ROS bag from each listed session; `HANDOFF_MEASUREMENTS.csv` contains all 24 exact adapter-admission request transitions. Diagnostic values below are periodic `DiagnosticArray` observations in microseconds, with per-session p50/p95/p99/max. They are diagnostic samples, not deadline or flight qualification distributions.

| Source / session | Command store publish | Transport publish | Planning scheduling gap | Command transition lock wait | Owner publication lock wait |
|---|---|---|---|---|---|
| repair `112040` | `39/52/58/152` | `35/45/49.92/122` | `0/59.2/400005/400133` | `0/0/0/9` | unavailable |
| repair `112251` | `39/54/60.87/149` | `35.5/47.35/52.87/144` | `0/81.35/400018/400189` | `0/0/0/10` | unavailable |
| repair `112456` | `41/65/86.35/179` | `38/59/82.28/176` | `0.5/103.35/399954/400119` | `0/0/10/14` | unavailable |
| cut `134651` | `40/53/61/243` | `36/46.1/51/64` | `0/81.1/399953/400089` | `0/0/0/9` | `0/0/0/0` |
| cut `134910` | `45/64/75/153` | `41/59.45/67.89/149` | `2/86/399984/400061` | `0/0/0/0` | `0/0/0/1` |
| cut `135131` | `41/59/71.1/818` | `36/51/58.2/801` | `1/83.5/399952/400058` | `0/0/0/39` | `0/0/0/0` |

The isolated cut maximum transport publish sample of 801 us is larger than the repair maximum 176 us; the p99 range overlaps/improves. The cause of that one tail sample is not established. No concurrent adapter lease or mission-handoff gap occurred. The 400 ms planning scheduling spikes exist in both sources and are not interpreted as a new product deadline. The owner lock-wait observation records the *latest* publication attempt at each diagnostics tick, so it can miss transient peaks; the repair source has no equivalent direct measure. We do **not** claim a latency improvement from reducing execution mutexes `2 → 1`.

The cut's exact adapter-admission handoff gap min/median/max is `19.894/20.030/20.093 ms` over 12 transitions; repair is `19.898/20.007/20.072 ms`. These were measured by one common bag analyzer and are comparable. The previous repair artifact's `4/16/28 ms` used a different adapter tracking-setpoint boundary, so it is retained as historical context rather than compared as the same metric. Nearest recorded PX4 setpoint gap is approximate because the PX4 message has no request identity. Planner solve and polynomial sample remain outside the owner mutex. The bounded final publish callback remains within the owner mutex as it was in the old timeline store.
