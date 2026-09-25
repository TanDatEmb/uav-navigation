# Host progress

The historical 481 ms event has no process/PSI or observer-scheduler witness. It must not be called host overload, nor can host scheduling be ruled out. The new diagnostic observer samples `/proc` process CPU/RSS/scheduler counters and PSI at 1 Hz, plus its own wake cadence at 50 ms. The wake cadence is a bounded detection witness for a sub-second observer scheduling gap; 1 Hz CPU/PSI is context rather than a precise timestamp of a 150–200 ms incident.

No host monitor is connected to flight authority or admission. Native observer failure or dropped diagnostics leaves product behavior unchanged and makes first-component attribution `UNRESOLVED`.

In the new natural 784.659/269.229 ms event pair, the native observer's largest loop wake gap over the entire run was 105.953 ms; neither event overlapped a >150 ms observer scheduling gap. The 1 Hz host CPU PSI `some avg10` was 0.00 around 14:37:03–05 UTC, memory PSI was 0.00, and IO PSI `some avg10` decreased from 0.28 to 0.23. These low-rate samples do not prove that every host task was scheduled normally, but they supply no global CPU/memory-pressure explanation. Gazebo server CPU counters kept increasing across adjacent 1 Hz samples; the precise internal wait remains unobserved.
