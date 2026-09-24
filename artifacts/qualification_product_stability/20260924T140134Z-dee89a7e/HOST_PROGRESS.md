# Host progress

The historical 481 ms event has no process/PSI or observer-scheduler witness. It must not be called host overload, nor can host scheduling be ruled out. The new diagnostic observer samples `/proc` process CPU/RSS/scheduler counters and PSI at 1 Hz, plus its own wake cadence at 50 ms. The wake cadence is a bounded detection witness for a sub-second observer scheduling gap; 1 Hz CPU/PSI is context rather than a precise timestamp of a 150–200 ms incident.

No host monitor is connected to flight authority or admission. Native observer failure or dropped diagnostics leaves product behavior unchanged and makes first-component attribution `UNRESOLVED`.

