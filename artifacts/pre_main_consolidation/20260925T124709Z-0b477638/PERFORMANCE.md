# Focused timing / performance summary

Across five valid nominal sessions: Core PVA command publication interval, n=17,320, p50 19.913 ms, p95 51.962 ms, p99 61.405 ms, max 87.375 ms. Adapter admitted-command interval, n=17,305, p50 19.783 ms, p95 52.154 ms, p99 61.645 ms, max 85.598 ms. Both maxima remain below the unchanged 100 ms command lease. No nominal command lease expiry was recorded. Adapter odometry observed max gaps 20–24 ms; setpoint max gaps 16–20 ms; stale state failure counts zero. One pre-admission typed rejection was recorded separately; no identity rejection.

Mapping callback total distributions by session (microseconds):

| Session | n | p50 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| 135537 | 762 | 39,699 | 52,448 | 61,513 | 72,658 |
| 140742 | 680 | 41,545 | 77,454 | 97,575 | 116,039 |
| 141506 | 884 | 41,330 | 60,538 | 68,987 | 87,600 |
| 142616 | 840 | 42,324 | 64,756 | 80,636 | 104,844 |
| 143615 | 635 | 43,185 | 68,557 | 83,865 | 90,803 |

Worker enqueue wait p50 was 11–14 us; p99 ranged 66–119 us, with isolated maxima 1.523–4.209 ms. No nominal >150 ms accepted-state event, lease expiry, or evidence writer drop was observed. World event producer CPU and queue high-water are not in current evidence (`NOT_MEASURED`, P2 debt); world writer drops were zero. No WCET claim is made.
