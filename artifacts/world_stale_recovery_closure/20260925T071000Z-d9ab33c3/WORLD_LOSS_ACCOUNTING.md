# World evidence loss accounting

World witness records use the existing `EvidenceWriter` categories:

- `world_transaction`
- `world_transaction_producer_count`

For each category the reducer requires submitted, accepted, written, and dropped totals. It compares written totals with the exact `scenario.jsonl` artifact counts. Global serialization/write errors and category drops make required World evidence incomplete. This changes evidence eligibility only; it does not affect product state or command decisions.

The runtime event sequence and periodic cumulative producer count are independent witnesses. Internal event sequence holes, a producer count ahead of the last observed event, or a producer count regression are recorded as missing/conflicting evidence. The runtime producer ID binds both streams to one Core process instance.
