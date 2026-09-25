# A3 same-source-tick revision reachability

`WorldSnapshotStore` and the live producer are separate contracts. The store test permits a higher revision identity under an unchanged observation stamp. The live `MappingActor` rejects an input when `observation.stamp_ns <= last_observation_stamp_ns_` (`src/mapping/navigation_mapping/src/mapping_actor.cpp`, current source around the observation admission check), so ordinary live mapping does not emit same-stamp updates.

Classification: `STORE_CONTRACT_PERMITS`; `LIVE_PRODUCER_DOES_NOT_EMIT` on the inspected path. Historical A3 remains `NOT_EVALUABLE / SOURCE_TIMESTAMP_DUPLICATE_CONFLICT` where required. No runtime event is fabricated to close historical data loss.
