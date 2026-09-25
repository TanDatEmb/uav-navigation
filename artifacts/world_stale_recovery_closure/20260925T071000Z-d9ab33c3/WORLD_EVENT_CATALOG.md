# World event catalog

| Producer event | Meaning | Authority snapshot | Qualification use |
|---|---|---|---|
| `WORLD_PUBLICATION_COMMITTED` | World snapshot and exact certificate disposition committed | before/after active and pending identities, validation paths | world revision transaction |
| `WORLD_PUBLICATION_SUPERSEDED` | transaction lost currentness before commit | before/after owner snapshots | stale result evidence |
| `WORLD_PUBLICATION_FAILED` | publication did not finalize; existing fail-stop path follows | before/after owner snapshots | failure disposition |
| `WORLD_COMMAND_SUSPENDED` | exact current active generation had exposure suspended after temporal assessment failed | expected and post-suspension snapshot | stale-source transition |
| `WORLD_COMMAND_RECERTIFIED` | exact active certificate was retained against the newly committed immutable world | pre/post publication snapshot | recertification evidence |
| `WORLD_COMMAND_RESUMED` | same execution generation passed currentness and freshness checks and exposure resumed | pre/post resume snapshots | recovery evidence |
| `WORLD_RECERTIFICATION_REJECTED` | exact active certificate was not retained, or its transaction was superseded before mutation | pre/post transaction snapshots | unsafe/no-resume evidence |

Validation path values are numeric and stable in the producer: `0 none`, `1 changed/disjoint-region fast path`, `2 full immutable validation`, `3 terminal endpoint exception`, `4 expired recovery endpoint`, `5 rejected/invalidated`. Temporal assessment reason is a named enum string; non-temporal publication events use `NOT_APPLICABLE`.
