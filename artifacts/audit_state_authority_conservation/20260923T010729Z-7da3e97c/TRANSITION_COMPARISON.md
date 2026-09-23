# Transition comparison

| Event | TARGET AS-IS | Candidate target and equivalence obligation |
|---|---|---|
| goal arrives during active MAIN | runtime desired goal/Episode may advance while Store old active remains | one intent transition; active bundle unchanged until exact staged/immediate commit |
| goal arrives during BACKUP | pending goal slot; suffix continues | one deferred intent; consume only after stopped replacement commits |
| planner result | key, world, predecessor, transaction and episode checks across node/Store | immutable result event; reducer checks one context key plus independent certificate/lease checks |
| future successor | Store stages then command timer activates | retained future activation and predecessor identity; no early sample |
| world update | mapping publishes latest, runtime recertifies/suspends exact generation | separate latest/certified identities and expiry; no unsafe resume after stop policy chosen |
| PASS_THROUGH crossing | previous measured segment overwritten each update; progress requires simultaneous continuation/suffix witness | persist bounded measured crossing independently, then policy acceptance when readiness holds; identity/age/reset guards |
| STOP endpoint | terminal hold and measured arrival confirmation | endpoint event distinct from measured stationary and mission accepted gate |
| BACKUP sample | Episode phase/recovery/suffix updated; adapter may enter Braking | explicit decision whether one-way stop begins; current adapter nominal recovery remains a conflict to resolve |
| localization reset | ingress locks/epochs invalidate old work | high-priority reset event fences all prior context; PX4 independently rejects old epoch |
| Hold | mode requests; executor schedules API; VehicleStatus confirms | typed PX4 protocol retains requested/in-flight/API complete/status confirmed/authority lost |

No entry is an equivalence claim yet. Test all ten scenarios under both event orders and same pinned input stream before removing AS-IS state. The independent target model in `tests/target_model.py` checks only abstract invariants.
