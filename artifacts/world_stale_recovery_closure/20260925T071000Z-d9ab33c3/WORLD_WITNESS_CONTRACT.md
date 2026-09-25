# World transaction witness contract

## Authority and source

WorldModel remains the sole world authority. `NavigationRuntimeNode` emits an immutable diagnostic witness only after a WorldSnapshotStore / ExecutionAuthority transaction has returned its disposition. The witness is not consumed by admission, planning, command publication, or PX4.

Each event carries a process-local `runtime_instance_id`, monotonic `producer_event_sequence`, observer-supplied `session_id`, ROS and steady event timestamps, previous/next world identity, before/after execution timeline version and active/pending identities, validation path, temporal reason, disposition, and exact transaction key. The observer copies producer identity verbatim and adds receive timestamps; it does not infer a transaction by temporal proximity.

Transaction key:

```text
<before timeline version>:<next localization epoch>:<next generation>:<next revision>:<next observation stamp ns>
```

This distinguishes world revisions, localization epochs, and execution timeline transactions. Producer sequence disambiguates events sharing one transaction key.

## Event creation points

- Publication committed / superseded / failed: immediately after the exact WorldSnapshotStore commit decision.
- Command suspended: after `ExecutionAuthority::suspendIfCurrentSnapshot` succeeds for the captured active snapshot.
- Command recertified: after the exact active pointer is retained by world publication and before any resume attempt.
- Command resumed: after the currentness / freshness / exposure gates permit the same bundle generation to resume.
- Recertification rejected: after the publication transaction declines to retain the exact active certificate.

The event carries bounded identities and small scalar dispositions only. No map or trajectory payload is serialized.

## Runtime identity scope

The runtime producer ID is a process-local steady-clock identity created on first World event. Session ID is owned by the C0-SW observer. A producer restart within one required World session is incomplete evidence, not a continuation of the old sequence.
