# Typed admission reasons

Implemented categories at source `33ec3362`:

- Presence: `MESSAGE_MISSING`.
- Contract: `EXPECTED_FRAME_MISSING`, `MISSION_ID_EMPTY`, `FRAME_MISMATCH`, `HEADER_STAMP_INVALID`, `VALIDITY_WINDOW_INVALID`, `WORLD_STAMP_INVALID`, `STATE_STAMP_INVALID`, `LOCALIZATION_ID_INVALID`, `GOAL_ID_INVALID`, `WORLD_ID_INVALID`, `SAMPLE_ID_INVALID`, `STATUS_INVALID`, `ROLE_STATUS_MISMATCH`, `PVAJ_OR_YAW_NONFINITE`, `TRAJECTORY_TIME_INVALID`, `CONTINUATION_CONTRACT_INVALID`.
- Temporal: `NOW_INVALID`, `NOT_YET_VALID`, `EXPIRED`, `INVALID_WINDOW`. `VALID` is not a rejection. Current interval is `header > 0`, `header <= now <= valid_until`, `valid_until > header`.
- Session: `HEALTH_EPOCH_MISMATCH`, `MODE_ACTIVATION_MISMATCH`, `MISSION_SESSION_MISMATCH`, `REQUEST_REGRESSION`, `WORLD_IDENTITY_REGRESSION` (including world/localization/state source monotonicity exactly as current helper).
- Dynamic: `ODOMETRY_STALE`, `SAMPLE_NONINCREASING`, `TRACKING_ENVELOPE_EXCEEDED`, `TERMINAL_AUTHORITY_CLOSED`; terminal completed/recovery is an accepted command with a later bounded action, not a generic reject.

Each code maps to one diagnostic-only rejection record carrying stage, disposition, callback ROS/steady time, and compact command identity/time values. The numeric reason code is interpreted within its typed stage; numeric values from different families may overlap. No string-only `valid=0` remains. `NavigationCommandAdmission` keeps successful exact-admission semantics. N3 demonstrated `TEMPORAL_LEASE / NOT_YET_VALID / REJECT_RETAIN_PREVIOUS` on the actual runtime path.
