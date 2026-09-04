# Instrumented E5 replay addendum

This is supplemental evidence from the post-telemetry replay. It does not
rewrite the Task A result from the original E5 artifact, and its failure was a
runtime/native failure rather than a valid fixed-cycle injection.

The captured planner trace at the emergency transition reported:

| field | measured value |
| --- | ---: |
| anchor_error_m | 0.482191 |
| projected_anchor_error_m | 0.579379 |
| retained_tracking_limit_m | 0.250000 |
| relative_anchor_speed_mps | 0.485939 |
| sampled_path_clear | true |
| backup_available | true |
| time_to_backup_start_s | 0.854239 |
| committed_suffix_usable | false |
| tracking_certificate_exceeded | true |
| projected_tracking_certificate_exceeded | true |
| emergency_authorization_reason | 1 (`ACTUAL_ANCHOR_CERTIFICATE_EXCEEDED`) |
| emergency_candidate_commit_result | 1 (committed) |

The trace therefore directly measures certificate exhaustion at this replay's
emergency branch. It does not establish that the original E5 injected failure
was safe-margin; that causal question remains unresolved until a valid E5b
stimulus is captured.
