# Coverage (generated from model)

{
  "scope": "3 requested state contracts; 8 object owners; 56 individually listed fields; 12 transitions; 4 scenarios",
  "objects": {
    "scoped": 8,
    "documented": 8
  },
  "fields": {
    "scoped": 56,
    "documented": 56,
    "writer_reader_closed_for_selected_paths": 56,
    "cross_boundary_runtime_unobserved": 8,
    "cross_boundary_field_ids": [
      "F-AVAILABLE",
      "F-COMMAND-CACHE",
      "F-COMMAND-RECEIVE",
      "F-HOLD-CONFIRMED",
      "F-HOLD-IN-FLIGHT",
      "F-MODE-ACTIVE",
      "F-RECOVERY-DEADLINE",
      "F-SUSPENDED"
    ],
    "note": "Writer/reader sites are enumerated for selected source paths; these eight still lack the live profile, downstream PX4 application, or workload timing needed to close runtime outcome."
  },
  "transitions": {
    "scoped": 12,
    "documented": 12,
    "related_test_groups": 8,
    "end_to_end_unjoined": 3
  },
  "scenarios": {
    "scoped": 4,
    "documented": 4,
    "runtime_observed": 0
  },
  "limitations": [
    "Claim-focused slice, not whole-repository exhaustive writer inventory.",
    "No live parameter dump, matched target-workload trace, or PX4 applied-state observation.",
    "Selected production class/helper tests use synthetic clocks/world/transport and do not demonstrate field behavior in a target workload."
  ]
}

`PARTIAL_AS_IS`: this scoped model does not claim whole-repository exhaustive writers, target workload observation, or PX4 applied-state evidence.
