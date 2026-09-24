# Recorder contract

`external_mode_scenario.py` records producer fields on their originating event: bundle owner cycle/source, heading rebind, activation, supersession, recovery retry, terminal monitor gates, and exact command admission/rejection. The old rolling activation/owner cache is removed. A missing field remains missing. The recorder does not synthesize an owner from a previous event, nearest time, or accepted waypoint.

Qualification-required streams use `EvidenceWriter` accounting for submitted, accepted, written, dropped, pending, snapshot-rejected, queue-full, closed-rejected, serialization-error and write-error counts. In the primary cohort, scenario+monitor writers submitted and wrote 420,486 records with zero drops/errors. The independent verification cohort submitted and wrote 342,942 with zero drops/errors. Both cohorts recorded `capture_complete=true` for all ten sessions. See `WRITER_ACCOUNTING.csv` and the raw manifest for each session.

Recorder metadata includes requested `qualification_scope`, version and provenance, plus requested/effective runtime configuration. Diagnostic drops must never affect flight authority; a qualification-required drop makes evidence ineligible.
