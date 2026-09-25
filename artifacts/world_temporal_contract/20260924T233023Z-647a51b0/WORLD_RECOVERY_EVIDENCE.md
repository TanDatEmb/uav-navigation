# Fresh-world recovery evidence

Source contract found: a newly published immutable world is recertified before publication commit. A suspended active bundle is copied with the new validated world identity only if the exact authority timeline/pointers/prior world remain current. After commit, runtime checks suspended generation, active bundle validity and validity interval, localization epoch, failure latch and exposure latch, then re-locks and rechecks exact active goal/bundle before `observeRetainedCommand` resumes publication.

This is stricter than “freshness alone resumes”. Automatic resume after successful recertification is the existing behavior. No fresh-world SITL recovery was executed in this branch; runtime evidence remains missing. E1→E2 replacement race is fenced in source by owner version/pointer checks but needs barrier coverage specifically spanning suspended resume and transition locks.
