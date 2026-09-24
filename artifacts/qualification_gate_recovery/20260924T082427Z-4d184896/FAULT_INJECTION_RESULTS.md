# Controlled fault results

No new controlled delay has been injected. The pinned A2 event is a natural observed failure, not an injected fault. Controlled producer, executor, mutex or clock tests should be added only after natural instrumented attempts show which boundary needs isolation. No safety gate was relaxed to make a fault run succeed.
