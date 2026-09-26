# Navigation reactivation contract

`NavigationMode::onActivate()` starts a new mode activation identity and invalidates the previously accepted command. The adapter rejects a command whose `mode_activation_id` does not match the current activation. `NavigationModeExecutor::onActivate()` starts a distinct callback-fencing generation and schedules the owned external mode. The previous Hold attempt token cannot mutate this new generation.

The new activation therefore requires a newly admitted command; the old command cache is not restored. This is a source/component contract. The long-World-stale → confirmed Hold → healthy restoration → explicit mode reactivation sequence has not yet been run in SITL on this branch.
