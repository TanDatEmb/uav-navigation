# Git baseline and provenance

Before analysis, the original checkout `/home/letandat/Dev/uav-navigation` was on `codex/close-proven-findings` at `7da3e97cb399c2e39d62cfe60213a45e8a92300e`. `git status --porcelain=v2 --branch` contained only branch metadata (`branch.ab +0 -0`), with no staged, unstaged or untracked entries. `git fetch origin` updated the five named refs successfully. Remote target and original checkout HEAD matched at freeze time. The original checkout remained clean at the final drift check.

```
TARGET_REF  refs/remotes/origin/codex/close-proven-findings
TARGET_SHA  7da3e97cb399c2e39d62cfe60213a45e8a92300e
TARGET_TREE e7d2b56dbe4353810f029c9c6f0fe8991ebd63eb
MAIN_SHA    287b7b84cf4311e31d52ca04796223cc4efc1bc5
MERGE_BASE  287b7b84cf4311e31d52ca04796223cc4efc1bc5
```

Submodule gitlink revisions: `src/external/px4_msgs` `86d8239e962f6939e05c3737784f60c02fa884db`; `src/external/px4_ros2_interface_lib` `4a3370f084ac6f1ef001a4afa2b007845ffd0837`. They were initialized in the original checkout, **not initialized in the new audit worktree**; no submodule source claim is upgraded from prior artifacts. The new branch starts exactly at TARGET_SHA and contains only this artifact directory. No reset, clean, stash, rebase, force push or merge of old audit branches was used.

The target/main comparison is `git diff --stat main...TARGET`: 49 tracked paths, including 28,375 insertions and 23,508 deletions, dominated by the runtime safety ledger archive migration. The semantic audit does not infer behavior from diff size.
