---
name: rebase-onto-main
description: Rebase a branch onto main, handling squash-merged parent branches cleanly
argument-hint: "[optional: branch name, defaults to current branch]"
allowed-tools: Bash Read Glob Grep
---

# Rebase Branch onto Main

A squash-merged parent branch makes a plain `git rebase main` replay its old
commits and conflict. Cherry-pick only this branch's own commits instead.

1. **Branch:** the named one, else current. `git worktree list` first; a branch
   checked out elsewhere cannot be checked out here, so use
   `git -C <worktree> rebase main`. In loops guard
   `git checkout -q "$b" || { echo "SKIP $b"; continue; }`.
2. `git fetch origin main:main`
3. `git merge-base <branch> main`, then `git log --oneline <merge-base>..<branch>`.
4. Find the first commit that belongs to this branch (look for the squashed
   parent's PR on `main`). No parent branch: plain `git rebase main`, done.
5. `git checkout -b <branch>-rebase main`, then
   `git cherry-pick <first-unique>^..<branch>`; resolve and
   `git cherry-pick --continue`.
6. `git branch -m <branch> <branch>-old && git branch -m <branch>-rebase <branch>`
7. **Prove nothing changed** before the old head is lost. Compare from the
   first unique commit, not the old merge-base, or the squash-merged parent's
   commits show as dropped (`<`) and the check never passes:
   `git range-diff <first-unique>^..<old-head> main..<branch>`
   Every line must be `=`. `!` = diff changed, `<` = dropped, `>` = appeared:
   stop. Scripted (the `-:` pattern is needed to catch `>` lines):
   ```
   rd=$(git range-diff --no-color <first-unique>^..<old-head> main..<branch>)
   tot=$(echo "$rd" | grep -cE '^ *([0-9]+|-):')
   eq=$(echo  "$rd" | grep -cE '^ *[0-9]+: +[0-9a-f]+ = [0-9]+:')
   [ "$tot" = "$eq" ] && echo CLEAN || echo REVIEW
   ```
8. **Ask the user** before `git push origin <branch> --force-with-lease`
   (never bare `--force`). Rebase any stacked branch above it with
   `git rebase --onto <branch> <old-head> <branch-above>`.
9. `git branch -D <branch>-old`
