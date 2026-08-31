# Round 07b — Resume Phase 4 from R07's blocked colcon build

## Blocked steps

**Step 1b (`colcon build --packages-select ranger_base ranger_bringup
--symlink-install`) failed** because `ranger_base` declares
`<depend>ugv_sdk</depend>` and `ugv_sdk` is not in the workspace's
`install/` tree. The prompt explicitly said for step 1b:

> If either fails, STOP and write to ## Blocked steps — something is
> wrong with the real-robot packages we'd want to know about. We'd
> NOT modify them to fix the failure.

Halting per the rule. Steps 1c–6c not executed. Step 1a (build
ranger_msgs) succeeded; the unblocker for the messenger build is in
place, so steps 2 onward could in principle run, but the prompt's
"STOP if either fails" is unambiguous.

### What ran

```
$ colcon build --packages-select ranger_base ranger_bringup --symlink-install
Starting >>> ranger_bringup
Starting >>> ranger_base
[0.138s] ERROR:colcon.colcon_cmake.task.cmake.build:Failed to find the following files:
- /home/sao/agilex_ws/install/ugv_sdk/share/ugv_sdk/package.sh
Check that the following packages have been built:
- ugv_sdk
Failed   <<< ranger_base [0.00s, exited with code 1]
Aborted  <<< ranger_bringup [0.01s]

Summary: 0 packages finished [0.11s]
  1 package failed: ranger_base
  1 package aborted: ranger_bringup
```

### Diagnosis — same shape as R07's halt

This is structurally the same situation as R07's halt: a sibling
package (`ranger_base`) declares a dep (`ugv_sdk`) that isn't built
into `install/`. ament_cmake's pre-flight refuses before doing any
actual work.

ranger_base's package.xml deps:

```
ament_cmake (build)
geometry_msgs nav_msgs rclcpp sensor_msgs std_msgs
tf2 tf2_ros boost controller_manager
ugv_sdk          ← not in install/
ranger_msgs      ← in install/ now (just built in 1a)
```

`ugv_sdk` source IS present at `~/agilex_ws/src/ugv_sdk/`
(CMakeLists.txt, package.xml, include/, sample/, etc.) — bootstrap
listed it as "Existing dependency: ~/agilex_ws/src/ugv_sdk
(real-robot CAN driver, do not touch)." Building it from existing
source would not modify it.

**Subtle observation:** `install/ranger_base/` already exists on
disk, but only contains `share/ranger_base/package.{sh,bash,zsh,ps1,dsv}`
— it's a stub from a prior partial colcon attempt (likely from when
the architect or operator sourced+built earlier in the project).
The launch files, executables, etc. are missing. Step 0b's cleanup
did not touch this stub because the cleanup only targeted
`ranger_mini_v3_sim_messenger`.

### What needs to happen to unblock

**Option A (analog of R07's resolution): build ugv_sdk first, then
retry step 1b.** Same logic the architect already accepted for
ranger_msgs — building generates artifacts from existing source,
doesn't mutate the source files. Bootstrap's "do not touch ugv_sdk"
rule (read carefully) is about source mutation, not about building.

```
colcon build --packages-select ugv_sdk --symlink-install
colcon build --packages-select ranger_base ranger_bringup --symlink-install
```

If ugv_sdk has its own deps (it shouldn't — it's a vendored CAN
driver, but worth checking), the same pattern repeats one more
level. Worst case 2–3 minutes of build time.

**Option B: skip step 1b entirely as a "defensive" step that's
optional.** The prompt explicitly framed step 1b as defensive
("having them in install/ now means we never hit this trap again").
Step 2's actual messenger build doesn't depend on ranger_base or
ranger_bringup; only on ranger_msgs (which 1a built). Skipping 1b
still moves Phase 4 forward; ranger_base/bringup get built later
when something actually needs them.

**Option C: clean the stub `install/ranger_base/` before step 2** so
nothing downstream picks up the half-built artifacts.

I recommend **A + C**: build ugv_sdk (the literal canonical fix),
clean the half-stub, then re-run 1b cleanly. Then 1c onwards.

### What I deliberately did NOT do

- Did NOT run `colcon build --packages-select ugv_sdk
  --symlink-install` to chain-resolve the dep.
- Did NOT re-attempt step 1b after the failure.
- Did NOT proceed to step 2 (messenger build) even though 1a's
  success means it would probably succeed. The prompt's halt was
  unambiguous; deciding which subsequent steps "can still run"
  is exactly the architect tidying that the rule reserves for them.
- Did NOT touch the source of `ugv_sdk`, `ranger_base`, or
  `ranger_bringup`.
- Did NOT commit any new files. `ranger_mini_v3_sim_messenger/` and
  this handoff file remain untracked on disk.
- Did NOT push anything new.

### What state the workspace is in

- `ranger_msgs` is now in `install/` (step 1a succeeded — message
  bindings generated cleanly).
- `ranger_base` is half-stub in `install/`: only the
  `package.{sh,bash,zsh,ps1,dsv}` files exist (created by ament's
  pre-flight before the build failed at step 1b). No actual
  ranger_base library/executable artifacts.
- `ranger_bringup` is not in `install/` at all.
- `ugv_sdk` is not in `install/` (and is the unblocker).
- Other packages unchanged.

---

## Summary

R07b unblocked the messenger build by getting ranger_msgs into
`install/` (step 1a succeeded in 6.59s), but immediately hit the
defensive step 1b's failure: `ranger_base` needs `ugv_sdk` and
`ugv_sdk` isn't built. The prompt explicitly says STOP on 1b
failure. Halted. The actual messenger build (step 2) was not
attempted; if attempted it would probably succeed (its only
sibling-package dep, ranger_msgs, is now in install/), but the
"halt and ask" rule beats the "this would probably work" instinct.

Same fix shape as R07: `colcon build --packages-select ugv_sdk
--symlink-install` then retry 1b. Two-minute round-trip.

## Diff

No commits this round. No source mutations. Two state changes:

- `install/ranger_msgs/` populated by step 1a's successful build.
- `install/ranger_base/` half-populated by step 1b's failed-mid-build
  attempt (only the dep-tracking shell files; no library/executable
  artifacts).

## New files (full content)

### .claude_handoff/round_07b_phase4_messenger_dual_ackermann_resume.md

This file. Not committed — see Blocked steps.

No other new files this round. The
`ranger_mini_v3_sim_messenger/` package source from R07 is still
on disk uncommitted, exactly as captured in
`round_07_phase4_messenger_dual_ackermann.md`.

## Verification output

### Step 0a — Pre-flight

```
On branch phase-4-messenger
Untracked files:
	.claude_handoff/round_07_phase4_messenger_dual_ackermann.md
	ranger_mini_v3_sim_messenger/

nothing added to commit but untracked files present
phase-4-messenger
908c94c fix(sim): URDF dimensions match real-robot RangerMiniV3Params
```

### Step 0b — Clean stub install

```
$ rm -rf install/ranger_mini_v3_sim_messenger build/ranger_mini_v3_sim_messenger
$ ls -la install/ | grep -E "ranger_|ugv_"
drwxrwxr-x 3 sao sao  4096 May 12 02:05 ranger_mini_v3_description
drwxrwxr-x 3 sao sao  4096 May 12 02:05 ranger_mini_v3_sim
```

(Note: `install/ranger_base/` was NOT here at this point. It got
created later by step 1b's failed attempt — see step 1b output.)

### Step 1a — Build ranger_msgs (SUCCESS)

```
$ colcon build --packages-select ranger_msgs --symlink-install
Starting >>> ranger_msgs
Finished <<< ranger_msgs [6.59s]

Summary: 1 package finished [6.68s]
```

### Step 1b — Build ranger_base + ranger_bringup (BLOCKED)

```
$ colcon build --packages-select ranger_base ranger_bringup --symlink-install
Starting >>> ranger_bringup
Starting >>> ranger_base
[0.138s] ERROR:colcon.colcon_cmake.task.cmake.build:Failed to find the following files:
- /home/sao/agilex_ws/install/ugv_sdk/share/ugv_sdk/package.sh
Check that the following packages have been built:
- ugv_sdk
Failed   <<< ranger_base [0.00s, exited with code 1]
Aborted  <<< ranger_bringup [0.01s]

Summary: 0 packages finished [0.11s]
  1 package failed: ranger_base
  1 package aborted: ranger_bringup
```

Halt per prompt rule. See `## Blocked steps`.

### Diagnostic captures

```
$ ls /home/sao/agilex_ws/src/ugv_sdk/
CHANGELOG.md  cmake  CMakeLists.txt  docs  doxyfile
include  LICENSE  package.xml  README.md  sample

$ ls /home/sao/agilex_ws/install/ | grep -E "ranger_|ugv_"
ranger_base                    ← NEW; created by failed step 1b
ranger_mini_v3_description
ranger_mini_v3_sim
ranger_msgs

$ ls /home/sao/agilex_ws/install/ranger_base/share/ranger_base/
package.bash  package.dsv  package.ps1  package.sh  package.zsh
```

`install/ranger_base/` is a half-stub — just the dep-tracking shell
files. No actual ranger_base build artifacts.

### Steps 1c, 2, 3, 4, 5, 6

Not executed.

### Manual verification steps for the operator

The full bringup integration test (step 4) — including the forward-
drive verification, the arc-turn test, and the per-wheel steering
angle captures — was the visible payoff of this round and never
ran. Once unblocked, the round resumes from step 1b (re-run with
ugv_sdk built) and ends with the operator running:

```bash
# Terminal A
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after ~30s)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

## Deviations

- None. Step 0 + step 1a ran exactly as the prompt specified. Step
  1b's literal failure triggered the prompt-mandated halt. No
  workarounds attempted.

## Open questions

1. **Same shape as R07's open question 1.** Building ugv_sdk to
   unblock ranger_base — yes or no? The architect's R07b prompt
   explicitly extends the building-is-OK rationale to
   ranger_base/ranger_bringup; extending it one more level to
   ugv_sdk feels obvious to me but isn't literally written down.
   Recommend yes.

2. **The `install/ranger_base/` half-stub** should be cleaned
   before the next attempt to avoid downstream confusion:

   ```
   rm -rf install/ranger_base build/ranger_base
   ```

3. **Should I also clean `install/ranger_mini_v3_description` and
   `install/ranger_mini_v3_sim`** before the next round? They're
   from R06, with the OLD URDF dimensions (wheelbase 0.50, track
   0.38). R07 step 1b rebuilt `ranger_mini_v3_description` with
   the new dimensions (0.494, 0.364), but `--symlink-install`
   means the ranger_mini_v3_sim install just symlinks back into
   src — and src is now consistent. Should still be fine, but a
   `colcon build --packages-up-to ranger_mini_v3_sim` after the
   ugv_sdk fix would guarantee everything is rebuilt against the
   updated URDF in one go. Architect call.

4. **R07's open question 2 still applies** ("workspace sweep build
   to catch any other missing deps"). After ugv_sdk → ranger_base
   → ranger_bringup is sorted, the answer is "yes, do it once."
   The simplest is `colcon build --symlink-install` (no
   --packages-select) to build everything. Worth ~1 minute.
