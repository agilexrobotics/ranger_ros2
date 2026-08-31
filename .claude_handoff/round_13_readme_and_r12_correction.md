# Round 13 — Sim README + R12 correction

## Summary

Added `ranger_mini_v3_sim/README.md` documenting the launch
file hierarchy (the key clarification: use
`gazebo_full.launch.py` to drive the robot, not the
shorter-named `gazebo.launch.py`). Added a prominent docstring
warning at the top of `gazebo.launch.py` pointing users to the
full launch. Appended an architect correction to R12's handoff:
the "transient DDS shared-memory state" diagnosis was wrong;
the actual cause of the operator's symptom was using the wrong
launch file.

No code changes. Documentation only.

## Diff

Two commits this round:

- `dbbeb90 docs(sim): launch file hierarchy README + R12 handoff correction`
  - 4 files changed, 183 insertions(+), 1 deletion(-)
  - Adds `ranger_mini_v3_sim/README.md` (new file)
  - Modifies `ranger_mini_v3_sim/launch/gazebo.launch.py` (docstring header)
  - Modifies `.claude_handoff/round_12_regression_diag.md` (correction appended)
  - Modifies `.claude_handoff/CONTEXT.md` (R13 entry appended)

- (Pending) the R13 handoff commit (this file).

## New files (full content)

### ranger_mini_v3_sim/README.md

```markdown
# ranger_mini_v3_sim

Gazebo Harmonic simulation for the AgileX Ranger Mini v3. The
sim mirrors the real-robot driver's ROS interface
(ranger_base/src/ranger_messenger.cpp), so application code
(teleop, nav2, behavior trees) is portable between sim and
real with zero changes.

## Quick start

Build and source the workspace first:

    cd ~/agilex_ws
    colcon build --symlink-install
    source install/setup.bash

Then in three terminals:

    # Terminal A — start Gazebo with the robot + controllers
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

    # Terminal B — start the Twist messenger (waits ~30s for gz)
    ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

    # Terminal C — drive
    ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.3}}"

(Full file in repo at ranger_mini_v3_sim/README.md — includes
launch hierarchy table, ROS interface listing, motion modes
table, troubleshooting section, and dev history pointer.)
```

### .claude_handoff/round_13_readme_and_r12_correction.md

This file. Will be committed in step 6c below.

## Verification output

### Step 0 — Pre-flight

```
On branch phase-4-messenger
nothing to commit, working tree clean
[no orphans]
```

(Note: prompt referenced HEAD `4ee64a4`; actual HEAD was
`771c58d` — the R12 handoff commit. Same branch state
otherwise; proceeded.)

### Step 1 — Tour of launch files

5 launch files inventoried:
- `ranger_mini_v3_description/launch/display.launch.py` (RViz only)
- `ranger_mini_v3_sim/launch/gazebo.launch.py` (gz + robot, NO controllers)
- `ranger_mini_v3_sim/launch/gazebo_full.launch.py` (full stack)
- `ranger_mini_v3_sim/launch/view_sim_xacro.launch.py` (URDF print)
- `ranger_mini_v3_sim_messenger/launch/messenger.launch.py` (Twist node)

### Step 2 — README created

`ranger_mini_v3_sim/README.md` written (the literal architect spec).

### Step 3 — Docstring update

```
$ head -20 gazebo.launch.py
"""Gazebo bringup for Ranger Mini v3 — PARTIAL (no controllers).

==============================================================================
WARNING: This launch does NOT load ros2_control controllers.
…
For the FULL sim (gz + robot + controllers + bridge), use:
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py
…
==============================================================================

(Original docstring follows.)

Gazebo bringup for Ranger Mini v3 (Phase 3 checkpoint B).
…
"""
```

The warning block sits prominently above the original docstring,
which is preserved.

### Step 4 — Build + parse-check

```
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.09s]
Summary: 1 package finished [0.18s]
```

Both launch files parse cleanly post-edit:

```
=== gazebo.launch.py --print ===
<launch.launch_description.LaunchDescription object at …>
├── 5 × DeclareLaunchArgument
├── SetEnvironmentVariable
├── IncludeLaunchDescription (gz_sim)
├── ExecuteProcess (robot_state_publisher)
├── ExecuteProcess (parameter_bridge)

=== gazebo_full.launch.py --print ===
<launch.launch_description.LaunchDescription object at …>
├── IncludeLaunchDescription (gazebo.launch.py)
├── 9 × spawner ExecuteProcess (chained via OnProcessExit)
```

### Step 5 — R12 correction appended

```
$ tail -25 .claude_handoff/round_12_regression_diag.md
…
## Architect's correction (added 2026-05-12)

The "root cause" stated above is **incorrect**. The R12
symptom was NOT a transient DDS / shared-memory state issue.

**Actual cause:** the user had been launching
`gazebo.launch.py` (robot only, NO controllers) instead of
`gazebo_full.launch.py` (robot + controllers). …

(Full text in the R12 file. Original R12 content is preserved
above the correction; only appended.)
```

### Step 6 — CONTEXT entry + commit

```
[phase-4-messenger dbbeb90] docs(sim): launch file hierarchy README + R12 handoff correction
 4 files changed, 183 insertions(+), 1 deletion(-)
 create mode 100644 ranger_mini_v3_sim/README.md
```

(R13 handoff commit follows after this file is staged.)

## Deviations

None. All literal commands ran as written. Both launch parses
succeeded. README and docstring landed cleanly. R12 correction
appended without rewriting the original.

## Open questions

1. **The lesson from R12** ("when a user reports a symptom,
   first ask 'what did you actually type?' before running my own
   commands to reproduce") is captured in the R12 correction. It's
   worth folding into the bootstrap / running-context for future
   sessions so it's a standing principle, not a one-time note.
2. **Phase 4 is complete and functional.** Recommended next:
   squash-merge `phase-4-messenger` to `jazzy`, then proceed to
   Phase 5 (real-driver-side-by-side audit).
3. **Some launches share state across rounds** (e.g. the messenger's
   tick_diag log persists across a session). The `.claude_handoff/`
   trail should keep working for the squash-merge — `.claude_handoff/`
   stays tracked per the architect's existing strategy.
