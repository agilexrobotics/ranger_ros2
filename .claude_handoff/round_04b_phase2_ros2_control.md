# Round 04b — Phase 2 ros2_control wiring

## Blocked steps

**Step 12 (xacro expansion of `ranger_mini_v3_sim.xacro`) failed**
because the cross-branch build strategy from step 10, combined with
`--symlink-install`, leaves the description package's installed
`urdf/ranger_mini_v3.xacro` as a **dangling symlink** the moment we
switch the source tree back to `phase-2-ros2-control` (where
`ranger_mini_v3_description/` does not exist on disk).

Per the no-silent-route-arounds rule, halting here. Steps 13, 14, 15,
17, 18, 19 not executed. The phase-2 source commit (step 9) and the
two colcon build steps (step 10) DID complete; phase 2 itself is in a
defensible state on local `phase-2-ros2-control`. The breakage is in
the verification path only.

### What ran

```
$ source install/setup.bash
$ SIM_SHARE=$(ros2 pkg prefix ranger_mini_v3_sim)/share/ranger_mini_v3_sim
$ xacro "$SIM_SHARE/urdf/ranger_mini_v3_sim.xacro" \
        "controller_yaml:=$SIM_SHARE/config/ranger_mini_v3_controllers.yaml"
error: No such file or directory:
       /home/sao/agilex_ws/install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
       [Errno 2] No such file or directory:
       '/home/sao/agilex_ws/install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro'
when processing file: /home/sao/agilex_ws/install/ranger_mini_v3_sim/share/ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
(exit 2)
```

### Root cause

`colcon build --symlink-install` for `ranger_mini_v3_description`
(step 10's first build) created symlinks in `install/` pointing back
into the source tree:

```
$ ls -la install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/
ranger_mini_v3.xacro -> /home/sao/agilex_ws/src/ranger_ros2/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
```

Step 10 then `git checkout phase-2-ros2-control`-ed the source tree.
That branch does not contain `ranger_mini_v3_description/`, so:

```
$ ls -la src/ranger_ros2/ranger_mini_v3_description/urdf/
ls: cannot access '…': No such file or directory
```

The symlink is now dangling. xacro chokes on it. The same trap would
hit the meshes, the rviz config, and the launch file.

`colcon` itself reported success for both invocations — the breakage
only surfaces when the installed files are actually consumed (step 12
+ later runtime).

### Why the prescribed strategy can't work as written

The architect's step 10 instruction was:

> Build phase 1 from its branch (description)
>   ( cd src/ranger_ros2 && git checkout phase-1-description )
>   colcon build --packages-select ranger_mini_v3_description --symlink-install
> Switch back to phase 2 branch
>   ( cd src/ranger_ros2 && git checkout phase-2-ros2-control )
> Source so phase 1 is visible to phase 2's build
>   source install/setup.bash
> Build phase 2
>   colcon build --packages-select ranger_mini_v3_sim --symlink-install

This works for the colcon build of `ranger_mini_v3_sim` (which only
needs to write its own files), but it cannot work for **runtime
consumption** of the description package — anything that opens
`install/ranger_mini_v3_description/share/.../ranger_mini_v3.xacro`
is opening a symlink to a path that no longer exists in the working
tree of phase-2.

The fix has to be one of (architect to choose; I did NOT pick one):

**Option A — drop `--symlink-install` for the description build.**
Use plain `colcon build --packages-select ranger_mini_v3_description`
(or `--merge-install`). That copies files into `install/` rather than
symlinking, so they survive the source-tree branch switch.

**Option B — git worktrees.** Check out
`phase-1-description` in a separate worktree under e.g.
`~/agilex_ws/src_phase1/ranger_ros2/`. Add it to `COLCON_PREFIX_PATH`
or temporarily into the workspace `src/`. Both source trees coexist;
neither moves; symlinks stay valid.

**Option C — temporarily place phase-1's
`ranger_mini_v3_description/` directory into phase-2's source tree
just for the build.** E.g. `git checkout phase-1-description --
ranger_mini_v3_description/` while on phase-2. The directory then
exists on disk and the symlinks resolve. Risk: if I forget to remove
it before the next git operation, it gets staged onto phase-2.

**Option D — defer cross-package verification to integration.** Skip
step 12's standalone xacro check on phase-2; trust that when both
packages land together (squash-merge into jazzy), the description's
URDF is in the same source tree as the sim's URDF and the include
resolves. The phase-2 colcon build (step 10) already succeeded; the
package itself is buildable.

My read: **Option A** is the smallest change. The description
package's `--symlink-install` was a phase-1-time choice that doesn't
need to extend to cross-branch use. **Option B** is the most robust
long-term but is more setup. **Option D** is least work but skips a
verification the architect specifically wanted.

### What state the local repo is in

- Branch: `phase-2-ros2-control` @ `a3f7289 feat(sim): add
  ranger_mini_v3_sim package with ros2_control wiring (phase 2)`.
- This is **1 commit ahead of `origin/jazzy`** and **not yet pushed**.
- Working tree: clean except `.claude_handoff/` (untracked — phase-2
  doesn't carry the prior rounds' handoffs).
- `install/ranger_mini_v3_sim/` exists and was built cleanly.
- `install/ranger_mini_v3_description/` exists but its files are
  symlinks pointing at non-existent paths in the current source tree
  (the dangling-symlink problem above).

### What I deliberately did NOT do

- Did NOT switch back to `phase-1-description` to make step 12
  succeed. That would route around the failure.
- Did NOT rebuild description with `--merge-install` or without
  `--symlink-install`. Architect choice (Option A above).
- Did NOT create a worktree (Option B), check out phase-1's directory
  inline (Option C), or skip the verification (Option D).
- Did NOT push `phase-2-ros2-control` to origin. The branch is in
  good shape, but pushing without verification feels off-protocol.
- Did NOT commit this handoff file. (Step 18 was the commit; halted
  upstream of it.)

---

## Summary

Pre-flight fix-ups (Step 0a–0d) ran cleanly: working tree was clean
modulo the expected untracked Round-04 handoff; local `jazzy`
repointed to `origin/jazzy` and fast-forwarded to `89c3308`; local
`humble` deleted (`-d` succeeded with a warning that it's only merged
on the upstream remote, which is fine). Phase-2 source created and
committed (`a3f7289`). The two cross-branch colcon builds completed
clean. Static URDF expansion (step 12) then failed because
`--symlink-install` for the description package leaves dangling
symlinks once the source tree is switched back to `phase-2-ros2-control`.
Halted per the rule.

## Diff

The phase-2 source commit `a3f7289` is the only commit this round
(the handoff commit, step 18, was halted). Its diff:

```diff
commit a3f7289…
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>
Date:   Tue May 12 ~01:45 2026 -0400

    feat(sim): add ranger_mini_v3_sim package with ros2_control wiring (phase 2)

 ranger_mini_v3_sim/CMakeLists.txt                     |  11 ++
 ranger_mini_v3_sim/config/ranger_mini_v3_controllers.yaml | 50 ++++++++
 ranger_mini_v3_sim/launch/view_sim_xacro.launch.py    |  29 +++++
 ranger_mini_v3_sim/package.xml                        |  31 +++++
 ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro      |  66 ++++++++++++
 5 files changed, 187 insertions(+)
```

Full content of each new file is in the next section.

## New files (full content)

### ranger_mini_v3_sim/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ranger_mini_v3_sim</name>
  <version>0.1.0</version>
  <description>
    Gazebo Harmonic simulation support for the AgileX Ranger Mini v3.
    Wraps ranger_mini_v3_description with ros2_control hardware
    interfaces and the gz_ros2_control plugin. The kinematic Twist
    messenger lives in a separate Phase 4 package.
  </description>
  <maintainer email="solanrewaju2020@fau.edu">Shuaib Olanrewaju</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>ranger_mini_v3_description</exec_depend>

  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>joint_state_broadcaster</exec_depend>
  <exec_depend>position_controllers</exec_depend>
  <exec_depend>velocity_controllers</exec_depend>
  <exec_depend>gz_ros2_control</exec_depend>
  <exec_depend>ros_gz_sim</exec_depend>
  <exec_depend>ros_gz_bridge</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### ranger_mini_v3_sim/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro

```xml
<?xml version="1.0" encoding="utf-8"?>
<!--
  ranger_mini_v3_sim.xacro
  Top-level xacro for Gazebo Harmonic simulation. Layers
  ros2_control hardware interfaces and the gz_ros2_control
  plugin onto the kinematics-only description package.
-->
<robot name="ranger_mini_v3" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:arg name="controller_yaml" default=""/>

  <!-- The kinematic description, untouched. -->
  <xacro:include filename="$(find ranger_mini_v3_description)/urdf/ranger_mini_v3.xacro"/>

  <!--
    Hardware interfaces.
    Steering joints: position interface (commanded by Phase 4 node).
    Wheel joints:    velocity interface (commanded by Phase 4 node).
  -->
  <ros2_control name="ranger_mini_v3_gz_system" type="system">
    <hardware>
      <plugin>gz_ros2_control/GazeboSimSystem</plugin>
    </hardware>

    <xacro:macro name="steering_iface" params="prefix">
      <joint name="${prefix}_steering_joint">
        <command_interface name="position">
          <param name="min">-2.1</param>
          <param name="max"> 2.1</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </xacro:macro>

    <xacro:macro name="wheel_iface" params="prefix">
      <joint name="${prefix}_wheel">
        <command_interface name="velocity">
          <param name="min">-30.0</param>
          <param name="max"> 30.0</param>
        </command_interface>
        <state_interface name="position"/>
        <state_interface name="velocity"/>
      </joint>
    </xacro:macro>

    <xacro:steering_iface prefix="fl"/>
    <xacro:steering_iface prefix="fr"/>
    <xacro:steering_iface prefix="rl"/>
    <xacro:steering_iface prefix="rr"/>

    <xacro:wheel_iface prefix="fl"/>
    <xacro:wheel_iface prefix="fr"/>
    <xacro:wheel_iface prefix="rl"/>
    <xacro:wheel_iface prefix="rr"/>
  </ros2_control>

  <gazebo>
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlSystem">
      <parameters>$(arg controller_yaml)</parameters>
    </plugin>
  </gazebo>

</robot>
```

### ranger_mini_v3_sim/config/ranger_mini_v3_controllers.yaml

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    fl_steering_position_controller:
      type: position_controllers/JointGroupPositionController
    fr_steering_position_controller:
      type: position_controllers/JointGroupPositionController
    rl_steering_position_controller:
      type: position_controllers/JointGroupPositionController
    rr_steering_position_controller:
      type: position_controllers/JointGroupPositionController

    fl_wheel_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
    fr_wheel_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
    rl_wheel_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController
    rr_wheel_velocity_controller:
      type: velocity_controllers/JointGroupVelocityController

fl_steering_position_controller:
  ros__parameters:
    joints: [fl_steering_joint]
fr_steering_position_controller:
  ros__parameters:
    joints: [fr_steering_joint]
rl_steering_position_controller:
  ros__parameters:
    joints: [rl_steering_joint]
rr_steering_position_controller:
  ros__parameters:
    joints: [rr_steering_joint]

fl_wheel_velocity_controller:
  ros__parameters:
    joints: [fl_wheel]
fr_wheel_velocity_controller:
  ros__parameters:
    joints: [fr_wheel]
rl_wheel_velocity_controller:
  ros__parameters:
    joints: [rl_wheel]
rr_wheel_velocity_controller:
  ros__parameters:
    joints: [rr_wheel]
```

### ranger_mini_v3_sim/launch/view_sim_xacro.launch.py

```python
"""Static verification launch for Phase 2.

Prints the fully-expanded sim URDF to stdout. Does NOT start
Gazebo, controller_manager, or robot_state_publisher.

Usage:
    ros2 launch ranger_mini_v3_sim view_sim_xacro.launch.py
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg = FindPackageShare("ranger_mini_v3_sim")

    xacro_path = PathJoinSubstitution([sim_pkg, "urdf", "ranger_mini_v3_sim.xacro"])
    yaml_path  = PathJoinSubstitution([sim_pkg, "config", "ranger_mini_v3_controllers.yaml"])

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                "xacro", xacro_path,
                "controller_yaml:=", yaml_path,
            ],
            output="screen",
        ),
    ])
```

### .claude_handoff/round_04b_phase2_ros2_control.md

This file. Not committed — see Blocked steps.

## Verification output

### Step 0a — Working-tree check (loosened)

```
On branch phase-1-description
Your branch is up to date with 'origin/phase-1-description'.

Untracked files:
	.claude_handoff/round_04_phase2_ros2_control.md

nothing added to commit but untracked files present
```

Untracked Round 04 handoff is the only thing — passes the loosened
check.

### Step 0b — Repoint and update local jazzy

```
=== git fetch origin ===
(no output)
=== git fetch upstream ===
(no output)
=== git branch -u origin/jazzy jazzy ===
branch 'jazzy' set up to track 'origin/jazzy'.
=== git switch jazzy ===
Switched to branch 'jazzy'
Your branch is behind 'origin/jazzy' by 5 commits, and can be fast-forwarded.
=== git reset --hard origin/jazzy ===
HEAD is now at 89c3308 Update ranger_ros2 repository URL in README
=== git log -1 --oneline ===
89c3308 Update ranger_ros2 repository URL in README
=== git branch -vv | head -5 ===
  humble              689a8f3 [upstream/humble: behind 1] update the CalculateSteeringAngle calculation method
* jazzy               89c3308 [origin/jazzy] Update ranger_ros2 repository URL in README
  phase-1-description 56ffca8 [origin/phase-1-description] docs(handoff): rounds 03 / 03b / 03c — phase 1 reset on S-abk jazzy
```

### Step 0c — Delete local humble

```
warning: deleting branch 'humble' that has been merged to
         'refs/remotes/upstream/humble', but not yet merged to HEAD
Deleted branch humble (was 689a8f3).
```

`-d` succeeded with a warning (humble is on the remote, not on local
HEAD). No `-D` fallback needed.

### Step 0d — Verify topology

```
* jazzy               89c3308 [origin/jazzy] Update ranger_ros2 repository URL in README
  phase-1-description 56ffca8 [origin/phase-1-description] docs(handoff): rounds 03 / 03b / 03c …
```

Clean: `jazzy` tracks `origin/jazzy`, `phase-1-description` tracks
its remote, no `humble`.

### Step 1 — Create phase-2 branch

```
Switched to a new branch 'phase-2-ros2-control'
branch 'phase-2-ros2-control' set up to track 'origin/jazzy'.
89c3308 Update ranger_ros2 repository URL in README
```

(Side note: `git checkout -b ... origin/jazzy` set the new branch's
upstream to `origin/jazzy`. That'll be re-pointed at push time when
we use `-u origin phase-2-ros2-control`.)

### Step 2 — Phase-1 URDF reachable on phase-1-description

```
<?xml version="1.0" encoding="utf-8"?>
<!--
  Ranger Mini v3 URDF / xacro
  Ported from agilexrobotics/ugv_gazebo_sim ranger_mini_v3/urdf/ranger_mini.xacro
  for ROS 2 Jazzy + Gazebo Harmonic.
```

### Step 3 — Package directory tree

```
total 20
drwxrwxr-x 5 sao sao 4096 May 12 01:44 .
drwxrwxr-x 9 sao sao 4096 May 12 01:44 ..
drwxrwxr-x 2 sao sao 4096 May 12 01:44 config
drwxrwxr-x 2 sao sao 4096 May 12 01:44 launch
drwxrwxr-x 2 sao sao 4096 May 12 01:44 urdf
```

### Step 4 — package.xml

Maintainer resolved: `Shuaib Olanrewaju <solanrewaju2020@fau.edu>`.
Full file content under "New files" above.

### Step 5–8 — CMakeLists, xacro, controllers.yaml, launch.py

All written verbatim from the prompt. Full contents under "New files"
above.

### Step 9 — Commit phase-2 source

```
=== git status ===
On branch phase-2-ros2-control
Your branch is up to date with 'origin/jazzy'.
Changes to be committed:
	new file:   ranger_mini_v3_sim/CMakeLists.txt
	new file:   ranger_mini_v3_sim/config/ranger_mini_v3_controllers.yaml
	new file:   ranger_mini_v3_sim/launch/view_sim_xacro.launch.py
	new file:   ranger_mini_v3_sim/package.xml
	new file:   ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
Untracked files:
	.claude_handoff/

=== commit ===
[phase-2-ros2-control a3f7289] feat(sim): add ranger_mini_v3_sim package with ros2_control wiring (phase 2)
 5 files changed, 187 insertions(+)
```

### Step 10 — Cross-branch builds (both reported success)

```
=== git checkout phase-1-description ===
Switched to branch 'phase-1-description'
Your branch is up to date with 'origin/phase-1-description'.

=== build phase 1 ===
Starting >>> ranger_mini_v3_description
Finished <<< ranger_mini_v3_description [0.30s]
Summary: 1 package finished [0.39s]

=== git checkout phase-2-ros2-control ===
Switched to branch 'phase-2-ros2-control'
Your branch is ahead of 'origin/jazzy' by 1 commit.

=== source + build phase 2 ===
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.96s]
Summary: 1 package finished [1.05s]
```

But note — see Blocked steps for the catch.

### Step 11 — Install paths

```
=== ros2 pkg prefix ranger_mini_v3_sim ===
/home/sao/agilex_ws/install/ranger_mini_v3_sim
=== ros2 pkg prefix ranger_mini_v3_description ===
/home/sao/agilex_ws/install/ranger_mini_v3_description
```

Both findable (the description's prefix exists; only the contents are
broken via dangling symlinks).

### Step 12 — Static URDF expansion (BLOCKED)

```
exit=2

(stderr)
error: No such file or directory:
       /home/sao/agilex_ws/install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
       [Errno 2] No such file or directory:
       '/home/sao/agilex_ws/install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro'
when processing file: /home/sao/agilex_ws/install/ranger_mini_v3_sim/share/ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
```

Diagnostic confirming the dangling-symlink root cause:

```
$ ls -la install/ranger_mini_v3_description/share/ranger_mini_v3_description/urdf/
ranger_mini_v3.xacro -> /home/sao/agilex_ws/src/ranger_ros2/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro

$ ls -la src/ranger_ros2/ranger_mini_v3_description/urdf/
ls: cannot access '…': No such file or directory   (current branch is phase-2)
```

### Steps 13, 14, 15, 17, 18, 19

Not executed. See Blocked steps.

### Manual verification steps for the operator

The push hasn't been done; nothing to inspect on github.com yet.

## Deviations

- None at the level of action. Every literal command was run; the
  failure surfaced at step 12's verification and the prompt-mandated
  HALT was triggered.

## Open questions

1. **Which fix path for the cross-branch verification — A, B, C, or
   D (see Blocked steps)?** Recommendation: **A** (rebuild description
   without `--symlink-install`, e.g. drop the flag or use
   `--merge-install`). Smallest change, fixes the dangling symlink at
   the root, no extra setup. The phase-2 build itself can keep
   `--symlink-install` since its files don't move during this round.

2. **Should the mid-round colcon `install/` state be wiped before the
   reissued round?** It's currently a mix of (a) cleanly-built
   phase-2 install and (b) phase-1 install with dangling symlinks.
   For a clean reissue I'd suggest `rm -rf build/ install/ log/` at
   the workspace root and let the reissued round rebuild from
   scratch.

3. **Should the local `phase-2-ros2-control` commit (`a3f7289`) be
   kept or reset?** The package source is correct and will not need
   to change once the verification path is fixed. Keeping it means
   the reissued round can resume from step 10 with a fresh build
   strategy. Resetting would re-do step 9 unnecessarily.

4. **Independent of the fix, does the `gz_ros2_control-system`
   plugin filename match what's actually shipped on this Ubuntu 24.04
   / Jazzy / Harmonic install?** Worth confirming with
   `dpkg -L ros-jazzy-gz-ros2-control | grep -i system` before Phase 3
   tries to load it. Not blocking this round, but pre-empting the
   Phase 3 surprise the architect already flagged in their CONTEXT
   note.
