# Round 07 — Phase 4 setup + DUAL_ACKERMAN

## Blocked steps

**Step 5a (`colcon build --packages-select ranger_mini_v3_sim_messenger
--symlink-install`) failed** because the package declares
`<exec_depend>ranger_msgs</exec_depend>` and `ranger_msgs` is not
present in the workspace's `install/` tree. Per the no-silent-route-
arounds rule, halting rather than building `ranger_msgs` implicitly.
Steps 5b–9d not executed.

### What ran

```
$ colcon build --packages-select ranger_mini_v3_sim_messenger --symlink-install
Starting >>> ranger_mini_v3_sim_messenger
[0.135s] ERROR:colcon.colcon_ros.task.ament_python.build:Failed to find the following files:
- /home/sao/agilex_ws/install/ranger_msgs/share/ranger_msgs/package.sh
Check that the following packages have been built:
- ranger_msgs
Failed   <<< ranger_mini_v3_sim_messenger [0.00s, exited with code 1]

Summary: 0 packages finished [0.10s]
  1 package failed: ranger_mini_v3_sim_messenger
```

### Diagnosis

- `ranger_msgs/` source exists at
  `~/agilex_ws/src/ranger_ros2/ranger_msgs/` (sibling package in the
  fork). It contains `CMakeLists.txt`, `package.xml`, and `msg/`.
- `ranger_msgs` has never been built in this workspace's `install/`
  tree (we've been doing `--packages-select` on our own packages
  only since R04).
- Our new `ranger_mini_v3_sim_messenger/package.xml` lists
  `ranger_msgs` as `exec_depend` per the architect's spec — correct
  forward-looking declaration since R09 will publish
  `ranger_msgs/SystemState`, `MotionState`, `ActuatorStateArray`.
- ament_python's build task does a pre-flight check that all
  declared deps' `package.sh` files exist in `install/`, even with
  `--packages-select`. Without them, the build aborts at 0.135 s.

### What needs to happen to unblock

Two reasonable paths; architect picks:

**Option A (recommended): build `ranger_msgs` once, then resume.**
A single `colcon build --packages-select ranger_msgs --symlink-install`
makes `install/ranger_msgs/share/ranger_msgs/package.sh` exist, after
which `ranger_mini_v3_sim_messenger` builds. `ranger_msgs` is a pure
message package (CMake + msg files); building it doesn't modify it
and doesn't violate the "do not edit ranger_msgs" rule (which is
about source mutation, not about generating bindings).

**Option B: temporarily drop `ranger_msgs` from package.xml** since
this round's code doesn't actually `import` it (PARALLEL/SPINNING
modes only zero commands; no message construction yet). Add it back
in R09 when `/system_state` etc. land. This avoids the workspace-
build dance but contradicts the architect's explicit dep list.

Recommendation: **Option A** is the truthful state — we will need
ranger_msgs in install/ from R09 onward anyway. Building it once now
is the simpler permanent fix. The two-line follow-up prompt is just
"build ranger_msgs first, then re-run from step 5a".

### What state the local repo + workspace are in

- `jazzy` branch:
  - `8b77653 feat(sim): integrate ranger_mini_v3_sim into jazzy (phases 2+3)`
  - `908c94c fix(sim): URDF dimensions match real-robot RangerMiniV3Params`

  Both committed AND pushed to `origin/jazzy` (steps 0d, 0e, 1c, 1d
  ran exactly as instructed by the prompt before the build attempt).
  These are intentional, prompt-mandated state changes — not affected
  by the halt.

- `phase-4-messenger` branch:
  - Created locally off updated jazzy (step 2a). Currently at
    `908c94c` (jazzy tip; no commits added yet).
  - **Uncommitted** on disk: the entire
    `ranger_mini_v3_sim_messenger/` package source tree
    (package.xml, setup.py, setup.cfg, resource marker,
    ranger_mini_v3_sim_messenger/__init__.py, sim_messenger.py,
    launch/messenger.launch.py).
  - **Not pushed** to origin.

- `install/` tree: contains
  `ranger_mini_v3_description`, `ranger_mini_v3_sim`,
  `ranger_mini_v3_sim_messenger` (the last is from a prior partial
  attempt at colcon — the build failed at the dep check, but the
  install dir already exists from a past run).

  The `ranger_mini_v3_description` install was rebuilt with the
  updated URDF dimensions in step 1b — that's correct.

### What I deliberately did NOT do

- Did NOT run `colcon build --packages-select ranger_msgs
  --symlink-install` to fix the dep before retrying.
- Did NOT edit `ranger_mini_v3_sim_messenger/package.xml` to drop
  `ranger_msgs`.
- Did NOT commit the messenger package source files. They sit
  uncommitted on `phase-4-messenger` until the build succeeds.
- Did NOT push `phase-4-messenger`.
- Did NOT execute the static smoke test (step 6) or the full bringup
  integration test (step 7) — both depend on the messenger node
  being importable, which depends on the build.
- Did NOT execute the handoff commit / push (steps 9c, 9d) for this
  round.

---

## Summary

Phase-4 setup work proceeded smoothly through step 4 then halted at
step 5a's colcon build:

- Squash-merged `phase-2-ros2-control` into `jazzy` (commit
  `8b77653`, 15 files / 3353 insertions including the
  ranger_mini_v3_sim package + R04–R06 handoffs). Pushed.
- Updated URDF wheelbase (0.50 → 0.494 m) and track (0.38 → 0.364 m)
  to match real-driver `RangerMiniV3Params` (commit `908c94c`).
  Description rebuilt; expanded URDF confirms steering origins now
  at x=±0.247, y=±0.182. Pushed.
- Created `phase-4-messenger` branch off updated jazzy.
- Wrote the full `ranger_mini_v3_sim_messenger/` package on disk:
  `package.xml`, `setup.py`, `setup.cfg`, resource marker,
  `__init__.py`, `sim_messenger.py` (~330 lines, dual-Ackermann +
  RK4 odometry), `launch/messenger.launch.py`.
- Build of the new package failed because `ranger_msgs` (a declared
  `exec_depend`) isn't in `install/`. See `## Blocked steps`.

The package source itself is correct as far as I can tell from
review (it parses as Python, follows the architect's spec line by
line); the failure is purely a workspace-state issue.

## Diff

Two commits on `jazzy`, both already pushed:

```
908c94c fix(sim): URDF dimensions match real-robot RangerMiniV3Params
8b77653 feat(sim): integrate ranger_mini_v3_sim into jazzy (phases 2+3)
```

The URDF fix's diff:

```diff
--- a/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
+++ b/ranger_mini_v3_description/urdf/ranger_mini_v3.xacro
@@ -21,8 +21,8 @@
-  <xacro:property name="wheelbase_half" value="0.25"/>
-  <xacro:property name="track_half"     value="0.19"/>
+  <xacro:property name="wheelbase_half" value="0.247"/>
+  <xacro:property name="track_half"     value="0.182"/>
```

The squash-merge's stat (full file list omitted — it's the
ranger_mini_v3_sim package + R04–R06 handoffs from
phase-2-ros2-control):

```
 .claude_handoff/CONTEXT.md                              |  68 +++
 .claude_handoff/round_04_phase2_ros2_control.md         | 210 +++++
 .claude_handoff/round_04b_phase2_ros2_control.md        | 617 +++++++++++++
 .claude_handoff/round_04c_squash_merge_phase1…          | 480 ++++++++
 .claude_handoff/round_04d_phase3_prereqs.md             | 325 ++++++
 .claude_handoff/round_05_phase3_checkpoint_AB.md        | 644 ++++++++++++
 .claude_handoff/round_06_phase3_checkpoint_CD.md        | 534 ++++++++++
 ranger_mini_v3_sim/CMakeLists.txt                       |  11 +
 ranger_mini_v3_sim/config/ranger_mini_v3_controllers.yaml | 50 ++
 ranger_mini_v3_sim/launch/gazebo.launch.py              | 142 +++
 ranger_mini_v3_sim/launch/gazebo_full.launch.py         |  87 ++
 ranger_mini_v3_sim/launch/view_sim_xacro.launch.py      |  30 +
 ranger_mini_v3_sim/package.xml                          |  32 +
 ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro        |  64 ++
 ranger_mini_v3_sim/worlds/empty_ground.sdf              |  59 ++
 15 files changed, 3353 insertions(+)
```

No commit on `phase-4-messenger` yet (the messenger package source
is uncommitted; it would have been the next commit if step 5a had
succeeded).

## New files (full content)

The messenger package source is on disk but uncommitted. The full
content of each file is captured below for the architect's review;
once unblocked I'll commit exactly these files.

### ranger_mini_v3_sim_messenger/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ranger_mini_v3_sim_messenger</name>
  <version>0.1.0</version>
  <description>
    Twist-to-controller-commands messenger for the Ranger Mini v3
    Gazebo simulation. Mirrors the real driver's ROS interface
    (ranger_base/ranger_messenger.cpp) so application code is
    sim/real-portable.
  </description>
  <maintainer email="solanrewaju2020@fau.edu">Shuaib Olanrewaju</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>ranger_msgs</exec_depend>

  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### ranger_mini_v3_sim_messenger/setup.py

```python
from setuptools import find_packages, setup

package_name = 'ranger_mini_v3_sim_messenger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/messenger.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shuaib Olanrewaju',
    maintainer_email='solanrewaju2020@fau.edu',
    description='Twist-to-controllers messenger for Ranger Mini v3 sim.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_messenger = ranger_mini_v3_sim_messenger.sim_messenger:main',
        ],
    },
)
```

### ranger_mini_v3_sim_messenger/setup.cfg

```ini
[develop]
script_dir=$base/lib/ranger_mini_v3_sim_messenger
[install]
install_scripts=$base/lib/ranger_mini_v3_sim_messenger
```

### ranger_mini_v3_sim_messenger/resource/ranger_mini_v3_sim_messenger

Empty marker file (0 bytes, ament_python convention).

### ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/__init__.py

Empty (0 bytes).

### ranger_mini_v3_sim_messenger/launch/messenger.launch.py

```python
"""Launch the SimMessenger node.

Use this with gazebo_full.launch.py running in another terminal:

    # Terminal A
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py

    # Terminal B
    ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

Or compose them in a single launch in Phase 5.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ranger_mini_v3_sim_messenger',
            executable='sim_messenger',
            name='sim_messenger',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'update_rate': 50,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom',
                'publish_odom_tf': False,
            }],
        ),
    ])
```

### ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py

Identical to the architect's spec, with one trivial cleanup: the
`_mode_warned` attribute is initialized to `None` in `__init__`
(rather than tested for existence with `hasattr` in the tick), so
PyLint doesn't squawk and the suppression behavior is identical.
~330 lines. Constants (WHEELBASE 0.494, TRACK 0.364, etc.),
`MotionMode` IntEnum, `WheelCommands` dataclass, kinematic helpers
(`calculate_steering_angle`, `inner_to_central`,
`per_wheel_steering_dual_ackermann`,
`compute_wheel_commands_dual_ackermann`), `SimMessenger` Node class
with `_cmd_cb`, `_tick`, `_integrate_dual_ackermann` (RK4, 10
substeps), `_publish_wheel_commands`, `_publish_odometry`,
`_yaw_to_quat`. Full text matches the spec — happy to paste the
verbatim contents into the next handoff once it's committed.

### .claude_handoff/round_07_phase4_messenger_dual_ackermann.md

This file. Not committed — see Blocked steps.

## Verification output

### Step 0a — Pre-flight

```
On branch phase-2-ros2-control
Your branch is up to date with 'origin/phase-2-ros2-control'.
nothing to commit, working tree clean
```

### Step 0b — Handoff state

All R04–R06 handoffs already committed. Nothing to save.

### Step 0c — Squash-merge

```
Switched to branch 'jazzy'
Updating 8e3a835..2b746db
Fast-forward
Squash commit -- not updating HEAD
 (15 files / 3353 insertions; full list under Diff)
```

15 `A` entries staged.

### Step 0d — Squash commit

```
[jazzy 8b77653] feat(sim): integrate ranger_mini_v3_sim into jazzy (phases 2+3)
 15 files changed, 3353 insertions(+)
```

### Step 0e — Push jazzy

```
To github.com:S-abk/ranger_ros2.git
   8e3a835..8b77653  jazzy -> jazzy
```

### Step 1a — URDF dim edit

```
$ grep -E 'wheelbase_half|track_half' …xacro
  <xacro:property name="wheelbase_half" value="0.247"/>
  <xacro:property name="track_half"     value="0.182"/>
```

### Step 1b — Description rebuild + URDF re-expand

```
Starting >>> ranger_mini_v3_description
Finished <<< ranger_mini_v3_description [0.09s]
Summary: 1 package finished [0.18s]
exit=0
```

xacro re-expansion clean. The grep pattern in the prompt missed the
attribute order (xacro emits `rpy="..." xyz="..."` not vice versa);
re-grepping with the right anchor confirms the four steering origins:

```
fr_steering_joint:  <origin rpy="0 0 0" xyz="0.247 -0.182 -0.1"/>
fl_steering_joint:  <origin rpy="0 0 0" xyz="0.247 0.182 -0.1"/>
rl_steering_joint:  <origin rpy="0 0 0" xyz="-0.247 0.182 -0.1"/>
rr_steering_joint:  <origin rpy="0 0 0" xyz="-0.247 -0.182 -0.1"/>
```

All four steering origins at x=±0.247, y=±0.182. ✓

### Step 1c — Commit URDF fix

```
[jazzy 908c94c] fix(sim): URDF dimensions match real-robot RangerMiniV3Params
 1 file changed, 2 insertions(+), 2 deletions(-)
```

### Step 1d — Push

```
To github.com:S-abk/ranger_ros2.git
   8b77653..908c94c  jazzy -> jazzy
```

### Step 2a — phase-4-messenger branch

```
Switched to a new branch 'phase-4-messenger'
908c94c fix(sim): URDF dimensions match real-robot RangerMiniV3Params
```

### Step 3a — Package directory tree

```
ranger_mini_v3_sim_messenger/
├── launch/
├── ranger_mini_v3_sim_messenger/
├── resource/
└── test/
```

### Steps 3b–3f, 4a–4b

Files written to disk. Content captured under "New files" above.
All on disk in `ranger_mini_v3_sim_messenger/`, uncommitted.

### Step 5a — Build (BLOCKED)

```
Starting >>> ranger_mini_v3_sim_messenger
[0.135s] ERROR:colcon.colcon_ros.task.ament_python.build:Failed to find the following files:
- /home/sao/agilex_ws/install/ranger_msgs/share/ranger_msgs/package.sh
Check that the following packages have been built:
- ranger_msgs
Failed   <<< ranger_mini_v3_sim_messenger [0.00s, exited with code 1]

Summary: 0 packages finished [0.10s]
  1 package failed: ranger_mini_v3_sim_messenger
```

See `## Blocked steps`.

### Steps 5b, 6, 7, 8, 9

Not executed.

### Manual verification steps for the operator

The full bringup integration test (step 7) was the visible payoff
for this round and never ran. Once the architect unblocks (likely
"build ranger_msgs first"), the round resumes from step 5a and
ends with the operator running:

```bash
# Terminal A
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

Robot expected to drive forward in a left-turning arc.

## Deviations

- None at the level of action. Steps 0–4 ran exactly as the prompt
  specified. Step 5a's literal command failed; halted per the rule.

- One observation: the `grep -E 'origin xyz="(0\.247…)'` in step 1b
  matched zero lines because xacro outputs `rpy="..." xyz="..."`
  (rpy first), not the other way around. The verification still
  passed via the alternative anchor; reporting because future
  rounds may want to grep for `xyz=` separately.

## Open questions

1. **Path to unblock the build: A or B?** Recommendation: Option A
   — a one-time `colcon build --packages-select ranger_msgs
   --symlink-install`. Building ranger_msgs doesn't violate the
   "do not edit ranger_msgs" rule (which is about source mutation).
   Option B (drop the dep, add it back later) is fragile and
   contradicts the architect's spec.

2. **Should the round re-issue include a one-time build of all
   sibling packages we'll eventually depend on?** `ranger_base`
   and `ranger_bringup` are also in the workspace; they don't get
   pulled in transitively because we don't depend on them, but a
   sweep build (`colcon build --symlink-install`) would catch
   anything else missing. Worth considering for forward-portability
   if Phase 5 imports more from those packages.

3. **The `install/` tree already contains a stub
   `install/ranger_mini_v3_sim_messenger/` directory** from a prior
   partial colcon attempt. It's not blocking the build error
   (the error is about the dep, not about that stub), but a
   clean `rm -rf build/ install/ranger_mini_v3_sim_messenger
   log/latest_build/` before resuming might give cleaner logs.
   Optional.
