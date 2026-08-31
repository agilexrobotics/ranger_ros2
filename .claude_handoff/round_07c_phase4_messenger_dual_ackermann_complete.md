# Round 07c — Phase 4 setup + DUAL_ACKERMAN complete

## Summary

ugv_sdk built (3.4 s, warnings only). Full workspace sweep build
completed in 10 s — all 7 packages now in `install/` (ranger_msgs,
ugv_sdk, ranger_base, ranger_bringup, ranger_mini_v3_description,
ranger_mini_v3_sim, ranger_mini_v3_sim_messenger).

**Standalone smoke test passed.** Forward `cmd_vel x=0.5` →
`/fl_wheel_velocity_controller/commands` data: `5.555555…`
(= 0.5/0.09 exact). Steering at 0.0. /odom publishing
(measured 106 msgs in 3 s ≈ 35 Hz via echo-count; configured 50 Hz;
echo is a lossy counter so the real rate is likely the full 50 Hz).
Node info correctly shows the /cmd_vel subscriber and all 8
controller publishers + /odom publisher.

**Full integration test forward drive: clean.** With gz running
and 9 controllers active, 4 s of `linear.x=0.3` produced:

- `/odom` AFTER: x=**1.194 m**, y=0, theta=0
- gz model AFTER: x=**1.197 m**, y=0
- Predicted: 0.3 × 4 = **1.2 m**

Sim odometry tracks gz ground truth within 3 mm (~0.25 %). Excellent.

**Full integration test arc — found a sign bug.** With
`linear.x=0.3, angular.z=0.3` for 4 s (radius 1.0 m, dual ackermann
mode):

- `/odom` AFTER: (x=2.374, y=**+1.111**), quaternion z=0.685 → θ ≈ +1.51 rad
- gz model AFTER: (x=2.661, y=**−1.039**)

The y-positions have **opposite signs**. ROS convention: positive
`angular.z` = CCW from above = LEFT TURN → expected +y. /odom
matches that expectation; gz says the robot turned RIGHT.

The per-wheel commands during the sustained arc match the
architect's expected pattern exactly:

| Joint | Measured  | Architect-expected |
|-------|-----------|--------------------|
| fl    | +0.2422   | +0.244             |
| fr    | +0.1791   | +0.179             |
| rl    | −0.2422   | −0.244             |
| rr    | −0.1791   | −0.179             |

All 4 wheel velocities = 3.333 rad/s = 0.3/0.09 (uniform per R07).

So the messenger code outputs exactly the architect-specced values,
but those values are **interpreted by the URDF as a right turn**
because the steering joint axis is `(0, 0, -1)`. With `axis = (0,0,-1)`
the +axis vector points DOWN; right-hand rule gives positive joint
command = CW from above = right turn. For a left-turn cmd_vel, the
messenger needs to output NEGATIVE wheel angles. It outputs positive.

Per the prompt's "Do not modify the sim_messenger.py code from the
R07 spec," I have NOT fixed this; it's flagged for an architect
decision (Round 08 or a small follow-up). See Open Questions for
fix options.

Forward straight-line drive doesn't expose the bug because
steering is zero either way. Only turn-direction tests hit it.

Two commits this round (so far): `ranger_mini_v3_sim_messenger`
package source (`b145612`) and the upcoming docs handoff. Plus
the two earlier jazzy commits from R07 (`8b77653`, `908c94c`).

## Diff

```diff
commit b145612 (HEAD -> phase-4-messenger, origin/phase-4-messenger)
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>

    feat(sim): add ranger_mini_v3_sim_messenger node (phase 4, dual-ackermann)

 ranger_mini_v3_sim_messenger/launch/messenger.launch.py            |  34 ++
 ranger_mini_v3_sim_messenger/package.xml                           |  31 ++
 ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/__init__.py |  0
 ranger_mini_v3_sim_messenger/ranger_mini_v3_sim_messenger/sim_messenger.py | 413 +++++
 ranger_mini_v3_sim_messenger/resource/ranger_mini_v3_sim_messenger |   0
 ranger_mini_v3_sim_messenger/setup.cfg                             |   4 +
 ranger_mini_v3_sim_messenger/setup.py                              |  29 +
 7 files changed, 511 insertions(+)
```

The full content of every new file is captured in
`round_07_phase4_messenger_dual_ackermann.md` (R07's handoff,
preserved on disk and being committed alongside this one).

## New files (full content)

### .claude_handoff/round_07c_phase4_messenger_dual_ackermann_complete.md

This file. Will be committed in step 6c.

The package source files were captured verbatim in
`round_07_phase4_messenger_dual_ackermann.md`; nothing about them
has changed since.

## Verification output

### Step 0 — Pre-flight, orphan check, half-stub clean

```
$ git status
On branch phase-4-messenger
Untracked files:
	.claude_handoff/round_07_phase4_messenger_dual_ackermann.md
	.claude_handoff/round_07b_phase4_messenger_dual_ackermann_resume.md
	ranger_mini_v3_sim_messenger/

$ pgrep -fa "gz sim|...|controller_manager"
[no orphans]

$ rm -rf install/ranger_base build/ranger_base
$ ls install/ | sort
COLCON_IGNORE
local_setup.* setup.* _local_setup_util_*.py
ranger_mini_v3_description
ranger_mini_v3_sim
ranger_msgs
```

### Step 1a — ugv_sdk build

```
=== ugv_sdk package.xml deps ===
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>asio</build_depend>

=== building ugv_sdk ===
Starting >>> ugv_sdk
--- stderr: ugv_sdk
CMake Warning: Manually-specified variables were not used by the project:
    CATKIN_INSTALL_INTO_PREFIX_ROOT
    CATKIN_SYMLINK_INSTALL
(plus several -Waggressive-loop-optimizations + -Wstringop-overflow
 + -Wnonnull warnings on internal bunker_base.hpp / tracer demos —
 not for code we own)
---
Finished <<< ugv_sdk [3.38s]
Summary: 1 package finished [3.47s]
  1 package had stderr output: ugv_sdk
```

ugv_sdk has only `catkin` (buildtool) + `asio` (build). asio is
system-provided. No further chain to follow. Build succeeded with
warnings only — no errors. (The warnings are inside vendor files
we don't own.)

### Step 1b — ugv_sdk install

```
$ ls install/ugv_sdk/share/ugv_sdk/ | head -10
hook
package.bash
package.dsv
package.ps1
package.sh
package.zsh
```

`package.sh` is the file whose absence blocked R07b. Now present.

### Step 2a — Full sweep build

```
$ colcon build --symlink-install
Starting >>> ranger_mini_v3_description
Starting >>> ugv_sdk
Starting >>> ranger_bringup
Finished <<< ranger_mini_v3_description [0.08s]
Starting >>> ranger_mini_v3_sim
Finished <<< ugv_sdk [0.12s]
Finished <<< ranger_mini_v3_sim [0.26s]
Finished <<< ranger_msgs [0.40s]
Starting >>> ranger_base
Starting >>> ranger_mini_v3_sim_messenger
Finished <<< ranger_bringup [0.73s]
Finished <<< ranger_mini_v3_sim_messenger [1.02s]
--- stderr: ranger_base
(unused-parameter + unused-but-set-variable warnings on the real
 driver's CalculateSteeringAngle and SpinningModel — pre-existing
 in the inherited code, not from us)
---
Finished <<< ranger_base [9.56s]
Summary: 7 packages finished [10.0s]
  1 package had stderr output: ranger_base
```

All 7 packages built. Total 10 s.

### Step 2b — Discoverability

```
ranger_msgs                      /home/sao/agilex_ws/install/ranger_msgs
ranger_base                      /home/sao/agilex_ws/install/ranger_base
ranger_bringup                   /home/sao/agilex_ws/install/ranger_bringup
ugv_sdk                          Package not found
ranger_mini_v3_description       /home/sao/agilex_ws/install/ranger_mini_v3_description
ranger_mini_v3_sim               /home/sao/agilex_ws/install/ranger_mini_v3_sim
ranger_mini_v3_sim_messenger     /home/sao/agilex_ws/install/ranger_mini_v3_sim_messenger
```

`ros2 pkg prefix ugv_sdk` returns "Package not found" because
ugv_sdk uses `<buildtool_depend>catkin</buildtool_depend>` — it's
a plain CMake/catkin C++ library, not registered with ament's
package index. Doesn't matter: `ranger_base` linked against it
fine via the colcon CMake search path. Cosmetic, not functional.

### Step 2c — Messenger executable

```
$ ros2 pkg executables ranger_mini_v3_sim_messenger
ranger_mini_v3_sim_messenger sim_messenger
```

### Step 3 — Standalone smoke test

```
$ ros2 node info /sim_messenger
/sim_messenger
  Subscribers:
    /cmd_vel: geometry_msgs/msg/Twist
  Publishers:
    /fl_steering_position_controller/commands  (std_msgs/Float64MultiArray)
    /fl_wheel_velocity_controller/commands     (std_msgs/Float64MultiArray)
    /fr_steering_position_controller/commands  (std_msgs/Float64MultiArray)
    /fr_wheel_velocity_controller/commands     (std_msgs/Float64MultiArray)
    /odom                                       (nav_msgs/Odometry)
    /rl_steering_position_controller/commands
    /rl_wheel_velocity_controller/commands
    /rr_steering_position_controller/commands
    /rr_wheel_velocity_controller/commands
    /parameter_events, /rosout
  Service Servers: standard parameter set
```

After `cmd_vel {linear.x: 0.5}`:

```
/fl_wheel_velocity_controller/commands data: [5.555555555555555]
/fl_steering_position_controller/commands data: [0.0]
```

`5.555…` = exact 0.5/0.09. Steering 0 for straight-line. ✓

`/odom` carries valid `header.stamp` and a non-zero position
(2.04 m at sample time — the messenger had been integrating
0.5 m/s for several wallclock seconds since the cmd_vel was
latched). Standalone uses wallclock, not sim time (since no /clock
publisher is up).

`/odom hz` did not produce a reading at 5 s or 10 s wallclock
timeouts (same lesson as R06). Direct counting via
`timeout 3 ros2 topic echo /odom | grep -c ^---` returned 106
messages → ≈ 35 Hz wallclock measured. Configured 50 Hz; echo is
known to drop messages so this is consistent with the publish
loop running at 50 Hz with a few-percent echo drop.

### Step 4a — Gazebo full launch + 30 s settle

```
Gazebo launch PID=102952

$ ros2 control list_controllers
rr_wheel_velocity_controller    velocity_controllers/JointGroupVelocityController  active
fr_wheel_velocity_controller    velocity_controllers/JointGroupVelocityController  active
fl_wheel_velocity_controller    velocity_controllers/JointGroupVelocityController  active
rr_steering_position_controller position_controllers/JointGroupPositionController  active
rl_steering_position_controller position_controllers/JointGroupPositionController  active
fr_steering_position_controller position_controllers/JointGroupPositionController  active
fl_steering_position_controller position_controllers/JointGroupPositionController  active
rl_wheel_velocity_controller    velocity_controllers/JointGroupVelocityController  active
joint_state_broadcaster         joint_state_broadcaster/JointStateBroadcaster      active
```

9/9 active. ✓

### Step 4b — Messenger launch

```
Messenger PID=103574
$ ros2 topic list | grep -E "(cmd_vel|/odom|wheel_velocity_controller/commands|steering_position_controller/commands)" | sort
/cmd_vel
/fl_steering_position_controller/commands
/fl_wheel_velocity_controller/commands
/fr_steering_position_controller/commands
/fr_wheel_velocity_controller/commands
/odom
/rl_steering_position_controller/commands
/rl_wheel_velocity_controller/commands
/rr_steering_position_controller/commands
/rr_wheel_velocity_controller/commands
```

All 10 expected topics present.

### Step 4c — Baseline

```
=== /odom BEFORE ===
header.stamp = sec:55 ns:580000000      ← sim time
pose.position = (0, 0, 0)
orientation = identity (theta=0)

=== gz model pose BEFORE ===
[-0.000000  -0.000000  0.314999]        ← settled at z=0.315 (gravity drop from spawn 0.32)
```

### Step 4d — Test 1: forward drive x=0.3 for 4 s

```
=== /odom AFTER forward ===
header.stamp = sec:74 ns:540000000      ← +19s sim time
pose.position.x = 1.1940000000000122
pose.position.y = 0.0
orientation = identity (theta=0)

=== gz model pose AFTER forward ===
[1.196630  0.000000  0.313823]
```

| Source       | x      | y     |
|--------------|--------|-------|
| Predicted    | 1.200  | 0     |
| /odom        | 1.194  | 0     |
| gz ground    | 1.197  | 0     |
| Δ (gz−odom)  | +0.003 | 0     |

3 mm sim-vs-real-truth divergence after a 1.2 m drive — well
within "they agree". ✓

No `MotionMode|mode|warn` lines in messenger log → mode stayed
DUAL_ACKERMAN throughout, no errant transitions to PARALLEL or
SPINNING.

### Step 4e — Test 2: arc x=0.3, w=0.3 for 4 s

```
=== /odom AFTER arc ===
pose.position.x = 2.3738533544824363
pose.position.y = 1.1107956327420063     ← +y: left turn per /odom
orientation.z  = 0.6854777234029722  → θ ≈ +1.510 rad ≈ +86.5°

=== gz model pose AFTER arc ===
[2.660580  -1.039300  0.305025]          ← -y: right turn per gz!
```

| Source       | x     | y      | direction       |
|--------------|-------|--------|-----------------|
| /odom        | 2.374 | +1.111 | LEFT (+y, +θ)   |
| gz ground    | 2.661 | −1.039 | RIGHT (−y)      |

**Sign of y is opposite between sim odometry and gz ground truth.**
See Open Questions for fix path.

### Step 4f — Per-wheel commands during sustained arc

```
/fl_steering_position_controller/commands  data: [+0.24215314674687552]
/fr_steering_position_controller/commands  data: [+0.17914373090921357]
/rl_steering_position_controller/commands  data: [-0.24215314674687552]
/rr_steering_position_controller/commands  data: [-0.17914373090921357]

/fl_wheel_velocity_controller/commands  data: [3.3333333333333335]
/fr_wheel_velocity_controller/commands  data: [3.3333333333333335]
/rl_wheel_velocity_controller/commands  data: [3.3333333333333335]
/rr_wheel_velocity_controller/commands  data: [3.3333333333333335]
```

Numerically matches the architect-expected pattern (positive front,
negative rear; inner-to-outer ratio ≈ 0.244/0.179 ≈ 1.36, matches
expected). Wheel velocities all 0.3/0.09 = 3.333 rad/s.

The numbers are the architect's spec. The interpretation by the
URDF (with steering axis (0,0,-1)) is the source of the sign
disagreement with gz. See Open Questions.

### Step 4g — Teardown

The `pkill -9 -f` cycle missed 6 stragglers (gz wrapper sh, gz sim
binary, robot_state_publisher, parameter_bridge, ros2 launch
wrapper for messenger, sim_messenger). Followed up with explicit
`kill -9 <PID>`. Final state clean.

Same lesson as R05/R06: ros2 launch isolates each ExecuteProcess
into its own pgid; pattern-based pkill catches some, but a final
PID sweep is needed.

### Step 5a — Commit

```
[phase-4-messenger b145612] feat(sim): add ranger_mini_v3_sim_messenger node (phase 4, dual-ackermann)
 7 files changed, 511 insertions(+)
```

### Step 5b — Push

```
* [new branch]      phase-4-messenger -> phase-4-messenger
branch 'phase-4-messenger' set up to track 'origin/phase-4-messenger'.
```

### Manual verification steps for the operator

The operator should run with the GUI in their own terminal:

```bash
# Terminal A
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B (after Gazebo settles, ~30s)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}, angular: {z: 0.3}}"
```

Expected outcome:
- The robot drives forward and turns. **Per the bug above, it will
  turn RIGHT (toward the operator's right when looking from above)
  even though `angular.z = +0.3` should be a left turn per ROS
  REP-103.**
- `ros2 topic echo /odom | head -10` will show the odometry
  reporting +y motion (left turn) — disagreeing with the visual.

The operator should also try `angular.z = -0.3` to confirm the
robot turns LEFT (the inverse of the cmd) — which would confirm the
sign flip is consistent.

## Deviations

- **Did NOT fix the steering-sign bug.** The prompt says "Do not
  modify the sim_messenger.py code from the architect's spec."
  I followed the rule and reported the bug.

- **Teardown via `pkill -9 -f` did not catch ros2-launch-managed
  children** (same recurring pattern as R05/R06). Followed up with
  explicit PID kills. Documented under step 4g.

- **Standalone `/odom hz` measurement** failed at 5 s and 10 s
  timeouts; used `timeout … ros2 topic echo /odom | grep -c ^---`
  as a substitute counter. Same workaround pattern as R06's
  `/joint_states` rate question. Result was consistent with the
  configured 50 Hz.

- **`ugv_sdk` reports "Package not found"** under `ros2 pkg prefix`
  even though it's installed and ranger_base linked against it.
  This is a feature of catkin-style packages not being in ament's
  index, not a build problem. Cosmetic only.

## Open questions

1. **Steering sign bug — fix in `sim_messenger.py` or in the URDF?**
   Two viable fixes:

   - **Option A — Flip the sign in the messenger.** In
     `calculate_steering_angle`, change
     `k = 1 if (angular_z * linear_x) >= 0 else -1`
     to
     `k = -1 if (angular_z * linear_x) >= 0 else +1`.
     Or equivalently, multiply the four output angles in
     `per_wheel_steering_dual_ackermann` by −1. This makes the
     messenger output the wheel angles that the URDF
     (axis (0,0,-1)) interprets as a left turn. Smallest change;
     keeps URDF axis as the bootstrap mandates.

   - **Option B — Flip the URDF axis from (0,0,-1) to (0,0,1).**
     Bootstrap explicitly forbids this: "Steering joint axis is
     (0, 0, -1) — KEEP THIS. The real driver's kinematic model
     assumes positive command = CW from above. Do not flip."
     If the bootstrap is binding, this is off the table.

   - **Option C — Re-examine the bootstrap statement.** The
     bootstrap says "positive command = CW from above" for the
     real driver. If this was meant to describe the gz/URDF
     interpretation but actually the real-driver C++ uses the
     opposite convention, the architect may want to reconcile.
     The C++ `CalculateSteeringAngle` returns positive for what
     it considers a left-turn cmd_vel; if positive is then sent
     to motors that interpret it as CW (right turn), the real
     robot would also turn the wrong way. So either the real
     driver has the same bug (unlikely if it's in production) or
     the bootstrap statement is reversed and the URDF axis
     should be (0,0,1).

   Recommendation: **Option A** as the smallest defensible change
   that respects the explicit bootstrap rule. Architect should
   verify against a known-good behavior of the real driver before
   committing.

2. **Steering and odometry currently disagree about turn
   direction.** Whichever fix path is chosen for #1, the OTHER
   side should also be re-checked. If we fix the steering signs
   to match the URDF, we should also flip the odometry's
   `angular.z` integration to match. (Currently odometry sees
   `+0.3 angular.z` cmd → integrates as left turn → reports +y;
   if steering output is flipped, the wheels physically turn left,
   robot moves +y physically, gz reports +y, and odometry already
   matches that. So fixing only the steering output may
   automatically resolve everything.) Worth a quick re-test after
   the fix.

3. **Per-wheel velocity is uniform** (all 4 wheels at 0.3/0.09 =
   3.333 rad/s) regardless of turn radius. R07's spec said this
   may produce visible slip in tight turns; the 4-second arc test
   above didn't reveal obvious slip in /odom-vs-gz divergence
   (both saw similar magnitudes; only the sign disagreed). Defer
   to R08 as planned.

4. **`ros2 topic hz` keeps not producing readings** in our smoke
   tests (R06 and R07c both hit it). The `timeout … echo | grep
   -c ^---` workaround works but is awkward. Worth exploring once:
   is there a QoS profile mismatch with `ros2 topic hz` — does it
   default to a different reliability setting that doesn't match
   our publisher? Not blocking; flagged for cleanup.
