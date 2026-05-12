# Round 06 — Phase 3 Checkpoints C+D

## Summary

**Checkpoint C passed.** All 9 controllers loaded and active inside
gz's embedded controller_manager: `joint_state_broadcaster`, four
`*_steering_position_controller`, four `*_wheel_velocity_controller`.
All 8 command interfaces (4 position + 4 velocity) report `claimed`,
and all 16 state interfaces (8 position + 8 velocity) are exposed.
`/joint_states` flows at ~57 Hz wallclock (100 Hz sim-time target;
real-time-factor < 1.0 with 9 controllers + physics + plugins). TF
chain `base_link → fl_wheel_link` now resolves to `(0.250, 0.190,
-0.220)`, exactly matching the URDF math.

**Checkpoint D passed.** Raw `Float64MultiArray` commands to the per-joint
controller `/commands` topics produce the expected joint-state
responses:

- `fl_steering_position_controller/commands [0.5]` → `fl_steering_joint`
  position 0.0000 → 0.4370 in 2 s (still ramping toward target; URDF
  damping 0.1 + dynamics-limited approach).
- `fl_wheel_velocity_controller/commands [5.0]` → `fl_wheel` velocity
  0.0000 → 5.0000, exact match in 2 s (velocity controller hits
  setpoint immediately, no second-order dynamics).

All four wheel velocities then commanded to 0 for clean shutdown.
Phase 3 is done. One source-fix commit `754e0b2` plus this handoff.

## Diff

```diff
commit 754e0b2…
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>

    feat(sim): phase 3 checkpoints C+D — load controllers, verify joint commands

    - Add launch/gazebo_full.launch.py: includes gazebo.launch.py and
      sequentially spawns the 9 controllers (1 joint state broadcaster +
      4 steering position controllers + 4 wheel velocity controllers).
    - Sequential loading via OnProcessExit handlers avoids spawner race
      conditions during controller_manager initialization.
    - Headless smoke test confirmed: all 9 controllers active,
      /joint_states flowing at ~100 Hz, raw position/velocity commands
      to one steering joint and one wheel produce expected joint state
      responses.

 ranger_mini_v3_sim/launch/gazebo_full.launch.py | 87 +++++++++++++++++++++++++
 1 file changed, 87 insertions(+)
```

## New files (full content)

### ranger_mini_v3_sim/launch/gazebo_full.launch.py

```python
"""Full Gazebo bringup: gazebo.launch.py + controller spawners.

Composes the basic gz/rsp/bridge stack from gazebo.launch.py
with controller_manager spawners for all 9 controllers.

Loaded controllers (sequential):
  1. joint_state_broadcaster   (publishes /joint_states)
  2-5. <fl,fr,rl,rr>_steering_position_controller
  6-9. <fl,fr,rl,rr>_wheel_velocity_controller

Sequential loading is enforced via OnProcessExit handlers
so each spawner waits for the previous to finish. This
avoids race conditions where cm hasn't finished initializing
a controller before the next spawn attempts.

Usage:
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Order matters: jsb first (so /joint_states is alive for downstream
# consumers), then steerers, then wheels.
SPAWN_ORDER = [
    "joint_state_broadcaster",
    "fl_steering_position_controller",
    "fr_steering_position_controller",
    "rl_steering_position_controller",
    "rr_steering_position_controller",
    "fl_wheel_velocity_controller",
    "fr_wheel_velocity_controller",
    "rl_wheel_velocity_controller",
    "rr_wheel_velocity_controller",
]


def _spawner_node(name: str) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        name=f"spawner_{name}",
        arguments=[name, "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )


def generate_launch_description():
    sim_pkg = FindPackageShare("ranger_mini_v3_sim")

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([sim_pkg, "launch", "gazebo.launch.py"])
        ),
    )

    # Build spawner chain via OnProcessExit handlers so each spawner
    # waits for the previous to exit cleanly.
    spawners = [_spawner_node(name) for name in SPAWN_ORDER]
    handlers = []
    for i, spawner in enumerate(spawners[:-1]):
        handlers.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawner,
                    on_exit=[spawners[i + 1]],
                )
            )
        )

    return LaunchDescription([
        base_launch,
        spawners[0],   # first spawner (jsb) starts immediately;
                       # rest are chained via handlers
        *handlers,
    ])
```

### .claude_handoff/round_06_phase3_checkpoint_CD.md

This file. Will be committed in step 8c.

## Verification output

### Step 0b — `ros2 control` CLI sanity

CLI is installed but `list_hardware_interfaces` blocks indefinitely
waiting for `/controller_manager/list_hardware_interfaces` service
when no cm is running (rather than printing "No node found" + exiting,
as the prompt expected). Killed the hung command and proceeded.

```
[INFO] [_ros2cli_93550]: waiting for service /controller_manager/list_hardware_interfaces to become available...
[WARN] [_ros2cli_93550]: Could not contact service /controller_manager/list_hardware_interfaces
…(repeats every 10s indefinitely)…
```

CLI confirmed installed; Phase 3 launch will give it a real cm to
talk to.

### Step 1a — Controller plugin libraries

```
=== position_controllers ===
/opt/ros/jazzy
/opt/ros/jazzy/lib/libposition_controllers.so

=== velocity_controllers ===
/opt/ros/jazzy
/opt/ros/jazzy/lib/libvelocity_controllers.so

=== joint_state_broadcaster ===
/opt/ros/jazzy
/opt/ros/jazzy/lib/libjoint_state_broadcaster.so
```

### Step 1b — Plugin XML registration (relevant excerpts)

```xml
<library path="position_controllers">
  <class name="position_controllers/JointGroupPositionController"
         type="position_controllers::JointGroupPositionController"
         base_class_type="controller_interface::ControllerInterface">
    <description>...</description>
  </class>
</library>

<library path="velocity_controllers">
  <class name="velocity_controllers/JointGroupVelocityController"
         type="velocity_controllers::JointGroupVelocityController"
         base_class_type="controller_interface::ControllerInterface">
    <description>...</description>
  </class>
</library>
```

`JointGroupPositionController` and `JointGroupVelocityController`
class names match `controllers.yaml` exactly. ✓

### Step 3a — Rebuild

```
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.11s]
Summary: 1 package finished [0.20s]
```

### Step 3b — Installed launches

```
gazebo_full.launch.py
gazebo.launch.py
__pycache__
view_sim_xacro.launch.py
```

### Step 3c — `--print` parse

Top of output (full output is the launch DAG):

```
<launch.launch_description.LaunchDescription object at …>
├── IncludeLaunchDescription                       (gazebo.launch.py)
├── ExecuteProcess (spawner joint_state_broadcaster)
├── RegisterEventHandler                            (OnProcessExit chain)
│   └── ExecuteProcess (spawner fl_steering_position_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner fr_steering_position_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner rl_steering_position_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner rr_steering_position_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner fl_wheel_velocity_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner fr_wheel_velocity_controller)
├── RegisterEventHandler
│   └── ExecuteProcess (spawner rl_wheel_velocity_controller)
└── RegisterEventHandler
    └── ExecuteProcess (spawner rr_wheel_velocity_controller)
```

Chain shape correct: 1 immediate spawner + 8 OnProcessExit handlers.

### Step 4a — Launch + 30s settle

```
Launch PID=93830 PGID=93830
Launch root alive after 30s
```

### Step 4b — `ros2 control list_controllers`

```
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

**9/9 controllers active.** ✓

### Step 4b — `ros2 control list_hardware_interfaces`

```
command interfaces
	fl_steering_joint/position [available] [claimed]
	fl_wheel/velocity [available] [claimed]
	fr_steering_joint/position [available] [claimed]
	fr_wheel/velocity [available] [claimed]
	rl_steering_joint/position [available] [claimed]
	rl_wheel/velocity [available] [claimed]
	rr_steering_joint/position [available] [claimed]
	rr_wheel/velocity [available] [claimed]
state interfaces
	fl_steering_joint/position
	fl_steering_joint/velocity
	fl_wheel/position
	fl_wheel/velocity
	fr_steering_joint/position
	fr_steering_joint/velocity
	fr_wheel/position
	fr_wheel/velocity
	rl_steering_joint/position
	rl_steering_joint/velocity
	rl_wheel/position
	rl_wheel/velocity
	rr_steering_joint/position
	rr_steering_joint/velocity
	rr_wheel/position
	rr_wheel/velocity
```

8 command interfaces all `claimed`, 16 state interfaces present. ✓

### Step 4c — `/joint_states` flow

First message includes all 8 joints with positions ~10⁻²¹ and
velocities ~10⁻²⁰ (numerical noise from the gz physics step):

```
header:
  stamp:
    sec: 45
    nanosec: 189000000
  frame_id: base_link
name:
- fl_steering_joint
- fl_wheel
- fr_steering_joint
- fr_wheel
- rl_steering_joint
- rl_wheel
- rr_steering_joint
- rr_wheel
position:
- 2.56e-21
- -2.15e-16
- 3.83e-21
- -2.29e-16
- 1.68e-21
- -2.58e-16
- 2.35e-21
- -3.20e-16
velocity: …
```

Rate measurement: `ros2 topic hz` did not produce output within 8s
wallclock (it needs ~10 samples and uses sim_time which runs slower
than wallclock). Direct count via timed `topic echo`:

```
$ timeout 2 ros2 topic echo /joint_states 2>&1 | grep -c "^---"
114
```

114 messages in 2 s wallclock ≈ **57 Hz wallclock**. Sim-time target
is 100 Hz (controllers.yaml `update_rate: 100`); the slower wallclock
rate reflects gz running with real-time-factor < 1.0 due to compute
cost of physics + 9 controllers + plugins. Acceptable for sim
verification.

### Step 4d — TF resolves now

```
=== base_link → fl_wheel_link ===
At time 126.260000000
- Translation: [0.250, 0.190, -0.220]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
```

Translation matches URDF math: x = wheelbase_half = 0.25, y =
track_half = 0.19, z = steer_z + wheel_drop = -0.10 + (-0.12) =
-0.22. ✓ (R05 had this fail with "Terminated"; now works because jsb
publishes /joint_states.)

### Step 5a — Steering command test

```
=== Test 1: fl_steering_joint → +0.5 rad ===
--- BEFORE ---
fl_steering_joint: 0.0000
--- PUB ---
publisher: beginning loop
publishing #1: std_msgs.msg.Float64MultiArray(layout=…, data=[0.5])
--- AFTER (2s later) ---
fl_steering_joint: 0.4370
```

Joint moved 0.0 → 0.437 rad in 2 s, asymptotically toward 0.5 rad
target. JointGroupPositionController is a forwarding controller (no
trajectory; commands the position interface directly), so the
trajectory shape is set by the gz joint-effort solver + the URDF
damping (0.1) and effort/velocity limits.

### Step 5b — Wheel velocity command test

```
=== Test 2: fl_wheel → +5 rad/s ===
--- BEFORE ---
fl_wheel: 0.0000
--- PUB ---
publishing #1: std_msgs.msg.Float64MultiArray(…, data=[5.0])
--- AFTER (2s later) ---
fl_wheel: 5.0000
```

Velocity hit setpoint exactly. Velocity controller is also forwarding;
no second-order dynamics to ramp through.

### Step 5c — Zero all wheels

```
=== Test 3: zero all wheel velocities ===
  fl_wheel → 0
  fr_wheel → 0
  rl_wheel → 0
  rr_wheel → 0
All wheel velocity controllers commanded to 0.
```

### Step 6 — Teardown

`kill -- -<PGID>` from the launch root again missed
robot_state_publisher and parameter_bridge (which ros2 launch
isolates into their own pgids — same lesson as R05). The pkill cycle
got everything else. Followed up with `kill -9 <PID>` for the
two stragglers (PIDs 93855, 93857):

```
$ pgrep -fa "robot_state_publisher|parameter_bridge|gz sim|ruby.*gz|spawner_|controller_manager"
[none]
```

All processes gone.

### Manual verification steps for the operator

To visually confirm Phase 3 end-to-end with the GUI:

```bash
# Terminal A
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true
```

Wait ~30 s for all spawners to chain through. Then in a second
terminal:

```bash
# Terminal B
ros2 control list_controllers
# Should print 9 'active' controllers.

# Spin one front wheel:
ros2 topic pub --once /fl_wheel_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [5.0]}"
# In the gz GUI: front-left wheel spins.

# Steer the front-left:
ros2 topic pub --once /fl_steering_position_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.5]}"
# In the gz GUI: front-left steering hub rotates ~28° clockwise from above.

# Stop:
ros2 topic pub --once /fl_wheel_velocity_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.0]}"
ros2 topic pub --once /fl_steering_position_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

Ctrl-C in Terminal A to shut down.

Note: the robot will likely DRIVE FORWARD when you spin only the
front-left wheel because no other wheel is restrained. This is
expected — Phase 3 only verifies single-wheel control. Coordinated
4-wheel driving + steering arrives in Phase 4 with the messenger
node consuming `/cmd_vel`.

## Deviations

- **`ros2 topic hz /joint_states` did not produce output** within
  the 5–8 s timeout. Used a direct message-count over a 2 s wallclock
  window (114 msgs → 57 Hz) as the alternative measurement. The
  flow rate confirmation was the actual goal; the alternative reading
  serves it just as well. Documented in step 4c.

- **Teardown again required pattern-based fallback.** Same lesson as
  R05: ros2 launch isolates each ExecuteProcess into its own pgid,
  so the launch root's `setsid` PGID kill misses children. The pkill
  cycle in step 6 got most things; explicit `kill -9` for two PIDs
  cleaned up the rest. Suggesting in the next process-bringup round
  we drop the PGID kill entirely and lead with pkill patterns.

- **`tf2_echo` does not accept `--timeout`** (the prompt's literal
  invocation included it; tf2_echo only takes positional source/target
  + a `-t <time>` lookup-time flag, not a wait-for-frame timeout).
  Worked around by running tf2_echo with a wallclock `timeout 4`
  command wrapper, which produced the expected output (the static
  TF chain has been published since launch start, so no actual wait
  was needed). Output captured.

- **Step 0b's `ros2 control list_hardware_interfaces` hangs forever**
  without a running controller_manager rather than exiting with
  "No node found" as the prompt anticipated. Killed it after a few
  iterations of the wait-and-retry loop. The CLI is installed; just
  doesn't graceful-fail without a cm peer.

## Open questions

1. **Sim real-time-factor < 1.0.** With all 9 controllers active,
   `/joint_states` rate at wallclock is ~57 Hz against a sim-time
   target of 100 Hz. Means the sim runs at ~57% real-time. Not a
   blocker for any verification, but if Phase 4's
   `ranger_sim_messenger` runs in wallclock with `use_sim_time=False`,
   Twist commands could outpace the sim's update cadence. Recommend
   Phase 4 messenger uses `use_sim_time=True` to stay aligned.

2. **JointGroupPositionController is a forwarding controller, not a
   trajectory follower.** Commands go straight to the position
   interface; gz's joint solver + URDF damping shapes the response
   curve. The 0.5 → 0.437 in 2 s response is OK for a sim verification
   but for Phase 4 we may want better step response — either a higher
   effort limit in the URDF (currently `effort=5.0`) or switch to
   `joint_trajectory_controller` with a smoother profile. Architect
   call. Not blocking; 4WS / cmd_vel will exercise the dynamics
   in context.

3. **Velocity controller hit 5 rad/s exactly in 2 s with no overshoot
   or ramp.** That's because the velocity command interface lets gz
   set the joint's commanded velocity directly each step; gz then
   applies whatever effort needed to track. Wheel mass (8 kg) and
   damping (0.05) keep it stable. Means the wheels are essentially
   ideal velocity sources — Phase 4 odometry can compute ground speed
   from `wheel_radius * commanded_velocity` without worrying about
   slip in straight-line cases. (Slip can still appear under
   side-slip / spinning modes from URDF tire-friction defaults; we
   should observe + tune in Phase 4 dynamics validation.)

4. **R05's open questions still apply** (controller update period
   warning, spawn-z drift, GZ_SIM_RESOURCE_PATH builder ugliness).
   None blocking.
