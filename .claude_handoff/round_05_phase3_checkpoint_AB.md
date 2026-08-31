# Round 05 — Phase 3 Checkpoints A+B

## Summary

**Checkpoint A passed.** Fixed the gz plugin class name in
`ranger_mini_v3_sim.xacro` from `GazeboSimROS2ControlSystem` to
`GazeboSimROS2ControlPlugin` (the symbol actually exported by
`libgz_ros2_control-system.so` per R04d's `nm` inspection). xacro
re-expands cleanly (exit 0); installed URDF carries the new name.

**Checkpoint B passed.** Added `worlds/empty_ground.sdf` (sun, ground
plane, four system plugins) and `launch/gazebo.launch.py` (gz sim
headless by default, `/clock` bridge, robot_state_publisher,
`ros_gz_sim create` to spawn the robot at z=0.32). Updated
`CMakeLists.txt` to install the new `worlds/` directory.

Headless smoke test: `ros2 launch ranger_mini_v3_sim gazebo.launch.py`
ran for 15 s. `gz model --list` returned `ground_plane` AND
`ranger_mini_v3` (settled at z=0.315 — just below the spawn 0.32 due
to ~0.005 m gravity drop). The four expected ROS topics all live:
`/clock`, `/robot_description`, `/tf`, `/tf_static`. `/clock` echoes
real timestamps. Static TF `base_link → base_footprint` resolves to
`(0, 0, -0.310)` matching the URDF math.

**Surprise (not a failure):** `gz_ros_control` and `controller_manager`
nodes appear in `ros2 node list` even though the launch file only
starts gz, rsp, and the bridge. That's correct behavior — the
`GazeboSimROS2ControlPlugin` we wired into the xacro's `<gazebo>`
block embeds controller_manager INSIDE the gz process when gz loads
the model. So "no controllers loaded" is still true (no spawners
ran), but cm itself is alive and waiting. Round 06 will spawn
controllers against this already-running cm.

One non-blocking warning in the gz log:
`Desired controller update period (0.01 s) is slower than the gazebo
simulation period (0.001 s).` — our controllers.yaml has
`update_rate: 100` Hz; gz ticks at 1000 Hz. That's fine; cm just
won't tick every gz step.

Single source-fix commit `89621df` plus this handoff.

## Diff

```diff
commit 89621df…
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>

    feat(sim): phase 3 checkpoints A+B — gz bringup launch

    - Fix gz_ros2_control plugin class name (ControlSystem → ControlPlugin)
      to match the symbol exported by libgz_ros2_control-system.so
      (verified via nm in Round 04d).
    - Add worlds/empty_ground.sdf: minimal world with sun, ground plane,
      default physics, and the four system plugins gz sim needs.
    - Add launch/gazebo.launch.py: spawns the robot in gz at (0, 0, 0.32),
      starts robot_state_publisher with the expanded URDF, and a /clock
      bridge. Does NOT yet load controllers — checkpoint C+D follows.

 ranger_mini_v3_sim/CMakeLists.txt                |   2 +-
 ranger_mini_v3_sim/launch/gazebo.launch.py       | 142 +++++++++++++++++++++++
 ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro |   2 +-
 ranger_mini_v3_sim/worlds/empty_ground.sdf       |  59 ++++++++++
 4 files changed, 203 insertions(+), 2 deletions(-)
```

The two single-line modifications:

```diff
--- a/ranger_mini_v3_sim/CMakeLists.txt
+++ b/ranger_mini_v3_sim/CMakeLists.txt
@@ -6,5 +6,5 @@ find_package(ament_cmake REQUIRED)
 install(
-  DIRECTORY urdf config launch
+  DIRECTORY urdf config launch worlds
   DESTINATION share/${PROJECT_NAME}
 )
```

```diff
--- a/ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
+++ b/ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
@@ -57,5 +57,5 @@
   <gazebo>
-    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlSystem">
+    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
       <parameters>$(arg controller_yaml)</parameters>
     </plugin>
   </gazebo>
```

## New files (full content)

### ranger_mini_v3_sim/worlds/empty_ground.sdf

```xml
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="empty_ground">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light name="sun" type="directional">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
            <specular>0.8 0.8 0.8 1</specular>
          </material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
```

### ranger_mini_v3_sim/launch/gazebo.launch.py

```python
"""Gazebo bringup for Ranger Mini v3 (Phase 3 checkpoint B).

Starts gz sim with the ground-plane world, spawns the robot
at (0, 0, 0.32), starts robot_state_publisher and a /clock
bridge. Does NOT load ros2_control controllers — that's
added in a follow-up launch.

Args:
    gui (bool, default false):  run gz sim with GUI
    world (str, default empty_ground.sdf): world file name
    spawn_x, spawn_y, spawn_z (float): spawn pose
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_pkg  = FindPackageShare("ranger_mini_v3_sim")
    gz_sim   = FindPackageShare("ros_gz_sim")

    xacro_path = PathJoinSubstitution([sim_pkg, "urdf", "ranger_mini_v3_sim.xacro"])
    yaml_path  = PathJoinSubstitution([sim_pkg, "config", "ranger_mini_v3_controllers.yaml"])
    world_path = PathJoinSubstitution([sim_pkg, "worlds", LaunchConfiguration("world")])

    gui    = LaunchConfiguration("gui")
    x      = LaunchConfiguration("spawn_x")
    y      = LaunchConfiguration("spawn_y")
    z      = LaunchConfiguration("spawn_z")

    gz_resource_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PythonExpression([
            "'", PathJoinSubstitution([FindPackageShare("ranger_mini_v3_description"), ".."]), "'",
            " + ':' + '",
            PathJoinSubstitution([FindPackageShare("ranger_mini_v3_sim"), ".."]), "'",
            " + ':' + __import__('os').environ.get('GZ_SIM_RESOURCE_PATH', '')",
        ]),
    )

    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " controller_yaml:=", yaml_path,
        ]),
        value_type=str,
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_sim, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": PythonExpression([
                "'-r ' + ('' if '", gui, "' == 'true' else '-s ') + '",
                world_path, "'"
            ]),
        }.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
        }],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="ros_gz_sim_create",
        output="screen",
        arguments=[
            "-name", "ranger_mini_v3",
            "-topic", "/robot_description",
            "-x", x, "-y", y, "-z", z,
            "-allow_renaming", "true",
        ],
        parameters=[{"use_sim_time": True}],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false",
            description="Run gz sim with the GUI window."),
        DeclareLaunchArgument("world", default_value="empty_ground.sdf",
            description="World file under ranger_mini_v3_sim/worlds/."),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_z", default_value="0.32"),

        gz_resource_env,
        gz_launch,
        rsp,
        clock_bridge,
        spawn,
    ])
```

### .claude_handoff/round_05_phase3_checkpoint_AB.md

This file. Will be committed in step 7c.

## Verification output

### Step 1a — Pre-fix grep

```
$ grep -n 'GazeboSimROS2Control' ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
59:    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlSystem">
```

### Step 1b — Post-fix grep

```
$ grep -n 'GazeboSimROS2Control' ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
59:    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">

$ grep -n 'filename=' ranger_mini_v3_sim/urdf/ranger_mini_v3_sim.xacro
13:  <xacro:include filename="$(find ranger_mini_v3_description)/urdf/ranger_mini_v3.xacro"/>
59:    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
```

`filename="gz_ros2_control-system"` unchanged.

### Step 1c — Re-build + xacro re-verify

```
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.09s]
Summary: 1 package finished [0.17s]

exit=0
    <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
```

Expanded URDF now carries `…ControlPlugin`. Checkpoint A confirmed.

### Step 3a — Pre-edit CMakeLists

```
cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### Step 3b — Post-edit

```
install(
  DIRECTORY urdf config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
```

### Step 5a — Rebuild after world + launch

```
Starting >>> ranger_mini_v3_sim
Finished <<< ranger_mini_v3_sim [0.31s]
Summary: 1 package finished [0.40s]
```

### Step 5b — Installed files (symlinks via --symlink-install)

```
=== installed worlds ===
total 12
lrwxrwxrwx 1 sao sao 78 May 12 03:10 empty_ground.sdf -> /home/sao/agilex_ws/src/ranger_ros2/ranger_mini_v3_sim/worlds/empty_ground.sdf

=== installed launches ===
total 16
lrwxrwxrwx 1 sao sao 78 May 12 03:10 gazebo.launch.py        -> /home/sao/.../launch/gazebo.launch.py
lrwxrwxrwx 1 sao sao 86 May 12 02:05 view_sim_xacro.launch.py -> /home/sao/.../launch/view_sim_xacro.launch.py

=== world file head ===
<?xml version="1.0" ?>
<sdf version="1.10">
  <world name="empty_ground">
…
```

### Step 5c — `--print` parse

```
<launch.launch_description.LaunchDescription object at 0x…>
├── 5 × DeclareLaunchArgument (gui, world, spawn_x, spawn_y, spawn_z)
├── SetEnvironmentVariable (GZ_SIM_RESOURCE_PATH)
├── IncludeLaunchDescription (gz_sim/launch/gz_sim.launch.py)
├── ExecuteProcess (robot_state_publisher)
├── ExecuteProcess (parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock)
└── ExecuteProcess (ros_gz_sim/create -name ranger_mini_v3 -topic /robot_description -x -y -z -allow_renaming true)
```

(Reformatted from agent capture for readability; full raw output has
ExecInPkg/LocalVar substitution objects.)

### Step 5d — Headless bringup smoke test

#### Launch alive after 15 s

```
Launch PID=92075 PGID=92075
Launch root alive after 15s
```

#### gz model list

```
$ gz model --list
Requesting state for world [empty_ground]...

Available models:
    - ground_plane
    - ranger_mini_v3
```

**Robot is in the world.** ✓

#### gz model info `ranger_mini_v3` (head)

```
Model: [10]
  - Name: ranger_mini_v3
  - Pose [ XYZ (m) ] [ RPY (rad) ]:
    [-0.000000 -0.000000 0.314999]
    [0.000000 -0.000000 -0.000000]
  - Link [11]
    - Name: base_link
    - Mass (kg): 10.000000
    …
  - Link [14]
    - Name: fl_steering_wheel_link
    …
```

Spawn pose was z=0.32; it settled at z=0.315 (≈5 mm gravity drop while
the launch was warming up — wheels touched the ground plane).

#### gz topics

```
/clock
/gazebo/resource_paths
/stats
/world/empty_ground/clock
/world/empty_ground/dynamic_pose/info
/world/empty_ground/pose/info
/world/empty_ground/scene/deletion
/world/empty_ground/scene/info
/world/empty_ground/state
/world/empty_ground/stats
/world/empty_ground/light_config
/world/empty_ground/material_color
```

`/world/empty_ground/...` topics confirm the world loaded correctly.

#### ROS node list

```
/clock_bridge
/controller_manager
/gz_ros_control
/robot_state_publisher
```

**`controller_manager` and `gz_ros_control` showed up unexpectedly** —
see Open questions and Summary; these come from the
`GazeboSimROS2ControlPlugin` in the xacro, embedded in the gz process,
which was the whole point of wiring it in Phase 2.

#### ROS topic list

```
/clock
/controller_manager/activity
/controller_manager/introspection_data/full
/controller_manager/introspection_data/names
/controller_manager/introspection_data/values
/controller_manager/statistics/full
/controller_manager/statistics/names
/controller_manager/statistics/values
/diagnostics
/joint_states
/parameter_events
/robot_description
/rosout
/tf
/tf_static
```

Notable: `/controller_manager/*`, `/joint_states` already present
because cm is up; no controller has been spawned yet, so
`/joint_states` will be empty until joint_state_broadcaster starts in
Round 06.

#### `/robot_description` size

```
6
```

Six lines (a YAML scalar wrapping the URDF string in a single field).
Topic carries the URDF; small line count is just YAML formatting of a
giant single-line string.

#### `/clock` first message

```
clock:
  sec: 51
  nanosec: 934000000
```

Real, non-zero, monotonic.

#### TF base_link → base_footprint (fixed joint)

```
At time 0.0
- Translation: [0.000, 0.000, -0.310]
- Rotation: in Quaternion (xyzw) [0.000, 0.000, 0.000, 1.000]
```

Matches URDF math: `steer_z + wheel_drop - wheel_radius
= -0.10 + (-0.12) + (-0.09) = -0.31`. ✓ (rsp publishing static TF.)

#### TF base_link → fl_wheel_link (continuous joint)

```
Terminated
[expected: no joint_states yet]
```

Expected fail — no controllers means no `/joint_states` data → rsp
can't compute the continuous joint TF chain. Round 06 fixes this.

#### gz log tail (selected lines)

```
[gazebo-1] [INFO] [gz_ros_control]: Loading joint: fl_wheel
…
[gazebo-1] [INFO] [controller_manager]: Initialize hardware 'ranger_mini_v3_gz_system'
[gazebo-1] [INFO] [resource_manager]: Successful 'configure' of hardware 'ranger_mini_v3_gz_system'
[gazebo-1] [INFO] [resource_manager]: 'activate' hardware 'ranger_mini_v3_gz_system'
[gazebo-1] [INFO] [resource_manager]: Successful 'activate' of hardware 'ranger_mini_v3_gz_system'
[gazebo-1] [INFO] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
[gazebo-1] [WARN] [gz_ros_control]: Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s).
```

cm initialized the hardware interface, activated it, and is awaiting
controllers. The warning about update period is informational.

### Step 5d-4 — Teardown

`kill -- -<PGID>` from the launch root only got the launch's own
process group; gz sim's ruby wrapper, the actual `gz sim` binary,
`robot_state_publisher`, and `parameter_bridge` all ended up in
distinct process groups (each launched by `ExecuteProcess` with its
own setsid-ish supervision). I followed up with explicit
`kill -9 <PID>` for the four stragglers (PIDs 92099–92102):

```
$ pgrep -fa "ruby.*gz|robot_state_publisher|parameter_bridge|^gz sim" | grep -v claude
[none]
```

All processes gone. (Lesson: even with `setsid` on the launch root,
ros2 launch isolates each ExecuteProcess into its own pgid so children
survive the parent's group kill. Need a more aggressive teardown
pattern in future smoke tests — maybe `pkill -9 -f` on each known
component name as the primary mechanism, with PGID kill as a hint.)

### Manual verification steps for the operator

To visually confirm Checkpoint B end-to-end:

```bash
# In a fresh terminal:
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_sim gazebo.launch.py gui:=true
```

Expected outcome:

- A gz sim window opens showing the ground plane with the Ranger Mini
  v3 model sitting on it, wheels in contact, body floating ~0.31 m
  above the ground.
- The robot does NOT move (no controllers yet — wheels are
  free-spinning, but with no commanded torque/velocity).
- In a second terminal, `ros2 node list` should show `/clock_bridge`,
  `/controller_manager`, `/gz_ros_control`, `/robot_state_publisher`.
- In a second terminal, `ros2 control list_controllers` should print
  an empty list (cm is up; no controllers loaded). Round 06 will
  load them.

To shut down: Ctrl-C in the launch terminal.

## Deviations

- **Process teardown required pattern-based fallback.** The literal
  `kill -- -<PGID>` from `setsid` did not reach the children that
  ros2 launch's ExecuteProcess wrapper isolates into distinct pgids.
  Followed up with explicit PID kills. Documented in step 5d-4
  Verification output. No state was lost; this is a tooling lesson
  for the next smoke test.

- **`controller_manager` and `gz_ros_control` nodes appeared in the
  ROS graph despite "no controllers" being the goal.** This is
  correct architecturally: the `GazeboSimROS2ControlPlugin` we wired
  into the URDF in Phase 2 starts cm inside the gz process when the
  model loads. The "no controllers" property holds in the sense that
  no controller spawners ran and `ros2 control list_controllers`
  would return empty. Documented in Summary and Open questions.

## Open questions

1. **gz log warning about controller update period.** The
   `controllers.yaml` has `update_rate: 100` (Hz). gz simulates at
   1000 Hz (`max_step_size: 0.001`). cm warns that it'll tick at
   100 Hz instead of every gz step. Reasonable for our use case
   (the real driver runs at ~50 Hz on CAN), but worth tuning during
   Phase 3 dynamics validation if any controller behaves laggy. No
   action needed now.

2. **Spawn-z drift to 0.315 vs. nominal 0.32.** The body settled
   ~5 mm below the spawn z because the wheels caught the ground
   while the bringup was still configuring. If we want the robot to
   spawn already-settled, drop spawn_z to ~0.315 (or compute it
   exactly from the URDF as `steer_z + wheel_drop - wheel_radius`
   times -1 = 0.31 + a tire compression margin). Cosmetic; not
   worth chasing now.

3. **For Round 06 (controllers), do we want a separate launch
   `gazebo_with_controllers.launch.py` that includes
   `gazebo.launch.py` + spawners, or do we extend `gazebo.launch.py`
   with a `load_controllers:=true` arg that defaults to true?** The
   second is more usable; the first is more compositional. Architect
   to pick. Either fits one round.

4. **The launch's `GZ_SIM_RESOURCE_PATH` builder uses
   `PythonExpression` to concatenate `FindPackageShare(...) + ".."`
   to point at each package's parent (the `share/` dir). It works
   (the robot mesh URIs resolved — model spawned with full
   collision and inertia data per `gz model -m`), but it's ugly.
   ros_gz_sim 1.x has a `gz_resource_path` arg on its
   `gz_sim.launch.py` that's the proper interface; could clean up
   in a follow-up. Not blocking.
