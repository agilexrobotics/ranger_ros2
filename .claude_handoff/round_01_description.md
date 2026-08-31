# Round 01 — Phase 1: import the description package

## Summary

Imported the `ranger_mini_v3_description` package from the architect-supplied
zip (`~/Downloads/ranger_mini_v3_description_phase1.zip`) into the existing
`~/agilex_ws/src/ranger_ros2/` repository on a new branch
`phase-1-description`. The package builds cleanly under ROS 2 Jazzy, the
xacro parses without warnings (275-line URDF), and the launch description
prints correctly. No file content was modified during the copy. Two commits
on this branch: one for the package, one for the handoff artifacts.

## Diff

The phase-1 commit is a pure-add (no pre-existing files modified). Binary
DAE meshes are listed but their bytes are omitted from the diff fence below;
their full contents appear in the commit.

```diff
commit a876c9b8013e9a4fb053cb2ac828e7d8eb57e4b5
Author: Shuaib Olanrewaju <solanrewaju2020@fau.edu>
Date:   Mon May 11 23:45:52 2026 -0400

    feat(sim): add ranger_mini_v3_description package (phase 1)

 ranger_mini_v3_description/CMakeLists.txt              |  +12
 ranger_mini_v3_description/README.md                   |  +60
 ranger_mini_v3_description/launch/display.launch.py    |  +76
 ranger_mini_v3_description/meshes/ranger_base.dae      | (binary, 38454037 bytes)
 ranger_mini_v3_description/meshes/steering_wheel.dae   | (binary,   171983 bytes)
 ranger_mini_v3_description/meshes/wheel_v3.dae         | (binary,  1041026 bytes)
 ranger_mini_v3_description/package.xml                 |  +26
 ranger_mini_v3_description/rviz/display.rviz           |  +46
 ranger_mini_v3_description/urdf/ranger_mini_v3.xacro   | +161

 9 files changed, 13480 insertions(+)
```

`git status` after the phase-1 commit and before the handoff commit was clean
except for the untracked `.claude_handoff/` directory. There were no
unstaged modifications.

## New files (full content)

### ranger_mini_v3_description/package.xml

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>ranger_mini_v3_description</name>
  <version>0.1.0</version>
  <description>
    URDF / xacro description and meshes for the AgileX Ranger Mini v3.
    Ported from agilexrobotics/ugv_gazebo_sim (ROS 1) to ROS 2 Jazzy + Gazebo Harmonic.
  </description>
  <maintainer email="you@example.com">you</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- Used at runtime (xacro processing, model viewing). -->
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### ranger_mini_v3_description/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.8)
project(ranger_mini_v3_description)

find_package(ament_cmake REQUIRED)

# Ship every asset the robot description, RViz, and launch system need.
install(
  DIRECTORY urdf meshes rviz launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
```

### ranger_mini_v3_description/README.md

```markdown
# ranger_mini_v3_description

ROS 2 Jazzy description package for the AgileX Ranger Mini v3, ported from the
ROS 1 model in [`agilexrobotics/ugv_gazebo_sim`](https://github.com/agilexrobotics/ugv_gazebo_sim).

This package contains **only** the URDF / xacro and meshes. It has no
dependency on Gazebo, `ros2_control`, or sensor plugins, so it can be reused
unchanged by nav2, RViz, MoveIt, etc.

## Build and run

```bash
# from your ranger_ros2 workspace root
colcon build --packages-select ranger_mini_v3_description --symlink-install
source install/setup.bash

# RViz2 + joint sliders
ros2 launch ranger_mini_v3_description display.launch.py

# Or, headless (joint_state_publisher publishes zeros)
ros2 launch ranger_mini_v3_description display.launch.py use_gui:=false
```

## Differences from the ROS 1 model

- **xacro-ized** into a `steering_hub` + `drive_wheel` macro per corner so
  every dimension is in one place at the top of the file.
- **Fixed `rr_steering_joint` origin** which was `(-0.23, -0.206, -0.1)` in
  the original — almost certainly a typo since the other three corners are at
  `(±0.25, ±0.19, -0.1)`. The asymmetry would have caused odometry drift if
  ever wired into a 4WS kinematic model. The port standardizes it to
  `(-0.25, -0.19, -0.1)`.
- **Removed all `<transmission>` blocks** and the `gazebo_ros_control` plugin
  — they will be reintroduced as `ros2_control` + `gz_ros2_control` in the
  sibling `ranger_mini_v3_sim` package (Phase 2).
- **Added `base_footprint`** under `base_link` for nav2 compatibility.
- **Mesh URIs** point to `package://ranger_mini_v3_description/meshes/...`
  instead of the original `package://ranger_mini_v3/meshes/...`.

## Joint inventory

| Joint                  | Type       | Notes                                  |
|------------------------|------------|----------------------------------------|
| `fl_steering_joint`    | revolute   | ±2.1 rad, axis `(0, 0, -1)`            |
| `fr_steering_joint`    | revolute   | ±2.1 rad, axis `(0, 0, -1)`            |
| `rl_steering_joint`    | revolute   | ±2.1 rad, axis `(0, 0, -1)`            |
| `rr_steering_joint`    | revolute   | ±2.1 rad, axis `(0, 0, -1)`            |
| `fl_wheel`             | continuous | drive, axis `(0, 1, 0)`                |
| `fr_wheel`             | continuous | drive, axis `(0, 1, 0)`                |
| `rl_wheel`             | continuous | drive, axis `(0, 1, 0)`                |
| `rr_wheel`             | continuous | drive, axis `(0, 1, 0)`                |
| `base_footprint_joint` | fixed      | base_link → base_footprint             |

## Next phases

- **Phase 2:** `ranger_mini_v3_sim` adds `ros2_control` tags, the
  `gz_ros2_control` plugin, controller YAML, and a sim-time launch file.
- **Phase 3:** Gazebo Harmonic empty-world launch + friction/inertia tuning.
- **Phase 4:** `ranger_sim_messenger` node — Twist→4WS kinematics + odometry
  + mock state topics matching the real `ranger_base` driver interface.
```

### ranger_mini_v3_description/launch/display.launch.py

```python
"""Launch RViz2 with the Ranger Mini v3 description.

Phase 1 sanity check: no Gazebo, no controllers. Just confirms the xacro
parses, meshes resolve, and the kinematic tree renders correctly. Use the
joint_state_publisher_gui sliders to manually drive the four steering and
four wheel joints to visually verify the model.

Usage:
    ros2 launch ranger_mini_v3_description display.launch.py
    ros2 launch ranger_mini_v3_description display.launch.py use_gui:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ranger_mini_v3_description")

    xacro_path = PathJoinSubstitution([pkg, "urdf",  "ranger_mini_v3.xacro"])
    rviz_path  = PathJoinSubstitution([pkg, "rviz",  "display.rviz"])

    use_gui = LaunchConfiguration("use_gui")

    # Process xacro -> URDF string at launch time. The ParameterValue wrapper
    # is required so the substitution is evaluated and the result is treated
    # as a string parameter (otherwise robot_state_publisher tries to parse
    # the substitution object itself).
    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_gui", default_value="true",
            description="Run joint_state_publisher_gui to drive joints manually."
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),

        # Default: GUI sliders. Set use_gui:=false to use the headless
        # joint_state_publisher which just publishes zeros — useful when
        # something else (Gazebo, a bag, the real robot) supplies the states.
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            condition=UnlessCondition(use_gui),
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_path],
            output="screen",
        ),
    ])
```

### ranger_mini_v3_description/urdf/ranger_mini_v3.xacro

```xml
<?xml version="1.0" encoding="utf-8"?>
<!--
  Ranger Mini v3 URDF / xacro
  Ported from agilexrobotics/ugv_gazebo_sim ranger_mini_v3/urdf/ranger_mini.xacro
  for ROS 2 Jazzy + Gazebo Harmonic.

  Phase 1: kinematic + visual description only. ros2_control + gz plugins land
  in ranger_mini_v3_sim/urdf/ranger_mini.gazebo.xacro and are xacro:included
  from the simulation launch — NOT from this file — so this package stays
  consumable by anyone who only wants the model in RViz / MoveIt / nav stack.
-->
<robot name="ranger_mini_v3" xmlns:xacro="http://ros.org/wiki/xacro">

  <!-- ============================================================
       Geometry parameters
       (Wheelbase / track are taken from the original origins:
        FL/FR x = +0.25, RL/RR x = -0.25  -> wheelbase 0.50 m
        L  y = +0.19,    R  y = -0.19    -> track     0.38 m
        The original RR was -0.23 / -0.206 — a typo. Fixed below.)
       ============================================================ -->
  <xacro:property name="wheelbase_half" value="0.25"/>
  <xacro:property name="track_half"     value="0.19"/>
  <xacro:property name="steer_z"        value="-0.1"/>
  <xacro:property name="wheel_drop"     value="-0.12"/>
  <xacro:property name="wheel_radius"   value="0.09"/>
  <xacro:property name="wheel_width"    value="0.08"/>
  <xacro:property name="wheel_mass"     value="8.0"/>
  <xacro:property name="steer_mass"     value="1.0"/>
  <xacro:property name="steer_limit"    value="2.1"/>
  <xacro:property name="steer_effort"   value="5.0"/>
  <xacro:property name="steer_vel"      value="6.28"/>
  <xacro:property name="mesh_pkg"       value="package://ranger_mini_v3_description/meshes"/>

  <!-- ============================================================
       Base link
       ============================================================ -->
  <link name="base_link">
    <inertial>
      <origin xyz="-0.0242698 -0.0030308 0.0544267" rpy="0 0 0"/>
      <mass value="10.0"/>
      <inertia ixx="0.1354" ixy="0.0" ixz="0.0"
               iyy="0.2417" iyz="0.0" izz="0.3085"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <mesh filename="${mesh_pkg}/ranger_base.dae" scale="1000 1000 1000"/>
      </geometry>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.35 0.2"/>
      </geometry>
    </collision>
  </link>

  <!-- A common nav convention: base_footprint at ground level under base_link.
       Useful for nav2 and TF consumers that expect it. -->
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed">
    <parent link="base_link"/>
    <child  link="base_footprint"/>
    <origin xyz="0 0 ${steer_z + wheel_drop - wheel_radius}" rpy="0 0 0"/>
  </joint>

  <!-- ============================================================
       Macros: one steering hub + one drive wheel per corner.
       ============================================================ -->

  <!-- Steering hub (revolute joint attached to base_link). -->
  <xacro:macro name="steering_hub" params="prefix x y mesh_yaw">
    <link name="${prefix}_steering_wheel_link">
      <inertial>
        <origin xyz="0 0 0.0437" rpy="0 0 0"/>
        <mass value="${steer_mass}"/>
        <inertia ixx="0.006363" ixy="0.0" ixz="0.0"
                 iyy="0.006363" iyz="0.0" izz="0.010465"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 ${mesh_yaw}"/>
        <geometry>
          <mesh filename="${mesh_pkg}/steering_wheel.dae" scale="10 10 10"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.001" length="0.001"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_steering_joint" type="revolute">
      <origin xyz="${x} ${y} ${steer_z}" rpy="0 0 0"/>
      <parent link="base_link"/>
      <child  link="${prefix}_steering_wheel_link"/>
      <!-- Original used axis (0,0,-1); we keep that so positive command =
           CW seen from above, matching the real-driver kinematic model. -->
      <axis xyz="0 0 -1"/>
      <limit effort="${steer_effort}" velocity="${steer_vel}"
             lower="${-steer_limit}" upper="${steer_limit}"/>
      <dynamics damping="0.1" friction="0.0"/>
    </joint>
  </xacro:macro>

  <!-- Drive wheel (continuous joint hanging off the steering hub). -->
  <xacro:macro name="drive_wheel" params="prefix mesh_roll">
    <link name="${prefix}_wheel_link">
      <inertial>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <mass value="${wheel_mass}"/>
        <inertia ixx="0.02047" ixy="0.0" ixz="0.0"
                 iyy="0.02047" iyz="0.0" izz="0.0324"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="${mesh_roll} 0 0"/>
        <geometry>
          <mesh filename="${mesh_pkg}/wheel_v3.dae" scale="10 10 10"/>
        </geometry>
      </visual>
      <collision>
        <origin xyz="0 0 -0.005" rpy="1.5707963 0 0"/>
        <geometry>
          <cylinder length="${wheel_width}" radius="${wheel_radius}"/>
        </geometry>
      </collision>
    </link>

    <joint name="${prefix}_wheel" type="continuous">
      <origin xyz="0 0 ${wheel_drop}" rpy="0 0 0"/>
      <parent link="${prefix}_steering_wheel_link"/>
      <child  link="${prefix}_wheel_link"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.05" friction="0.0"/>
    </joint>
  </xacro:macro>

  <!-- ============================================================
       Instantiate four corners.
       Mesh-yaw / mesh-roll values preserve the visual orientations
       used in the original URDF (some wheels' DAEs are mirrored).
       ============================================================ -->

  <!-- Front-Right -->
  <xacro:steering_hub prefix="fr" x="${ wheelbase_half}" y="${-track_half}" mesh_yaw="0"/>
  <xacro:drive_wheel  prefix="fr" mesh_roll="3.14"/>

  <!-- Front-Left -->
  <xacro:steering_hub prefix="fl" x="${ wheelbase_half}" y="${ track_half}" mesh_yaw="3.14"/>
  <xacro:drive_wheel  prefix="fl" mesh_roll="0"/>

  <!-- Rear-Left -->
  <xacro:steering_hub prefix="rl" x="${-wheelbase_half}" y="${ track_half}" mesh_yaw="3.14"/>
  <xacro:drive_wheel  prefix="rl" mesh_roll="0"/>

  <!-- Rear-Right  (FIX: original had (-0.23, -0.206); standardized here) -->
  <xacro:steering_hub prefix="rr" x="${-wheelbase_half}" y="${-track_half}" mesh_yaw="0"/>
  <xacro:drive_wheel  prefix="rr" mesh_roll="3.14"/>

</robot>
```

### ranger_mini_v3_description/rviz/display.rviz

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Plane: XY
      Cell Size: 0.5
      Line Style:
        Value: Lines
    - Class: rviz_default_plugins/RobotModel
      Name: RobotModel
      Enabled: true
      Description Source: Topic
      Description Topic:
        Value: /robot_description
        Depth: 5
        Reliability Policy: Reliable
        Durability Policy: Transient Local
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Name: TF
      Enabled: true
      Show Names: true
      Marker Scale: 0.3
  Global Options:
    Fixed Frame: base_link
    Background Color: 48; 48; 48
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Name: Current View
      Distance: 2.0
      Focal Point: { X: 0, Y: 0, Z: 0 }
      Pitch: 0.4
      Yaw: 0.8
```

### ranger_mini_v3_description/meshes/ranger_base.dae

Binary, 38454037 bytes (≈ 36.7 MiB). Content omitted.

### ranger_mini_v3_description/meshes/steering_wheel.dae

Binary, 171983 bytes (≈ 168 KiB). Content omitted.

### ranger_mini_v3_description/meshes/wheel_v3.dae

Binary, 1041026 bytes (≈ 1016 KiB). Content omitted.

## Verification output

### tree -L 3 ranger_mini_v3_description

```
ranger_mini_v3_description
├── CMakeLists.txt
├── config
├── launch
│   └── display.launch.py
├── meshes
│   ├── ranger_base.dae
│   ├── steering_wheel.dae
│   └── wheel_v3.dae
├── package.xml
├── README.md
├── rviz
│   └── display.rviz
└── urdf
    └── ranger_mini_v3.xacro

6 directories, 9 files
```

(Note: `config/` is present but empty in the source tree; see Open
questions for follow-up.)

### xacro ranger_mini_v3_description/urdf/ranger_mini_v3.xacro > /tmp/ranger_check.urdf

```
exit=0
275 /tmp/ranger_check.urdf
```

### colcon build --packages-select ranger_mini_v3_description --symlink-install

```
Starting >>> ranger_mini_v3_description
Finished <<< ranger_mini_v3_description [1.13s]

Summary: 1 package finished [1.24s]
```

### ros2 pkg prefix ranger_mini_v3_description

```
/home/sao/agilex_ws/install/ranger_mini_v3_description
```

### ros2 pkg xml ranger_mini_v3_description

```
<package format="3">
  <name>ranger_mini_v3_description</name>
  <version>0.1.0</version>
  <description>
    URDF / xacro description and meshes for the AgileX Ranger Mini v3.
    Ported from agilexrobotics/ugv_gazebo_sim (ROS 1) to ROS 2 Jazzy + Gazebo Harmonic.
  </description>
  <maintainer email="you@example.com">you</maintainer>
  <license>BSD-3-Clause</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### ros2 launch ranger_mini_v3_description display.launch.py --print

```
<launch.launch_description.LaunchDescription object at 0x70658c6adcd0>
├── Action('<launch.actions.declare_launch_argument.DeclareLaunchArgument object at 0x70658c639df0>')
├── ExecuteProcess(cmd=[ExecInPkg(pkg='robot_state_publisher', exec='robot_state_publisher'), '--ros-args', '-r', LocalVar('node name')], cwd=None, env=None, shell=False)
├── ExecuteProcess(cmd=[ExecInPkg(pkg='joint_state_publisher_gui', exec='joint_state_publisher_gui'), '--ros-args', '-r', LocalVar('node name')], cwd=None, env=None, shell=False)
├── ExecuteProcess(cmd=[ExecInPkg(pkg='joint_state_publisher', exec='joint_state_publisher'), '--ros-args', '-r', LocalVar('node name')], cwd=None, env=None, shell=False)
└── ExecuteProcess(cmd=[ExecInPkg(pkg='rviz2', exec='rviz2'), '-d', PathJoinSubstitution('FindPackageShare(pkg='ranger_mini_v3_description'), 'rviz', 'display.rviz''), '--ros-args', '-r', LocalVar('node name')], cwd=None, env=None, shell=False)
```

### Manual launch steps for the operator

The agent does not launch GUI applications. To visually verify the
description, the operator should run, in a fresh terminal:

```bash
# In a new terminal:
cd ~/agilex_ws
source install/setup.bash
ros2 launch ranger_mini_v3_description display.launch.py
```

Expected outcome:

- An RViz2 window opens with the Ranger Mini v3 model visible.
- The fixed frame is `base_link` (already set in `display.rviz`).
- The TF tree (TF display in RViz, or `ros2 run tf2_tools view_frames`)
  shows all 9 joints from the description: 4 steering hubs, 4 wheels,
  plus the `base_footprint_joint` fixed link.
- A separate `joint_state_publisher_gui` window appears with sliders for
  each of the 8 movable joints (`fl_steering_joint`, `fr_steering_joint`,
  `rl_steering_joint`, `rr_steering_joint`, `fl_wheel`, `fr_wheel`,
  `rl_wheel`, `rr_wheel`); moving each slider should rotate the
  corresponding link in RViz.
- For headless verification (no slider GUI, joints pinned at zero):
  `ros2 launch ranger_mini_v3_description display.launch.py use_gui:=false`.

## Deviations

- **Branched from `humble`, not `main`.** Bootstrap policy says "One
  branch per phase, off main", but the round-01 instruction says to
  branch off the current branch if the repo already exists. The
  pre-existing repo has no `main` branch — only `humble` (checked out)
  and remote `air_delta`, `humble`, `jazzy`, `old-version`. I followed
  the literal round-01 instruction and branched `phase-1-description`
  from `humble`. See Open Questions for the architect to confirm the
  desired base branch policy going forward (likely we should rename
  `humble` → `main`, or branch all future phases from `jazzy`).

- **The zip contained an empty `config/` directory** that was not
  enumerated in the prompt's file list. I copied it as-is (the prompt
  forbids file-content modification but does not address empty dirs);
  it is referenced by `CMakeLists.txt` (`install(DIRECTORY ... config
  DESTINATION ...)`). Git does not version empty directories, so the
  `config/` dir is **not** tracked in the commit. See Open questions.

- **Package author/maintainer info inherited from the existing repo's
  git config** for the commit (Shuaib Olanrewaju, solanrewaju2020@fau.edu).
  No identity changes were made; this is just a heads-up since
  `package.xml` still has the placeholder `you@example.com`.

## Open questions

1. **Base-branch policy.** Bootstrap says "off main" but no `main`
   branch exists in this repo. Phase 1 was branched from `humble`. For
   future phases: should I (a) rename/create `main` from `humble`,
   (b) branch from `jazzy` (which seems more aligned with the ROS 2
   Jazzy target), or (c) keep branching off `humble`? Awaiting an
   explicit instruction.

2. **Empty `config/` directory.** The CMakeLists.txt references
   `config` in the `install(DIRECTORY ...)` list, but the dir is empty
   and not committed (git ignores empty dirs). On a fresh clone this
   would cause `install(DIRECTORY config ...)` to fail at CMake-config
   time (CMake errors when the named directory does not exist). Options:
   (a) add a `.gitkeep` placeholder so the dir survives clone
   (recommended; minimal-change), (b) drop `config` from the
   CMakeLists.txt install list until phase 2 actually adds files there,
   or (c) leave as-is and accept the future build break. Awaiting
   instruction since the prompt forbade file edits.

3. **`package.xml` maintainer field** still contains the placeholder
   `you@example.com` / `you`. Should this be updated, or is it the
   architect's intent to leave it as a per-deployer field?
