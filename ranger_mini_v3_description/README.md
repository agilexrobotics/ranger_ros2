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
