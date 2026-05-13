# ranger_mini_v3_sim

Gazebo Harmonic simulation for the AgileX Ranger Mini v3. The
sim mirrors the real-robot driver's ROS interface
(ranger_base/src/ranger_messenger.cpp), so application code
(teleop, nav2, behavior trees) is portable between sim and
real with zero changes.

## Quick start

Build and source the workspace first:

```bash
cd ~/agilex_ws
colcon build --symlink-install
source install/setup.bash
```

Then in three terminals:

```bash
# Terminal A — start Gazebo with the robot + controllers
ros2 launch ranger_mini_v3_sim gazebo_full.launch.py gui:=true

# Terminal B — start the Twist messenger (waits ~30s for gz)
ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

# Terminal C — drive
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.3}}"
```

Robot should drive forward at 0.3 m/s.

## Launch file hierarchy

Multiple launch files exist with different scopes. **For most
use cases, use `gazebo_full.launch.py`.**

| Launch file                        | What it starts             | Use when |
|------------------------------------|----------------------------|----------|
| `gazebo_full.launch.py`            | gz + robot + controllers + bridge | **Default. Use this to drive the robot.** |
| `gazebo.launch.py`                  | gz + robot + bridge only (NO controllers) | Debugging the URDF spawn / mesh resolution. Without controllers, wheel commands have no subscribers and the robot will appear stationary. |
| `view_sim_xacro.launch.py`         | Prints the expanded sim URDF to stdout | Inspecting the URDF + ros2_control block; no Gazebo started. |
| `messenger.launch.py` (in ranger_mini_v3_sim_messenger) | The Twist → wheel-command messenger | Pair with `gazebo_full.launch.py`. |

## ROS interface (parity with real driver)

Subscribes:
- `/cmd_vel` (`geometry_msgs/msg/Twist`)

Publishes:
- `/odom` (`nav_msgs/msg/Odometry`)
- `/system_state` (`ranger_msgs/msg/SystemState`)
- `/motion_state` (`ranger_msgs/msg/MotionState`)
- `/actuator_state` (`ranger_msgs/msg/ActuatorStateArray`)
- `/battery_state` (`sensor_msgs/msg/BatteryState`)
- `/tf` (`odom → base_link`, gated by `publish_odom_tf` param,
  default false to match real driver)

Plus the standard ros2_control internal topics:
- `/joint_states` (from joint_state_broadcaster)
- `/<joint>_<position|velocity>_controller/commands` (8 of these,
  driven by the messenger; you can also send raw commands to
  them for low-level testing)

## Motion modes

The messenger auto-selects between four motion modes based on
the Twist content. Same logic as the real driver:

| cmd_vel                              | Mode               | Behavior |
|--------------------------------------|--------------------|----------|
| `x > 0, z = 0`                       | DUAL_ACKERMAN      | Straight-line forward driving. |
| `x > 0, z ≠ 0` and turn radius large | DUAL_ACKERMAN      | Front and rear axles steer toward shared instantaneous center. |
| `x > 0, z ≠ 0` and turn radius small | SPINNING           | Pure rotation in place (commanded yaw rate). |
| `y ≠ 0`                              | PARALLEL           | All four wheels point same direction (atan2 of x,y); body translates without yawing. |
| `x = 0, y ≠ 0`                       | PARALLEL (side-slip subcase) | All wheels at ±π/2; body crabs sideways. |

## Troubleshooting

**Wheels don't move when I send cmd_vel.** Check:
1. Are you launching `gazebo_full.launch.py` (NOT
   `gazebo.launch.py`)? Without controllers, the messenger's
   commands have no subscribers.
2. Is the messenger running? `ros2 node list` should show
   `/sim_messenger`.
3. Are controllers active? `ros2 control list_controllers`
   should show 9 controllers all 'active'.
4. Is /clock advancing? Look for `tick_diag` lines in the
   messenger terminal; sim_time should be increasing.

**`tick_diag` shows mean_dt much smaller than 0.0200s.** Possible
/clock QoS regression — see R10/R11 in `.claude_handoff/` for
symptoms and fix (already in place via
`config/ros_gz_bridge.yaml`).

**Topic type errors when echoing `/system_state` etc.** The
ros2 daemon's type cache has gone stale: `ros2 daemon stop && ros2 daemon start`.

**DDS state accumulation across many sessions** (rare). If
many ros2 nodes have exited ungracefully, FastDDS shared
memory in `/dev/shm/fastrtps_*` can persist. After a full
process kill, optionally run:
```bash
find /dev/shm -maxdepth 1 -user $(whoami) -name "fastrtps_*" -delete
```

## Development history

Detailed per-round handoff trail is in `.claude_handoff/`
(rounds 01 through 13). The squash-merged jazzy branch
contains one commit per phase; phase branches retain the
full granular history.
