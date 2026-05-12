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
