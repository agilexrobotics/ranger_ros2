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
