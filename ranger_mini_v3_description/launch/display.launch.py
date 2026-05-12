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
