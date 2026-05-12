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

    # Add the package share path to GZ_SIM_RESOURCE_PATH so the
    # robot's mesh URIs (package://ranger_mini_v3_description/...)
    # resolve. ros_gz_sim sets some defaults but ours need adding.
    gz_resource_env = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=PythonExpression([
            "'", PathJoinSubstitution([FindPackageShare("ranger_mini_v3_description"), ".."]), "'",
            " + ':' + '",
            PathJoinSubstitution([FindPackageShare("ranger_mini_v3_sim"), ".."]), "'",
            " + ':' + __import__('os').environ.get('GZ_SIM_RESOURCE_PATH', '')",
        ]),
    )

    # Process the full sim xacro (with controller_yaml resolved
    # to the install share path). The result becomes the
    # /robot_description parameter AND the source of the
    # SDF that gets spawned.
    robot_description = ParameterValue(
        Command([
            "xacro ", xacro_path,
            " controller_yaml:=", yaml_path,
        ]),
        value_type=str,
    )

    # Start gz sim via ros_gz_sim's launch file. Args:
    #   gz_args: '-r <world>' to run, plus '-s' for server-only
    #            when gui is false.
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

    # robot_state_publisher publishes /robot_description and TF
    # from joint_states (which will start flowing once controllers
    # are added in Round 06).
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

    # Spawn the robot in gz. -topic /robot_description reads the
    # URDF from rsp, -name sets the gz model name. -allow_renaming
    # is defensive in case of name collisions.
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

    # /clock bridge so use_sim_time consumers (rsp, controllers,
    # tools) sync to gz simulation time.
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
