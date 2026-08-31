"""Launch the SimMessenger node.

Use this with gazebo_full.launch.py running in another terminal:

    # Terminal A
    ros2 launch ranger_mini_v3_sim gazebo_full.launch.py

    # Terminal B
    ros2 launch ranger_mini_v3_sim_messenger messenger.launch.py

Or compose them in a single launch in Phase 5.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ranger_mini_v3_sim_messenger',
            executable='sim_messenger',
            name='sim_messenger',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'update_rate': 50,
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'odom_topic_name': 'odom',
                'publish_odom_tf': False,
            }],
        ),
    ])
