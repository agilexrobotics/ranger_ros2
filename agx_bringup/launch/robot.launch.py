import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  config = os.path.join(
    get_package_share_directory('agx_bringup'),
    'config',
    'agx_bringup.yaml'
  ) 
  
  return LaunchDescription([
    Node(
      package="agx_bringup",
      executable="agx_bringup_node",
      name="agx_bringup_node",
      output="screen",
      parameters=[config],
      remappings = [
        # ('/sub_cmd_vel', '/agx/twist/cmd_vel'),
        ('/sub_cmd_vel', '/cmd_vel'),
        ('/wheel/odom', '/agx/wheel_odom_raw')
      ]
    ),
  ])