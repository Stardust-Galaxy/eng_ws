import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import os

def generate_launch_description():
  ld = LaunchDescription()
  
  detector_node = Node(
    name='mine_detector_node',
    package='mine_detector',
    executable='mine_detector_node',
    output='screen',
  )
  
  ld.add_action(detector_node)
  return ld