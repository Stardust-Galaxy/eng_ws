import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchConfiguration

import os

def generate_launch_description():
  ld = LaunchDescription()
  
  detector_node = Node(
    name='detector_node',
    package='exchangestation_detector',
    executable='detector_node',
    output='screen',
  )
  
  ld.add_action(detector_node)
  return ld