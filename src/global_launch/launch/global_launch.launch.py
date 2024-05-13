import launch
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    ld = LaunchDescription()
    camera_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/dji/eng_ws/src/hik_camera/launch/hik_camera.launch.py'
        )
    )
    detector_node = Node(
        #name='detector_node',
        package='exchangestation_detector',
        executable='detector_node',
        output='screen',
        respawn=True,
    )
    mind_detector_node = Node(
        name='mine_detector_node',
        package='mine_detector',
        executable='mine_detector_node',
        output='screen',
        respawn=True,
    )
    serial_node = Node(
        #name='serial_node',
        package='serial',
        executable='serial_node',
        output='screen',
    )
    small_camera_node = Node(
        #name='serial_node',
        package='camera',
        executable='main',
        output='screen',
    )
    second_exchange_node = Node(
        #name='serial_node',
        package='second_exchange_station',
        executable='main',
        output='screen',
        respawn=True,
    )
    referee_system_graphic_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/dji/eng_ws/src/referee_system_graphic/launch/run_referee_system_graphic.launch.py'
        )
    )
    referee_system_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/dji/eng_ws/src/referee_system/launch/run_referee_system.launch.py'
        )
    )
    ld.add_action(camera_launch)
    ld.add_action(detector_node)
    #ld.add_action(mind_detector_node)
    ld.add_action(serial_node)
    #ld.add_action(small_camera_node)
    #ld.add_action(second_exchange_node)
    ld.add_action(referee_system_graphic_launch)
    #ld.add_action(referee_system_launch)
    return ld
  
  