import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    ld = LaunchDescription()
    camera_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            '/home/jlurobovision/eng_ws/src/galaxy_camera/launch/galaxy_camera.launch.py'
        )
    )
    detector_node = Node(
        name='detector_node',
        package='exchangestation_detector',
        executable='detector_node',
        output='screen',
    )
    serial_node = Node(
        name='serial_node',
        package='serial',
        executable='serial_node',
        output='screen',
    )
    ld.add_action(camera_launch)
    ld.add_action(detector_node)
    ld.add_action(serial_node)
    
    return ld
  
  