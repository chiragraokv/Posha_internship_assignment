from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_path = os.path.join(os.getenv('HOME'), 'Posha_internship_assignment', 'src', 'posha_simu')
    urdf_file = os.path.join(pkg_path, 'urdf', 'piper_description.urdf')
    empty_world = '/opt/ros/humble/share/gazebo_ros/worlds/empty.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join('/opt/ros/humble/share/gazebo_ros', 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': empty_world}.items()
    )

    spawn_robot_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', urdf_file,
            '-x', '0', '-y', '0', '-z', '0.5'
        ],
        output='screen'
    )

    spawn_robot = TimerAction(
        period=3.0,  # wait 3 seconds for Gazebo
        actions=[spawn_robot_node]
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
    ])