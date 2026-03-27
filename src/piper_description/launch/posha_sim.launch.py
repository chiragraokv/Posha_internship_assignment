from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'my_robot',
                '-file', '/home/laserhammer/Posha_internship_assignment/src/posha_simu/urdf/piper_description.urdf',
                '-x', '0', '-y', '0', '-z', '0.5'
            ],
            output='screen'
        )
    ])