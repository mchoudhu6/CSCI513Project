from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    world_file_path = os.path.join(
        os.getenv('HOME'), 'ros2_ws', 'src', 'act', 'worlds', 'living_room.sdf'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', world_file_path],
            output='screen'
        ),
        Node(
            package='act',
            executable='path_finding_algo.py',
            output='screen'
        )
    ])
