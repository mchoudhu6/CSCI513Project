from launch import LaunchDescription
from launch.actions import ExecuteProcess
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
    ])
