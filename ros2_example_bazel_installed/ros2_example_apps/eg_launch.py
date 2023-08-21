from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3','ros2_example_apps/eg_talker.py'],
        ),
    ])
