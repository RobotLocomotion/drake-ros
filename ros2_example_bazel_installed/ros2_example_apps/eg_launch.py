from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Running a talker python node.
        ExecuteProcess(
            cmd=['python3','ros2_example_apps/roslaunch_eg_nodes/eg_talker.py'],
        ),
        # Running a listener python node.
        ExecuteProcess(
            cmd=['./ros2_example_apps/eg_listener'],
        ),
    ])
