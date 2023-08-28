from launch import LaunchDescription
from launch.actions import ExecuteProcess
from roslaunch_util import ExecuteBazelTarget

def generate_launch_description():
    return LaunchDescription([
        # Running a talker written in python.
        ExecuteBazelTarget('eg_talker'),
        # Running a listener written in cpp.
        ExecuteBazelTarget('eg_listener'),
    ])
