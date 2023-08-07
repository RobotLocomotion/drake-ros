from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('drake_ros_examples'), 'multirobot.rviz')

    # Node for RViz2
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    # Node for multirobot in Python
    multirobot_py_node = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('drake_ros_examples'), 'lib', 'drake_ros_examples', 'multirobot.py')],
    )

    return LaunchDescription([
        rviz_node,
        multirobot_py_node,
    ])
