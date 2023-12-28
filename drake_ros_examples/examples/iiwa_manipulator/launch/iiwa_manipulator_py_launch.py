import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('drake_ros_examples'), 'iiwa_manipulator.rviz')

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    iiwa_manipulator_py_node = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('drake_ros_examples'), 'lib', 'drake_ros_examples', 'iiwa_manipulator.py')],
    )
    return LaunchDescription([
        rviz_node,
        iiwa_manipulator_py_node,
    ])
