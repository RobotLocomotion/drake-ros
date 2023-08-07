import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('drake_ros_examples'), 'hydroelastic/hydroelastic.rviz')

    # Node for RViz2
    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    # Node for multirobot in C++
    hydroelastic_cpp_node = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('drake_ros_examples'), 'lib', 'drake_ros_examples', 'hydroelastic')],
    )
    return LaunchDescription([
        rviz_node,
        hydroelastic_cpp_node,
    ])
