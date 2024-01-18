#!/bin/python
import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_ros.actions
import launch_ros.descriptions


def generate_launch_description():
    rviz_config_file = os.path.join(get_package_share_directory('drake_ros_examples'),
                                    'rgbd/rgbd.rviz')

    rviz_node = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    rgbd_node = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('drake_ros_examples'), 'lib',
                          'drake_ros_examples', 'rgbd')],
    )

    return LaunchDescription([
        rviz_node,
        rgbd_node,
        # launch plugin through rclcpp_components container
        launch_ros.actions.ComposableNodeContainer(
            name='container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                # Driver itself
                launch_ros.descriptions.ComposableNode(
                    package='depth_image_proc',
                    plugin='depth_image_proc::PointCloudXyzrgbNode',
                    name='point_cloud_xyzrgb_node',
                    remappings=[('rgb/camera_info', '/color/camera_info'),
                                ('rgb/image_rect_color', '/color/image_raw'),
                                ('depth_registered/image_rect',
                                 '/depth/image_raw')],
                    parameters=[{'qos_overrides./color/image_raw.subscription.reliability':
                                 'best_effort',
                                 'qos_overrides./depth/image_raw.subscription.reliability':
                                 'best_effort'}]
                ),
            ],
            output='screen',
        ),
    ])
