from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="ros2_example_bazel_installed",
                executable="eg_talker",
            ),
            launch_ros.actions.Node(
                package="ros2_example_bazel_installed",
                executable="eg_listener",
            ),
        ]
    )
