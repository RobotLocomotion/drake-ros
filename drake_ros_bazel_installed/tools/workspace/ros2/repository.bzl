load("//tools/skylark/ros2:ros2.bzl", "ros2_local_repository")

ROS2_DIST = "rolling"

def ros2_repository(name, overlays = []):
    ros2_local_repository(
        name = name,
        workspaces = ['/opt/ros/{}'.format(ROS2_DIST)],
        include_packages = [
            "action_msgs",
            "builtin_interfaces",
            "rosidl_default_generators",
            "rclcpp_action",
            "rclcpp",
            "rclpy",
            # RMW implementations
            "rmw_cyclonedds_cpp",
            "rmw_fastrtps_cpp",
        ]
    )
