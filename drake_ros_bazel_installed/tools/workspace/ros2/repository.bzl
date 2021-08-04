load("//tools/skylark/ros2:ros2.bzl", "ros2_local_repository")

ROS2_DIST = "rolling"

def ros2_repository(name, overlays = []):
    overlays = ["/home/michel/Workspaces/drake_ros_ws/install"]
    ros2_local_repository(
        name = name,
        workspaces = overlays,
        include_packages = [
            "std_msgs",
            "geometry_msgs",
            "visualization_msgs",
            "rosidl_default_generators",
            "fastcdr",
            "rcpputils",
            "rcutils",
            "rclcpp",
            "rclpy",
            "rviz2",
            # RMW implementation
            "rmw_cyclonedds_cpp",
        ]
    )
