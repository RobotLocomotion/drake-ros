DRAKE_ROS_REQUIRED_PACKAGES = [
    "geometry_msgs",
    "rclpy",
    "rclcpp",
    "rosidl_runtime_c",
    "rosidl_typesupport_cpp",
    "tf2_eigen",
    "tf2_ros",
    "visualization_msgs",
    "rmw_cyclonedds_cpp"
]

DRAKE_ROS_TEST_DEPENDENCIES = [
    "test_msgs",
    "tf2_ros_py",
]

def drake_ros_fail_if_missing_required_packages(packages):
    for pkg in DRAKE_ROS_REQUIRED_PACKAGES:
        if pkg not in packages:
            fail("Missing Drake-ROS dependency: " + pkg)
