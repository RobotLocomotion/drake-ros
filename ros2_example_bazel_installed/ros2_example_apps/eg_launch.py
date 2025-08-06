from python.runfiles import runfiles
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    # See bazel_ros2_rules/ros2/README.md, Launch Files, for notes on
    # features and limitations.
    r = runfiles.Create()
    prefix = "ros2_example_bazel_installed/ros2_example_apps"
    talker_bin = r.Rlocation(f"{prefix}/eg_talker")
    listener_bin = r.Rlocation(f"{prefix}/eg_listener")

    return LaunchDescription([
        # Running a talker written in python.
        ExecuteProcess(cmd=[talker_bin]),
        # Running a listener written in cpp.
        ExecuteProcess(cmd=[listener_bin]),
    ])
