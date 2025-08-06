import os
import subprocess
import sys

from bazel_tools.tools.python.runfiles import runfiles
from lib.ros_environment.unique import enforce_unique_ros_environment


def test_launch_py():
    r = runfiles.Create()
    launch_bin = r.Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/roslaunch_eg_py"
    )
    subprocess.run([launch_bin], check=True)


def test_launch_xml():
    r = runfiles.Create()
    launch_bin = r.Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/roslaunch_eg_xml"
    )
    subprocess.run([launch_bin], check=True)


def main():
    enforce_unique_ros_environment()
    # For simplicity, run test points directly (rather than via pytest).
    test_launch_py()
    test_launch_xml()


if __name__ == '__main__':
    main()
