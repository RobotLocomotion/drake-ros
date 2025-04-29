import os
import subprocess
import sys

from bazel_ros_env import Rlocation, make_unique_ros_isolation_env


def test_launch_py():
    launch_bin = Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/roslaunch_eg_py"
    )
    subprocess.run([launch_bin], check=True)


def test_launch_xml():
    launch_bin = Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/roslaunch_eg_xml"
    )
    subprocess.run([launch_bin], check=True)


def main():
    os.environ.update(make_unique_ros_isolation_env())
    # For simplicity, run test points directly (rather than via pytest).
    test_launch_py()
    # test_launch_xml()


if __name__ == '__main__':
    main()
