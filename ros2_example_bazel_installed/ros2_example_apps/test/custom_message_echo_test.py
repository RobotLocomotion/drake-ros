import os
import subprocess

from bazel_tools.tools.python.runfiles import runfiles
from lib.ros_environment.unique import enforce_unique_ros_environment

def main():
    enforce_unique_ros_environment()

    r = runfiles.Create()
    ros2_bin = r.Rlocation("ros2_example_bazel_installed/tools/ros2")
    talker_bin = r.Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/simple_talker")

    timeout = 5.0
    topic_echo = subprocess.Popen([
        ros2_bin, "topic", "echo", "--once",
        "/status", "ros2_example_apps_msgs/msg/Status"])
    talker = subprocess.Popen([talker_bin])
    topic_echo.wait(timeout=timeout)
    assert topic_echo.returncode == 0
    talker.kill()

    print("[ Done ]")


if __name__ == "__main__":
    main()
