import os
import subprocess

from bazel_tools.tools.python.runfiles import runfiles
from bazel_ros2_rules.lib.ros_environment.unique import enforce_unique_ros_environment


def main():
    enforce_unique_ros_environment()
    
    manifest = runfiles.Create()
    ros2_bin = manifest.Rlocation("ros2_example_bazel_installed/tools/ros2")

    interfaces = subprocess.run(
        [ros2_bin, "interface", "list"],
        check=True, text=True, stdout=subprocess.PIPE,
    ).stdout
    print(interfaces)
    assert "ros2_example_apps_msgs/msg/Status" in interfaces

    print("[ Done ]")


if __name__ == "__main__":
    main()
