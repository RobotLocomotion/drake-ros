import subprocess

from lib.ros_environment.unique import enforce_unique_ros_environment
from python.runfiles import runfiles


def main():
    enforce_unique_ros_environment()

    r = runfiles.Create()
    ros2_bin = r.Rlocation("ros2_example_bazel_installed/tools/ros2")

    interfaces = subprocess.run(
        [ros2_bin, "interface", "list"],
        check=True,
        text=True,
        stdout=subprocess.PIPE,
    ).stdout
    print(interfaces)
    assert "ros2_example_apps_msgs/msg/Status" in interfaces

    print("[ Done ]")


if __name__ == "__main__":
    main()
