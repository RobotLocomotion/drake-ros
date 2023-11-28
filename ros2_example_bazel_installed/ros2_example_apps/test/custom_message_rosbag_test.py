"""
To run:
    export ROS2_DISTRO_PREFIX=/opt/ros/humble
    bazel test --nocache_test_results --test_output=streamed //ros2_example_apps:custom_message_rosbag_test
OR
    export ROS2_DISTRO_PREFIX=/opt/ros/humble
    bazel build //ros2_example_apps:custom_message_rosbag_test
    bazel-bin/ros2_example_apps/custom_message_rosbag_test
WARNING: `bazel run` does not work as expected
"""

import os
import shutil
import select
import subprocess
import time

from bazel_tools.tools.python.runfiles import runfiles
from rmw_isolation import isolate_rmw_by_path


def read_available(f, timeout=0.0, chunk_size=1024):
    """
    Reads all available data on a given file. Useful for using PIPE with Popen.
    """
    readable, _, _ = select.select([f], [], [f], timeout)
    out = None
    if f in readable:
        while True:
            cur = os.read(f.fileno(), chunk_size)
            if out is None:
                out = cur
            else:
                out += cur
            if len(cur) < chunk_size:
                break
    text = out.decode("utf-8")
    return text


def open_process(args):
    return subprocess.Popen(
        args,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        stdin=subprocess.PIPE,
        text=True,
    )


def attempt_record():
    if "TEST_TMPDIR" in os.environ:
        tmp_dir = os.environ["TEST_TMPDIR"]
        isolate_rmw_by_path(tmp_dir)
        os.environ["ROS_HOME"] = os.path.join(tmp_dir)
    else:
        tmp_dir = "/tmp"

    manifest = runfiles.Create()
    ros2_bin = manifest.Rlocation("ros2_example_bazel_installed/tools/ros2")
    assert ros2_bin is not None
    talker_bin = manifest.Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/simple_talker")
    bag_dir = os.path.join(tmp_dir, "test_bag")
    if os.path.exists(bag_dir):
        shutil.rmtree(bag_dir)

    try:
        talker = open_process([talker_bin])
        rosbag = open_process(
            [ros2_bin, "bag", "record", "--all", "-o", bag_dir]
        )

        time.sleep(1.0)

        assert talker.returncode is None, read_available(talker.stdout)
        assert rosbag.returncode is None, read_available(rosbag.stdout)

        text = read_available(rosbag.stdout)
        return text
    finally:
        talker.kill()
        rosbag.kill()


def main():
    text = attempt_record()
    print(text)

    assert "Recording..." in text
    assert "Subscribed to topic '/status'" in text

    # This is the error we're running into.
    assert "has unknown type" not in text
    assert "Failure in topics discovery" not in text


if __name__ == "__main__":
    main()
