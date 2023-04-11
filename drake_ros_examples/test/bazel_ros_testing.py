# TODO(eric.cousineau): Hoist this to rmw_isolation / bazel_ros2_rules.

import functools
import os

from rmw_isolation import generate_isolated_rmw_env
from rules_python.python.runfiles import runfiles


@functools.lru_cache
def _runfiles():
    return runfiles.Create()


def Rlocation(path):
    return _runfiles().Rlocation(path)


def make_unique_ros_isolation_env(
    *,
    unique_identifier=None,
    scratch_directory=None,
    environ=os.environ,
    temp_dir=None,
):
    """
    Generates ROS 2 environment variables suitable for isolating tests and/or
    processes on (at least) the same machine.

    Warning:
        scratch_directory should generally be unique to prevent collisions
        among environments intended to be distinct.

    For more info, see:
    https://github.com/RobotLocomotion/drake-ros/blob/main/bazel_ros2_rules/ros2/resources/rmw_isolation/rmw_isolation.py
    """  # noqa

    if unique_identifier is None:
        # A PID should be unique for a single machine.
        unique_identifier = str(os.getpid())

    if temp_dir is None:
        temp_dir = environ.get("TEST_TMPDIR", "/tmp")

    if scratch_directory is None:
        scratch_directory = os.path.join(temp_dir, unique_identifier)
        if not os.path.isdir(scratch_directory):
            os.mkdir(scratch_directory)

    assert unique_identifier is not None
    assert scratch_directory is not None
    assert os.path.isdir(scratch_directory), scratch_directory
    env = generate_isolated_rmw_env(
        unique_identifier=unique_identifier,
        scratch_directory=scratch_directory,
    )
    # Limit ROS traffic to a single machine.
    env["ROS_LOCALHOST_ONLY"] = "1"
    # ROS wants to write text logs, so point it to the temporary test directory
    # via ROS_HOME.
    env["ROS_HOME"] = os.path.join(scratch_directory, "fake_ros_home")
    return env


def make_bazel_runfiles_env():
    env = dict(_runfiles().EnvVars())
    return env


def maybe_make_test_ros_isolation_env():
    env = dict()
    if "TEST_TMPDIR" in os.environ:
        # Only isolate when testing.
        env.update(make_unique_ros_isolation_env())
    return env
