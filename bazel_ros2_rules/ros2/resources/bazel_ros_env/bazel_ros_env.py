# TODO(#303): Move RMW + ROS isolation to a location accessible by CMake as
# well.

import functools
import os

from rmw_isolation import generate_isolated_rmw_env
from rules_python.python.runfiles import runfiles


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
    """  # noqa

    if unique_identifier is None:
        is_bazel_test = "TEST_TMPDIR" in environ
        # A PID should be unique for a single machine if we are not
        # running inside `bazel test`. This may or may not be true
        # inside of `bazel test - it is explicitly undefined.
        # https://bazel.build/reference/test-encyclopedia#initial-conditions
        if is_bazel_test:
            unique_identifier = os.environ["TEST_TMPDIR"]
        else:
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


@functools.lru_cache
def _runfiles():
    return runfiles.Create()


def Rlocation(path):
    return _runfiles().Rlocation(path)


def make_bazel_runfiles_env():
    env = dict(_runfiles().EnvVars())
    return env
