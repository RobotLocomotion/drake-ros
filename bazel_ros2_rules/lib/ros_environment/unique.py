import os


def make_unique_ros_environment(
    *,
    unique_identifier=None,
    scratch_directory=None,
    temp_dir=None,
):
    """
    Generates a unique ROS 2 environment for tests and processes.

    Note this environment does not imply isolation. 
    See //lib/network_isolation for same host isolation.  

    Warning:
        scratch_directory should generally be unique to prevent collisions
        among environments intended to be distinct.
    """  # noqa

    if unique_identifier is None:
        is_bazel_test = "TEST_TARGET" in os.environ
        # A PID should be unique for a single machine if we are not
        # running inside `bazel test`. This may or may not be true
        # inside of `bazel test - it is explicitly undefined.
        # https://bazel.build/reference/test-encyclopedia#initial-conditions
        if is_bazel_test:
            target_name = os.environ["TEST_TARGET"]
            unique_identifier = target_name.replace("//", "/")
            unique_identifier = unique_identifier.lstrip("/")
        else:
            unique_identifier = str(os.getpid())

    if temp_dir is None:
        temp_dir = os.environ.get("TEST_TMPDIR", "/tmp")

    if scratch_directory is None:
        scratch_directory = os.path.join(temp_dir, unique_identifier)
        if not os.path.isdir(scratch_directory):
            os.makedirs(scratch_directory, exist_ok=True)

    assert unique_identifier is not None
    assert scratch_directory is not None
    assert os.path.isdir(scratch_directory), scratch_directory
    env = {}
    # ROS wants to write text logs, so point it to the temporary
    # test directory via ROS_HOME.
    env["ROS_HOME"] = os.path.join(scratch_directory, "ros")
    return env


def enforce_unique_ros_environment(**kwargs):
    os.environ.update(make_unique_ros_environment(**kwargs))
