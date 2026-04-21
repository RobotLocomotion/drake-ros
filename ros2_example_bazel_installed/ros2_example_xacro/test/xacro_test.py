"""Tests that ros_xacro correctly processes xacro args and includes."""

from python.runfiles import runfiles


def _read_urdf():
    r = runfiles.Create()
    path = r.Rlocation(
        "ros2_example_bazel_installed/ros2_example_xacro/example.urdf"
    )
    with open(path) as f:
        return f.read()


def test_sim_arg_is_substituted():
    """The sim xacro arg value appears in the output URDF."""
    content = _read_urdf()
    assert "<sim>False</sim>" in content, (
        "Expected '<sim>False</sim>' in URDF output:\n" + content
    )


def test_local_package_macros_expanded():
    """Macros from the local my_robot package are resolved and expanded."""
    content = _read_urdf()
    # The macros.xacro defines materials; verify they appear in the output.
    assert "aluminum" in content, (
        "Expected materials from my_robot/urdf/macros.xacro in output:\n"
        + content
    )


def test_relative_include_expanded():
    """A xacro included via a relative path (data=) appears in the output."""
    content = _read_urdf()
    assert 'name="snippet_link"' in content, (
        "Expected snippet_link from snippet.xacro in output:\n" + content
    )


def test_system_package_include_resolved():
    """$(find realsense2_description) is resolved and its macros are expanded.

    Verifies that ros_xacro sets AMENT_PREFIX_PATH so xacro can find
    system-installed ROS packages. The D435i macro from
    realsense2_description is instantiated in the example xacro; if the
    include had not resolved, xacro would have exited non-zero and the
    output URDF would not exist.
    """
    content = _read_urdf()
    assert 'name="head_camera_link"' in content, (
        "Expected D435i links from realsense2_description in output:\n"
        + content
    )


if __name__ == "__main__":
    test_sim_arg_is_substituted()
    test_local_package_macros_expanded()
    test_relative_include_expanded()
    test_system_package_include_resolved()
    print("All tests passed.")
