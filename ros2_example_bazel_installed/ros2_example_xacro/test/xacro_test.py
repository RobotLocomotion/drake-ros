"""Tests that ros_xacro correctly processes xacro args and includes."""

import unittest

from python.runfiles import runfiles


class XacroTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        r = runfiles.Create()
        path = r.Rlocation(
            "ros2_example_bazel_installed/ros2_example_xacro/example.urdf"
        )
        with open(path) as f:
            cls.urdf = f.read()

    def test_sim_arg_is_substituted(self):
        """The sim xacro arg value appears in the output URDF."""
        self.assertIn("<sim>False</sim>", self.urdf)

    def test_local_package_macros_expanded(self):
        """Macros from the local my_robot package are resolved and expanded."""
        self.assertIn("aluminum", self.urdf)

    def test_relative_include_expanded(self):
        """A xacro included via a relative path (data=)"""
        self.assertIn('name="snippet_link"', self.urdf)

    def test_system_package_include_resolved(self):
        """$(find realsense2_description) is resolved and expanded."""
        self.assertIn('name="head_camera_link"', self.urdf)


if __name__ == "__main__":
    unittest.main()
