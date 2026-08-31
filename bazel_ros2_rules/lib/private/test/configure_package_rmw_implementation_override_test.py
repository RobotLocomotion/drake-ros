import unittest

from ros2bzl.scraping.properties import CcProperties
from ros2bzl.templates import configure_package_rmw_implementation_override
from ros2bzl.utilities import interpolate


def identity_sandbox(library):
    return library


class ConfigurePackageRmwImplementationOverrideTest(unittest.TestCase):
    def test_package_with_matching_library(self):
        properties = CcProperties(
            link_libraries=[
                "/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so",
                "/opt/ros/jazzy/lib/libddsc.so.0",
            ]
        )
        target_name, template, config = (
            configure_package_rmw_implementation_override(
                "rmw_cyclonedds_cpp", properties, identity_sandbox
            )
        )
        self.assertEqual(target_name, "rmw_cyclonedds_cpp_as_rmw_implementation")
        self.assertIsNotNone(template)
        content = interpolate(template, config)
        self.assertIn(
            'name = "rmw_cyclonedds_cpp_as_rmw_implementation"', content
        )
        self.assertIn(
            'srcs = ["/opt/ros/jazzy/lib/librmw_cyclonedds_cpp.so"]', content
        )
        self.assertIn(
            'outs = ["rmw_cyclonedds_cpp/librmw_implementation.so"]', content
        )

    def test_package_without_matching_library(self):
        properties = CcProperties(
            link_libraries=["/opt/ros/jazzy/lib/libsome_other_thing.so"]
        )
        target_name, template, config = (
            configure_package_rmw_implementation_override(
                "rmw_cyclonedds_cpp", properties, identity_sandbox
            )
        )
        self.assertEqual(target_name, "rmw_cyclonedds_cpp_as_rmw_implementation")
        self.assertIsNone(template)
        self.assertIsNone(config)


if __name__ == "__main__":
    unittest.main()
