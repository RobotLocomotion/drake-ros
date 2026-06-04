# -*- python -*-
# Copyright 2024 Open Source Robotics Foundation

"""launch_testing Pattern B example: launch nodes via launch_ros.actions.Node.

Unlike roslaunch_eg_launch_test.py (which locates the binary via Rlocation),
this test refers to nodes by their ament package/executable name.  The ament
index entries for eg_talker and eg_listener are made visible at test runtime
because roslaunch_eg_py (which registered them via executables=) is listed in
the data= of the ros_launch_test target.

To run via Bazel:
    bazel test //ros2_example_apps:roslaunch_eg_node_launch_test
"""

import unittest

import launch
import launch.actions
import launch.event_handlers
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers


@launch_testing.markers.keep_alive
def generate_test_description():
    talker = Node(
        package="ros2_example_bazel_installed",
        executable="eg_talker",
        output="screen",
    )
    listener = Node(
        package="ros2_example_bazel_installed",
        executable="eg_listener",
        output="screen",
    )

    # The listener exits first (transient-local QoS lets it buffer all 10
    # messages quickly).  Fire ReadyToTest() from the talker's OnProcessExit
    # so the transition to post-shutdown tests happens only after both nodes
    # have exited cleanly.  keep_alive prevents LaunchService from
    # auto-shutting down when the first node exits.
    return (
        launch.LaunchDescription(
            [
                talker,
                listener,
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessExit(
                        target_action=talker,
                        on_exit=[launch_testing.actions.ReadyToTest()],
                    )
                ),
            ]
        ),
        {"talker": talker, "listener": listener},
    )


@launch_testing.post_shutdown_test()
class TestExitCodes(unittest.TestCase):
    def test_talker_exit_code(self, talker, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, [0], talker)

    def test_listener_exit_code(self, listener, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, [0], listener)
