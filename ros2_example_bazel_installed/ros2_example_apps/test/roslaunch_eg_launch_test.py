# -*- python -*-
# Copyright 2024 Open Source Robotics Foundation

"""launch_testing example: verify talker/listener exchange via ros_launch_test.

This test spins up the eg_talker and eg_listener from the roslaunch_eg_py
launch binary, then checks exit codes in a post-shutdown test.

To run via Bazel:
    bazel test //ros2_example_apps:roslaunch_eg_launch_test
"""

import unittest

import launch
import launch.actions
import launch.event_handlers
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from python.runfiles import runfiles as runfiles_api


@launch_testing.markers.keep_alive
def generate_test_description():
    r = runfiles_api.Create()

    # Locate the ros_launch binary via Bazel runfiles.
    # The binary already carries eg_talker and eg_listener in its ament index
    # because roslaunch_eg_py was declared with executables=[":eg_listener",
    # ":eg_talker"]. No additional ament registration is needed here.
    launch_bin = r.Rlocation(
        "ros2_example_bazel_installed/ros2_example_apps/roslaunch_eg_py"
    )

    launch_action = launch.actions.ExecuteProcess(
        cmd=[launch_bin],
        output="screen",
    )

    # Fire ReadyToTest() only after roslaunch_eg_py exits on its own.
    # This prevents launch_testing from sending SIGINT before the talker and
    # listener have finished exchanging their 10 messages.
    return (
        launch.LaunchDescription(
            [
                launch_action,
                launch.actions.RegisterEventHandler(
                    launch.event_handlers.OnProcessExit(
                        target_action=launch_action,
                        on_exit=[launch_testing.actions.ReadyToTest()],
                    )
                ),
            ]
        ),
        {"launch_proc": launch_action},
    )


@launch_testing.post_shutdown_test()
class TestExitCode(unittest.TestCase):
    def test_launch_exit_code(self, launch_proc, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info, [0], launch_proc)
