import os
import random
import string
import sys
import threading
import time

import numpy as np
import pytest
import rclpy
import rclpy.executors
import rclpy.node
from visualization_msgs.msg import Marker, MarkerArray

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource

import drake_ros.core
from drake_ros.core import RosInterfaceSystem
from drake_ros.viz import RvizVisualizer


def isolate_if_using_bazel():
    # Do not require `make_unique_ros_isolation_env` module for CMake.
    # TODO(eric.cousineau): Expose this to CMake in better location..
    try:
        from bazel_ros_env import make_unique_ros_isolation_env
        os.environ.update(make_unique_ros_isolation_env())
    except ImportError:
        assert "TEST_TMPDIR" not in os.environ


class ManagedSubscription:
    def __init__(
            self,
            topic_name='/scene_markers/visual',
            required_message_count=1):
        self._context = rclpy.Context()
        self._topic_name = topic_name
        self._required_message_count = required_message_count
        self._received_messages = []
        self._spin_complete = threading.Event()

    def __enter__(self):
        self._context.init()

        # Use a randomised node name to enable parallel usage
        random.seed()
        self._node = rclpy.node.Node('managed_subscription_{}'.format(
            ''.join(random.choices(string.ascii_letters, k=10))),
            context=self._context)

        # TODO(gbiggs): When this is upstreamed, the topic type needs to be
        # parameterised.
        # TODO(gbiggs): When this is upstreamed, should it be possible to
        # subscribe to multiple topics ("ManagedSubscriptions")? Or should
        # there be a one-to-one relationship between a ManagedSubscription and
        # a subscription?
        self._subscription = self._node.create_subscription(
            MarkerArray,
            self._topic_name,
            self.callback,
            10)

        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self._context.try_shutdown()

    def callback(self, message):
        self._received_messages.append(message)

    def spin_subscription(self, timeout=10.0):
        self._spin_complete.clear()
        self._spinning_thread = threading.Thread(
            target=lambda to: self.spinner(time.monotonic_ns(), to),
            args=([int(timeout) * 1000000000]))
        self._spinning_thread.start()

    def spinner(self, start_time, timeout):
        executor = rclpy.executors.SingleThreadedExecutor(
            context=self._context)
        executor.add_node(self._node)
        while self.continue_spinning(start_time, timeout):
            executor.spin_once(timeout_sec=1)
        self._spin_complete.set()

    def continue_spinning(self, start_time, timeout):
        if len(self._received_messages) >= self._required_message_count:
            return False
        if (time.monotonic_ns() - start_time) > timeout:
            return False
        return True

    def wait_for_and_get_received_messages(self):
        self._spinning_thread.join()
        return self._received_messages

    def spin_complete(self):
        return self._spin_complete.is_set()

if __name__ == '__main__':
    isolate_if_using_bazel()
    sys.exit(pytest.main(sys.argv))
