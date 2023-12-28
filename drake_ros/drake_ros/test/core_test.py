import os
import sys
import warnings

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import TriggerType

import pytest

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from std_msgs.msg import Int32
from test_msgs.msg import BasicTypes

import drake_ros.core
from drake_ros.core import CppNode
from drake_ros.core import CppNodeOptions
from drake_ros.core import DrakeRos
from drake_ros.core import RosInterfaceSystem
from drake_ros.core import RosPublisherSystem
from drake_ros.core import RosSubscriberSystem
from drake_ros_test_pub_and_sub_cc import CppPubAndSub


def isolate_if_using_bazel():
    # Do not require `make_unique_ros_isolation_env` module for CMake.
    # TODO(eric.cousineau): Expose this to CMake in better location..
    try:
        from bazel_ros_env import make_unique_ros_isolation_env
        os.environ.update(make_unique_ros_isolation_env())
    except ImportError:
        assert "TEST_TMPDIR" not in os.environ


@pytest.fixture
def drake_ros_fixture():
    drake_ros.core.init()
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()
        drake_ros.core.shutdown()


def test_nominal_case(drake_ros_fixture):
    builder = DiagramBuilder()

    system_ros = builder.AddSystem(
        RosInterfaceSystem('pub_to_sub_py'))

    publish_period = 1.0

    qos = QoSProfile(
        depth=10,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE)

    system_pub_out = builder.AddSystem(RosPublisherSystem.Make(
        BasicTypes, 'out_py', qos, system_ros.get_ros_interface(),
        {TriggerType.kPeriodic}, publish_period))

    system_sub_in = builder.AddSystem(RosSubscriberSystem.Make(
        BasicTypes, 'in_py', qos, system_ros.get_ros_interface()))

    builder.Connect(
        system_sub_in.get_output_port(0),
        system_pub_out.get_input_port(0))
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()

    simulator_context = simulator.get_mutable_context()

    direct_ros_node = rclpy.create_node('sub_to_pub_py')

    # Create publisher talking to subscriber system.
    direct_pub_in = direct_ros_node.create_publisher(BasicTypes, 'in_py', qos)

    # Create subscription listening to publisher system
    rx_msgs_direct_sub_out = []

    def rx_callback_direct_sub_out(msg):
        rx_msgs_direct_sub_out.append(msg)
    direct_sub_out = direct_ros_node.create_subscription(
        BasicTypes, 'out_py', rx_callback_direct_sub_out, qos)

    # Wait for the subscriber to connect
    for i in range(1, 10):
        if direct_pub_in.get_subscription_count() > 0:
            break
        rclpy.spin_once(direct_ros_node, timeout_sec=0.1)
    else:
        assert False, 'Timeout waiting for publisher and subscriber to connect'

    pub_sub_rounds = 5
    for i in range(1, pub_sub_rounds + 1):
        rx_msgs_count_before_pubsub = len(rx_msgs_direct_sub_out)
        # Publish a message to the drake ros subscriber system.
        message = BasicTypes()
        message.uint64_value = i
        direct_pub_in.publish(message)
        # Step forward to allow the message to be dispatched to the drake ros
        # subscriber system. The drake ros publisher system should not publish
        # just yet.
        rclpy.spin_once(direct_ros_node, timeout_sec=0.)
        simulator.AdvanceTo(simulator_context.get_time() + publish_period / 2.)
        assert len(rx_msgs_direct_sub_out) == rx_msgs_count_before_pubsub
        # Step forward until it is about time the drake ros publisher
        # publishes. Allow the message to be dispatched to the direct
        # subscription.
        simulator.AdvanceTo(simulator_context.get_time() + publish_period / 2.)
        rclpy.spin_once(direct_ros_node, timeout_sec=0.)
        rx_msgs_count_after_pubsub = rx_msgs_count_before_pubsub + 1
        assert len(rx_msgs_direct_sub_out) == rx_msgs_count_after_pubsub
        assert rx_msgs_direct_sub_out[-1].uint64_value == i


def assert_equal_for_cyclone_dds(actual, expected, name):
    rmw_implementation = rclpy.utilities.get_rmw_implementation_identifier()
    is_cyclone_dds = rmw_implementation == "rmw_cyclonedds_cpp"
    if is_cyclone_dds:
        assert actual == expected
    else:
        # TODO(#286): At least for rmw_fastrtps_cpp, the behavior of publishing
        # and receiving a message seems much less reliable than cyclone. Demote
        # errors to warnings in this case.
        if actual != expected:
            warnings.warn(
                f"WARNING! For RMW_IMPLEMENTATION={rmw_implementation}, "
                f"incorrect value for {name}: {actual} != {expected}"
            )


def test_cpp_node(drake_ros_fixture):
    # Create a Python node.
    node_py = rclpy.create_node("node_py")
    sub_value_py = -1

    def on_sub(message):
        nonlocal sub_value_py
        sub_value_py = message.data

    node_py.create_subscription(Int32, "/cpp_pub", on_sub, 1)
    pub_py = node_py.create_publisher(Int32, "/cpp_sub", 1)

    # Create a C++ node.
    cpp_node_options = CppNodeOptions(use_global_arguments=False)
    assert not cpp_node_options.use_global_arguments
    node_cpp = CppNode("direct_node_cpp", node_options=cpp_node_options)
    assert node_cpp.get_name() == "direct_node_cpp"
    # Create a "fixture" that will publish and subscribe to above topics.
    fixture = CppPubAndSub(node=node_cpp)
    timeout_sec = 0.0

    # Show that we can publish from C++ to Python.
    fixture.Publish(value=10)
    rclpy.spin_once(node_py, timeout_sec=timeout_sec)
    assert_equal_for_cyclone_dds(sub_value_py, 10, "sub_value_py")

    # Show that we can publish from Python to C++.
    message = Int32()
    message.data = 100
    pub_py.publish(message)
    sub_value_cpp = fixture.SpinAndReturnLatest(timeout_sec=timeout_sec)
    assert_equal_for_cyclone_dds(sub_value_cpp, 100, "sub_value_cpp")


def test_drake_ros(drake_ros_fixture):
    drake_ros = DrakeRos("sample_node")
    assert isinstance(drake_ros.get_node(), CppNode)


def main():
    isolate_if_using_bazel()
    sys.exit(pytest.main(sys.argv))


if __name__ == '__main__':
    main()
