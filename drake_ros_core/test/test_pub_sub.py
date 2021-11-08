# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from test_msgs.msg import BasicTypes

from drake_ros_core import RosInterfaceSystem
from drake_ros_core import RosPublisherSystem
from drake_ros_core import RosSubscriberSystem


def test_nominal_case():
    builder = DiagramBuilder()

    ros_interface_system = builder.AddSystem(RosInterfaceSystem())

    qos = QoSProfile(
        depth=10,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    ros_publisher_system = builder.AddSystem(RosPublisherSystem.Make(
        BasicTypes, 'out_py', qos, ros_interface_system.get_ros_interface()))

    ros_subscriber_system = builder.AddSystem(RosSubscriberSystem.Make(
        BasicTypes, 'in_py', qos, ros_interface_system.get_ros_interface()))

    builder.Connect(
        ros_subscriber_system.get_output_port(0),
        ros_publisher_system.get_input_port(0))
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    rclpy.init()
    node = rclpy.create_node('sub_to_pub_py')

    # Create publisher talking to subscriber system.
    publisher = node.create_publisher(BasicTypes, 'in_py', qos)

    # Create subscription listening to publisher system
    num_msgs = 5
    rx_msgs = []

    def rx_callback(msg):
        # Cope with lack of synchronization between subscriber
        # and publisher systems by ignoring duplicate messages.
        if not rx_msgs or rx_msgs[-1].uint64_value != msg.uint64_value:
            rx_msgs.append(msg)
    node.create_subscription(BasicTypes, 'out_py', rx_callback, qos)

    num_msgs_sent = 0
    timeout_sec = 5
    spins_per_sec = 10
    time_delta = 1.0 / spins_per_sec
    for _ in range(timeout_sec * spins_per_sec):
        if len(rx_msgs) >= num_msgs:
            break
        # Cope with lack of synchronization between subscriber
        # and publisher systems by sending one message at a time.
        if len(rx_msgs) == num_msgs_sent:
            # Send messages into the drake system
            message = BasicTypes()
            message.uint64_value = num_msgs_sent
            publisher.publish(message)
            num_msgs_sent = num_msgs_sent + 1
        rclpy.spin_once(node, timeout_sec=time_delta)
        simulator.AdvanceTo(simulator_context.get_time() + time_delta)

    # Make sure same number of messages got out
    assert num_msgs == len(rx_msgs)
    # Make sure all messages got out and in the right order
    for i in range(num_msgs):
        assert rx_msgs[i].uint64_value == i
