# Copyright 2020 Open Source Robotics Foundation, Inc.
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

from drake_ros_core import RosInterfaceSystem
from drake_ros_core import RosPublisherSystem
from drake_ros_core import RosSubscriberSystem

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

import rclpy
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy

from test_msgs.msg import BasicTypes


def test_nominal_case():
    builder = DiagramBuilder()

    sys_ros_interface = builder.AddSystem(RosInterfaceSystem())

    qos = QoSProfile(
        depth=10,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    sys_pub = builder.AddSystem(
        RosPublisherSystem(BasicTypes, 'out_py', qos, sys_ros_interface.get_ros_interface()))

    sys_sub = builder.AddSystem(
        RosSubscriberSystem(BasicTypes, 'in_py', qos, sys_ros_interface.get_ros_interface()))

    builder.Connect(sys_sub.get_output_port(0), sys_pub.get_input_port(0))
    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator_context = simulator.get_mutable_context()
    simulator.set_target_realtime_rate(1.0)

    rclpy.init()
    node = rclpy.create_node('sub_to_pub_py')

    # Create publisher talking to subscriber system.
    publisher = node.create_publisher(BasicTypes, 'in_py', qos)

    # Create subscription listening to publisher system
    rx_msgs = []
    rx_callback = (lambda msg: rx_msgs.append(msg))
    node.create_subscription(BasicTypes, 'out_py', rx_callback, qos)

    # Send messages into the drake system
    num_msgs = 5
    for _ in range(num_msgs):
        publisher.publish(BasicTypes())

    timeout_sec = 5
    spins_per_sec = 10
    time_delta = 1.0 / spins_per_sec
    for _ in range(timeout_sec * spins_per_sec):
        if len(rx_msgs) >= num_msgs:
            break
        rclpy.spin_once(node, timeout_sec=time_delta)
        simulator.AdvanceTo(simulator_context.get_time() + time_delta)

    # Make sure same number of messages got out
    assert num_msgs == len(rx_msgs)
