#!/usr/bin/env python

import numpy as np

from drake_ros_systems import RosClockSystem
from drake_ros_systems import RosInterfaceSystem
from drake_ros_systems import RosPublisherSystem
from drake_ros_systems import TfBroadcasterSystem

from pydrake.common import FindResourceOrThrow
from pydrake.common.value import AbstractValue
from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.framework import TriggerType
from pydrake.systems.primitives import ConstantValueSource

from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def main():
    builder = DiagramBuilder()

    ros_interface_system = builder.AddSystem(RosInterfaceSystem())

    manipulation_station = builder.AddSystem(ManipulationStation())
    manipulation_station.SetupClutterClearingStation()
    manipulation_station.Finalize()

    clock_system = builder.AddSystem(
        RosClockSystem(ros_interface_system.get_ros_interface()))

    tf_broadcaster_system = builder.AddSystem(TfBroadcasterSystem(
        ros_interface_system.get_ros_interface(),
        {'iiwa': ''},  # drop iiwa namespace from frame names
        {TriggerType.kForced, TriggerType.kPeriodic},
        0.1))

    builder.Connect(
        clock_system.GetOutputPort('clock'),
        tf_broadcaster_system.GetInputPort('clock')
    )

    builder.Connect(
        manipulation_station.GetOutputPort('query_object'),
        tf_broadcaster_system.GetInputPort('graph_query')
    )

    # TODO(hidmic): fetch SDF path from ManipulationStation when/if it is exposed
    manipulator_sdf_path = FindResourceOrThrow(
        'drake/manipulation/models/iiwa_description/iiwa7/'
        'iiwa7_no_collision.sdf')
    with open(manipulator_sdf_path, 'r') as f:
        manipulator_sdf_string = f.read()

    robot_description_source = builder.AddSystem(
        ConstantValueSource(AbstractValue.Make(
            String(data=manipulator_sdf_string))))

    qos = QoSProfile(depth=1)
    qos.history = HistoryPolicy.KEEP_LAST
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE

    robot_description_publisher = builder.AddSystem(
        RosPublisherSystem(String, '/robot_description', qos,
                           ros_interface_system.get_ros_interface(),
                           {TriggerType.kForced}, 0))

    builder.Connect(
        robot_description_source.get_output_port(),
        robot_description_publisher.get_input_port()
    )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    manipulation_station_context = \
        diagram.GetMutableSubsystemContext(manipulation_station, context)
    manipulation_station.GetInputPort('iiwa_position').FixValue(
        manipulation_station_context,
        manipulation_station.GetIiwaPosition(manipulation_station_context))
    manipulation_station.GetInputPort('wsg_position').FixValue(
        manipulation_station_context, np.zeros(1))

    try:
        diagram.Publish(context)
        while True:
            simulator.AdvanceTo(context.get_time() + 0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
