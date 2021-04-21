#!/usr/bin/env python

from drake_ros_systems import RosClockSystem
from drake_ros_systems import RosInterfaceSystem
from drake_ros_systems import RvizVisualizer

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder


def main():
    builder = DiagramBuilder()

    ros_interface_system = builder.AddSystem(RosInterfaceSystem())

    manipulation_station = builder.AddSystem(ManipulationStation())
    manipulation_station.SetupClutterClearingStation()
    manipulation_station.Finalize()

    clock_system = builder.AddSystem(
        RosClockSystem(ros_interface_system.get_ros_interface()))

    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface()))

    builder.Connect(
        clock_system.GetOutputPort('clock'),
        rviz_visualizer.GetInputPort('clock')
    )

    builder.Connect(
        manipulation_station.GetOutputPort('query_object'),
        rviz_visualizer.GetInputPort('graph_query')
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
        while True:
            simulator.AdvanceTo(context.get_time() + 0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
