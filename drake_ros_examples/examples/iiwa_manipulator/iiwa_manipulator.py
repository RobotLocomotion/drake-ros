#!/usr/bin/env python3
import numpy as np

import drake_ros.core
from drake_ros.core import RosInterfaceSystem
from drake_ros.viz import RvizVisualizer

from pydrake.examples.manipulation_station import ManipulationStation
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import Adder
from pydrake.systems.primitives import ConstantVectorSource
from pydrake.systems.primitives import Sine


def main():
    builder = DiagramBuilder()

    drake_ros.core.init()
    ros_interface_system = builder.AddSystem(RosInterfaceSystem("iiwa_manipulator_node"))

    manipulation_station = builder.AddSystem(ManipulationStation())
    manipulation_station.SetupClutterClearingStation()
    manipulation_station.Finalize()

    # Make the base joint swing sinusoidally.
    constant_term = builder.AddSystem(ConstantVectorSource(
        np.zeros(manipulation_station.num_iiwa_joints())))

    amplitudes = np.zeros(manipulation_station.num_iiwa_joints())
    amplitudes[0] = np.pi / 4.  # == 45 degrees
    frequencies = np.ones(manipulation_station.num_iiwa_joints())
    phases = np.zeros(manipulation_station.num_iiwa_joints())
    variable_term = builder.AddSystem(Sine(amplitudes, frequencies, phases))

    joint_trajectory_generator = builder.AddSystem(
        Adder(2, manipulation_station.num_iiwa_joints()))

    builder.Connect(constant_term.get_output_port(),
                    joint_trajectory_generator.get_input_port(0))
    builder.Connect(variable_term.get_output_port(0),
                    joint_trajectory_generator.get_input_port(1))
    builder.Connect(joint_trajectory_generator.get_output_port(),
                    manipulation_station.GetInputPort('iiwa_position'))

    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface()))

    rviz_visualizer.RegisterMultibodyPlant(
        manipulation_station.get_multibody_plant())
    rviz_visualizer.ComputeFrameHierarchy()

    builder.Connect(
        manipulation_station.GetOutputPort('query_object'),
        rviz_visualizer.get_graph_query_input_port()
    )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_mutable_context()

    manipulation_station_context = \
        diagram.GetMutableSubsystemContext(manipulation_station, context)
    constant_term_context = \
        diagram.GetMutableSubsystemContext(constant_term, context)

    # Fix gripper joints' position.
    manipulation_station.GetInputPort('wsg_position').FixValue(
        manipulation_station_context, np.zeros(1))

    # Use default positions for every joint but the base joint.
    constants = constant_term.get_mutable_source_value(constant_term_context)
    constants.set_value(
        manipulation_station.GetIiwaPosition(manipulation_station_context))
    constants.get_mutable_value()[0] = -np.pi / 4.

    try:
        while True:
            simulator.AdvanceTo(context.get_time() + 0.1)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
