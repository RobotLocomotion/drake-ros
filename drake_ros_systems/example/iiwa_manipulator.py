#!/usr/bin/env python3
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

import numpy as np

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

    rviz_visualizer = builder.AddSystem(
        RvizVisualizer(ros_interface_system.get_ros_interface()))

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
