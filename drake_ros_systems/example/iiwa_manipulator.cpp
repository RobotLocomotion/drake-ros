// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>

#include <drake/examples/manipulation_station/manipulation_station.h>

#include <drake_ros_systems/drake_ros.hpp>
#include <drake_ros_systems/ros_clock_system.hpp>
#include <drake_ros_systems/ros_interface_system.hpp>
#include <drake_ros_systems/rviz_visualizer.hpp>

#include <memory>
#include <utility>

using drake_ros_systems::DrakeRos;
using drake_ros_systems::RosClockSystem;
using drake_ros_systems::RosInterfaceSystem;
using drake_ros_systems::RvizVisualizer;

using drake::examples::manipulation_station::ManipulationStation;


int main() {
  drake::systems::DiagramBuilder<double> builder;

  auto ros_interface_system =
    builder.AddSystem<RosInterfaceSystem>(std::make_unique<DrakeRos>());

  auto manipulation_station = builder.AddSystem<ManipulationStation>();
  manipulation_station->SetupClutterClearingStation();
  manipulation_station->Finalize();

  auto rviz_visualizer = builder.AddSystem<RvizVisualizer>(
    ros_interface_system->get_ros_interface());

  builder.Connect(
    manipulation_station->GetOutputPort("query_object"),
    rviz_visualizer->GetInputPort("graph_query"));

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  auto simulator =
    std::make_unique<drake::systems::Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);
  simulator->Initialize();

  auto & simulator_context = simulator->get_mutable_context();

  auto & manipulation_station_context =
    diagram->GetMutableSubsystemContext(*manipulation_station, &simulator_context);

  manipulation_station->GetInputPort("iiwa_position")
    .FixValue(
        &manipulation_station_context,
        manipulation_station->GetIiwaPosition(manipulation_station_context));

  manipulation_station->GetInputPort("wsg_position")
    .FixValue(&manipulation_station_context, 0.);

  while (true) {
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
  }
  return 0;

}
