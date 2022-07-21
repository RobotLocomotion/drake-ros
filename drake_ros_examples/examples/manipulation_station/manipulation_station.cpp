// Copyright 2021https://www.youtube.com/watch?v=SUQnduNzsw8 Open Source Robotics Foundation, Inc.
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

#include <drake/common/eigen_types.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/adder.h>
#include <drake/systems/primitives/constant_vector_source.h>
#include <drake/systems/primitives/sine.h>

#include <drake/examples/manipulation_station/manipulation_station.h>

#include <drake_ros_core/drake_ros.hpp>
#include <drake_ros_core/ros_interface_system.hpp>
#include <drake_ros_viz/rviz_visualizer.hpp>

#include <drake_ros_introspection/predicates.h>
#include <drake_ros_introspection/simulator_monitor.h>
#include <drake_ros_introspection/simulator_monitor_builder.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <memory>
#include <sstream>
#include <utility>

using drake_ros_core::DrakeRos;
using drake_ros_core::RosInterfaceSystem;
using drake_ros_viz::RvizVisualizer;

using drake::systems::Adder;
using drake::systems::ConstantVectorSource;
using drake::systems::Sine;
using drake::systems::Simulator;
using drake::examples::manipulation_station::ManipulationStation;

namespace drake_ros_introspection {
namespace utilities {

// This is a type converter.
// It is required to convert from the Drake type BasicVector<T>, which is the
// type of the output port in the Drake system, to the ROS interface type
// std_msgs::msg::Float64, which is the type that will be published over the
// ROS topic.
// It is a template parameterised by these two types, and provides a
// to_message() function that receives an instance of the Drake type and
// returns an instance the ROS interface type.
template <typename T>
struct conventional_convert<drake::systems::BasicVector<T>,
                            std_msgs::msg::Float64> {
  static std::unique_ptr<std_msgs::msg::Float64> to_message(
      const drake::systems::BasicVector<T>& value) {
    auto message = std::make_unique<std_msgs::msg::Float64>();
    // Copy the data from the Drake type into the ROS message
    message->data = value[0];
    return message;
  }
};

// This type converter is used to convert a Drake BasicVector<T> into
// a ROS sensor_msgs::msg::JointState.
template <typename T>
struct conventional_convert<drake::systems::BasicVector<T>,
                            sensor_msgs::msg::JointState> {
  static std::unique_ptr<sensor_msgs::msg::JointState> to_message(
      const drake::systems::BasicVector<T>& value) {
    auto message = std::make_unique<sensor_msgs::msg::JointState>();
    // Copy the data from the Drake type into the ROS message
    for (auto ii = 0; ii < value.size(); ++ii) {
      std::stringstream joint_name;
      joint_name << ii;
      message->name.push_back(joint_name.str());
      message->position.push_back(value[ii]);
    }
    return message;
  }
};

}  // namespace utilities
}  // namespace drake_ros_introspection


int main(int argc, char* argv[]) {
  // Create a ROS node that will be used to create publishers
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("introspection_demo_iiwa");

  drake::systems::DiagramBuilder<double> builder;

  // Add the Drake-ROS core system to the diagram
  auto ros_interface_system =
    builder.AddSystem<RosInterfaceSystem>(std::make_unique<DrakeRos>());

  // Add a manipulation station to the diagram
  auto manipulation_station = builder.AddSystem<ManipulationStation>();
  manipulation_station->SetupClutterClearingStation();
  manipulation_station->Finalize();

  // Make the base joint swing sinusoidally.
  auto constant_term = builder.AddSystem<ConstantVectorSource>(
    drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints()));

  drake::VectorX<double> amplitudes =
    drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints());
  amplitudes[0] = M_PI / 4.;  // = 45 degrees
  const drake::VectorX<double> frequencies =
    drake::VectorX<double>::Constant(
      manipulation_station->num_iiwa_joints(), 1.);  // Hz
  const drake::VectorX<double> phases =
    drake::VectorX<double>::Zero(manipulation_station->num_iiwa_joints());
  auto variable_term = builder.AddSystem<Sine>(amplitudes, frequencies, phases);

  auto joint_trajectory_generator =
    builder.AddSystem<Adder>(2, manipulation_station->num_iiwa_joints());

  // Connect up the joint trajectory generator and the iiwa robot
  builder.Connect(
    constant_term->get_output_port(),
    joint_trajectory_generator->get_input_port(0));
  builder.Connect(
    variable_term->get_output_port(0),
    joint_trajectory_generator->get_input_port(1));
  builder.Connect(
    joint_trajectory_generator->get_output_port(),
    manipulation_station->GetInputPort("iiwa_position"));

  // Add some ROS visualisation
  auto rviz_visualizer = builder.AddSystem<RvizVisualizer>(
    ros_interface_system->get_ros_interface());
  rviz_visualizer->RegisterMultibodyPlant(
    &manipulation_station->get_multibody_plant());
  builder.Connect(
    manipulation_station->GetOutputPort("query_object"),
    rviz_visualizer->get_graph_query_port());

  auto diagram = builder.Build();
  auto context = diagram->CreateDefaultContext();

  // Create a simulator to execute the manipulation station
  auto simulator = std::make_unique<Simulator<double>>(*diagram, std::move(context));
  simulator->set_target_realtime_rate(1.0);

  // Bring in the introspection types
  using drake_ros_introspection::SimulatorMonitor;
  using drake_ros_introspection::SimulatorMonitorBuilder;
  using drake_ros_introspection::predicates::DeclaredBy;
  using drake_ros_introspection::predicates::Each;
  using drake_ros_introspection::predicates::Named;
  // Create an instance of the type that constructs the SimulationMonitor
  SimulatorMonitorBuilder<double> simulator_monitor_builder;
  // This set of calls is a specification that builds up a type that will
  // receive a Drake type and publish it as a ROS interface type
  simulator_monitor_builder
      // For each OutputPort declared by ManipulationStation with the
      // name "iiwa_position_commanded"...
      .For(Each<drake::systems::OutputPort>(
        DeclaredBy<ManipulationStation>(),
        Named("iiwa_position_commanded")))
      // ... receive a drake::systems::BasicVector instance...
      .Expect<drake::systems::BasicVector>()
      // ... and publish a sensor_msgs::msg::JointState message
      .Publish<sensor_msgs::msg::JointState>();
  // Another specification to capture another specific port
  simulator_monitor_builder
      // For each OutputPort declared by ManipulationStation with the
      // name "iiwa_position_measured"...
      .For(Each<drake::systems::OutputPort>(
        DeclaredBy<ManipulationStation>(),
        Named("iiwa_position_measured")))
      // ... receive a drake::systems::BasicVector instance...
      .Expect<drake::systems::BasicVector>()
      // ... and publish a sensor_msgs::msg::JointState message
      .Publish<sensor_msgs::msg::JointState>();

  // Build the SimulationMonitor using the specification from above
  SimulatorMonitor<double> simulator_monitor =
      simulator_monitor_builder.Build(*diagram);
  simulator_monitor.Configure(node);
  // Connect the simulator and the simulator monitor
  simulator->set_monitor(simulator_monitor);

  simulator->Initialize();

  auto & simulator_context = simulator->get_mutable_context();

  auto & manipulation_station_context =
    diagram->GetMutableSubsystemContext(*manipulation_station, &simulator_context);

  auto & constant_term_context =
    diagram->GetMutableSubsystemContext(*constant_term, &simulator_context);

  // Fix gripper joints' position.
  manipulation_station->GetInputPort("wsg_position")
  .FixValue(&manipulation_station_context, 0.);

  // Use default positions for every joint but the base joint.
  drake::systems::BasicVector<double> & constants =
    constant_term->get_mutable_source_value(&constant_term_context);
  constants.set_value(
    manipulation_station->GetIiwaPosition(manipulation_station_context));
  constants.get_mutable_value()[0] = -M_PI / 4.;

  while (true) {
    // Advance the simulator one step
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
  }
  return 0;
}
