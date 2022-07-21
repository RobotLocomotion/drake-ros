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

#include <chrono>
#include <memory>
#include <utility>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/sine.h>
#include <drake_ros_introspection/predicates.h>
#include <drake_ros_introspection/simulator_monitor.h>
#include <drake_ros_introspection/simulator_monitor_builder.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

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

}  // namespace utilities
}  // namespace drake_ros_introspection

int main(int argc, char* argv[]) {
  // Create a ROS node that will be used to create publishers
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("introspection_demo_sine");

  drake::systems::DiagramBuilder<double> builder;

  // Create a sine wave generator system
  constexpr double kAmplitude = 1.;
  constexpr double kFrequency = 1.;
  constexpr double kPhase = 1.;
  constexpr int kSize = 1;
  auto source_system = builder.AddSystem<drake::systems::Sine>(
      kAmplitude, kFrequency, kPhase, kSize);
  source_system->set_name("sine");

  // Make a diagram to hold the sine wave generator and the ROS interfacing
  // system
  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
  diagram->set_name("introspection_demo");

  // Create a simulator to execute the sine wave generator
  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, diagram->CreateDefaultContext());
  simulator->set_target_realtime_rate(1.0);

  // Bring in the introspection types
  using drake_ros_introspection::SimulatorMonitor;
  using drake_ros_introspection::SimulatorMonitorBuilder;
  using drake_ros_introspection::predicates::DeclaredBy;
  using drake_ros_introspection::predicates::Each;
  // Create an instance of the type that constructs the SimulationMonitor
  SimulatorMonitorBuilder<double> simulator_monitor_builder;
  // This set of calls is a specification that builds up a type that will
  // receive a Drake type and publish it as a ROS interface type
  simulator_monitor_builder
      // For each OutputPort declared by drake::systems::Sine...
      .For(Each<drake::systems::OutputPort>(DeclaredBy<drake::systems::Sine>()))
      // ... receive a drake::systems::BasicVector instance...
      .ReceiveDrakeType<drake::systems::BasicVector>()
      // ... and publish a std_msgs::msg::Float64 message
      .PublishRosType<std_msgs::msg::Float64>();

  // Build the SimulationMonitor using the specification from above
  SimulatorMonitor<double> simulator_monitor =
      simulator_monitor_builder.Build(*diagram);
  simulator_monitor.Configure(node);
  // Connect the simulator and the simulator monitor
  simulator->set_monitor(simulator_monitor);

  simulator->Initialize();

  // Make sure the node that will be publishing the data received from the
  // Drake simulation receives execution time
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const drake::systems::Context<double>& simulator_context =
      simulator->get_context();
  while (rclcpp::ok()) {
    // Advance the simulator one step
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
    // Give the ROS node time to process any data that needs publishing
    executor.spin_once(std::chrono::nanoseconds::zero());
  }
  executor.remove_node(node);

  rclcpp::shutdown();
  return 0;
}
