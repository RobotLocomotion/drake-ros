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

template <typename T>
struct conventional_convert<drake::systems::BasicVector<T>,
                            std_msgs::msg::Float64> {
  static std::unique_ptr<std_msgs::msg::Float64> to_message(
      const drake::systems::BasicVector<T>& value) {
    auto message = std::make_unique<std_msgs::msg::Float64>();
    message->data = value[0];
    return message;
  }
};

}  // namespace utilities
}  // namespace drake_ros_introspection

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("introspection_demo_sine");

  drake::systems::DiagramBuilder<double> builder;

  constexpr double kAmplitude = 1.;
  constexpr double kFrequency = 1.;
  constexpr double kPhase = 1.;
  constexpr int kSize = 1;
  auto source_system = builder.AddSystem<drake::systems::Sine>(
      kAmplitude, kFrequency, kPhase, kSize);
  source_system->set_name("sine");

  std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
  diagram->set_name("introspection_demo");

  auto simulator = std::make_unique<drake::systems::Simulator<double>>(
      *diagram, diagram->CreateDefaultContext());
  simulator->set_target_realtime_rate(1.0);

  using drake_ros_introspection::SimulatorMonitor;
  using drake_ros_introspection::SimulatorMonitorBuilder;
  using drake_ros_introspection::predicates::DeclaredBy;
  using drake_ros_introspection::predicates::Each;
  SimulatorMonitorBuilder<double> simulator_monitor_builder;
  simulator_monitor_builder
      .For(Each<drake::systems::OutputPort>(DeclaredBy<drake::systems::Sine>()))
      .Expect<drake::systems::BasicVector>()
      .Publish<std_msgs::msg::Float64>();

  SimulatorMonitor<double> simulator_monitor =
      simulator_monitor_builder.Build(*diagram);
  simulator_monitor.Configure(node);
  simulator->set_monitor(simulator_monitor);

  simulator->Initialize();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const drake::systems::Context<double>& simulator_context =
      simulator->get_context();
  while (rclcpp::ok()) {
    simulator->AdvanceTo(simulator_context.get_time() + 0.1);
    executor.spin_once(std::chrono::nanoseconds::zero());
  }
  executor.remove_node(node);

  rclcpp::shutdown();
  return 0;
}
