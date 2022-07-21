// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#pragma once

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/input_port.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/framework/system_visitor.h>
#include <drake_ros_introspection/probes/probe_interface.h>
#include <drake_ros_introspection/rules/port_probing_rule.h>
#include <drake_ros_introspection/simulator_monitor.h>

namespace drake_ros_introspection {

template <typename T>
class SimulatorMonitorBuilder {
 public:
  rules::AbstractPortProbingRuleWriter<drake::systems::InputPort<T>> For(
      std::function<bool(const drake::systems::InputPort<T>&)> matcher) {
    blueprint_.input_port_probing_rules.push_back({std::move(matcher)});
    return rules::AbstractPortProbingRuleWriter<drake::systems::InputPort<T>>(
        blueprint_.input_port_probing_rules.back());
  }

  rules::AbstractPortProbingRuleWriter<drake::systems::OutputPort<T>> For(
      std::function<bool(const drake::systems::OutputPort<T>&)> matcher) {
    blueprint_.output_port_probing_rules.push_back({std::move(matcher)});
    return rules::AbstractPortProbingRuleWriter<drake::systems::OutputPort<T>>(
        blueprint_.output_port_probing_rules.back());
  }

  SimulatorMonitor<T> Build(const drake::systems::System<T>& system) {
    Worker worker{blueprint_};
    system.Accept(&worker);
    return worker.YieldOutput();
  }

 private:
  struct Blueprint {
    using InputPortProbingRule =
        rules::PortProbingRule<drake::systems::InputPort<T>>;
    std::vector<InputPortProbingRule> input_port_probing_rules;

    using OutputPortProbingRule =
        rules::PortProbingRule<drake::systems::OutputPort<T>>;
    std::vector<OutputPortProbingRule> output_port_probing_rules;
  };

  class Worker : public drake::systems::SystemVisitor<T> {
   public:
    explicit Worker(const Blueprint& blueprint) : blueprint_(blueprint) {}

    void VisitSystem(const drake::systems::System<T>& system) override {
      BuildInputPortProbes(system);
      BuildOutputPortProbes(system);
    }

    void VisitDiagram(const drake::systems::Diagram<T>& diagram) override {
      for (auto* system : diagram.GetSystems()) {
        system->Accept(this);
      }
      VisitSystem(diagram);
    }

    SimulatorMonitor<T> YieldOutput() {
      return SimulatorMonitor<T>{std::move(probes_)};
    }

   private:
    void BuildInputPortProbes(const drake::systems::System<T>& system) {
      if (!blueprint_.input_port_probing_rules.empty()) {
        for (int i = 0; i < system.num_input_ports(); ++i) {
          auto& input_port = system.get_input_port(i);
          for (const auto& rule : blueprint_.input_port_probing_rules) {
            if (rule.matches(input_port)) {
              probes_.push_back(rule.builder->Build(input_port));
            }
          }
        }
      }
    }

    void BuildOutputPortProbes(const drake::systems::System<T>& system) {
      for (int i = 0; i < system.num_output_ports(); ++i) {
        auto& output_port = system.get_output_port(i);
        for (auto& rule : blueprint_.output_port_probing_rules) {
          if (rule.matches(output_port)) {
            probes_.push_back(rule.builder->Build(output_port));
          }
        }
      }
    }

    const Blueprint& blueprint_;

    std::vector<std::unique_ptr<probes::ProbeInterface<T>>> probes_{};
  };

  Blueprint blueprint_{};
};

}  // namespace drake_ros_introspection
