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

#include <iostream>

namespace drake_ros_introspection {

// This object constructs a simulation monitor matching a system.
// T is the type of the system. e.g. if the system is a Sine<double> then T is double.
// There are two entry points:
// 1. The For() functions are used to add rules to the builder that will construct port probes.
// 2. The Build() function actually constructs the SimulatorMonitor<T> object that performs
// monitoring of a simulation.
template <typename T>
class SimulatorMonitorBuilder {
 public:
  // Add a rule to the blueprint that is used to find matching InputPort<T>s in the relevant
  // systems.
  // Returns an object that can be used to access the added rule to add additional information to
  // it, such as the Drake type used by the port or the builder that actually creates a port probe.
  rules::AbstractPortProbingRuleWriter<drake::systems::InputPort<T>> For(
      std::function<bool(const drake::systems::InputPort<T>&)> matcher) {
    blueprint_.input_port_probing_rules.push_back({std::move(matcher)});
    return rules::AbstractPortProbingRuleWriter<drake::systems::InputPort<T>>(
        blueprint_.input_port_probing_rules.back());
  }

  // Add a rule to the blueprint that is used to find matching OutputPort<T>s in the relevant
  // systems.
  // Returns an object that can be used to access the added rule to add additional information to
  // it, such as the Drake type used by the port or the builder that actually creates a port probe.
  rules::AbstractPortProbingRuleWriter<drake::systems::OutputPort<T>> For(
      std::function<bool(const drake::systems::OutputPort<T>&)> matcher) {
    blueprint_.output_port_probing_rules.push_back({std::move(matcher)});
    return rules::AbstractPortProbingRuleWriter<drake::systems::OutputPort<T>>(
        blueprint_.output_port_probing_rules.back());
  }

  // Call this to actually create the SimulatorMonitor<T> object, once all the necessary port
  // rules have been added to the blueprint.
  // This will create a worker that subclasses the Drake SystemVisitor. This worker is passed to
  // the system that is to be monitored, which causes its VisitDiagram() and VisitSystem()
  // functions to be called. Once the entire system (or all the systems in the diagram, if that's
  // what it is) has been visited, the SimulatorMonitor<T> instance is returned. As long as it
  // exists, Drake will update the port probes that it contains for each step of the simulation.
  SimulatorMonitor<T> Build(const drake::systems::System<T>& system) {
    Worker worker{blueprint_};
    system.Accept(&worker);
    return worker.YieldOutput();
  }

 private:
  // This stores the blueprint for a SimulatorMonitor. In other words, it stores the rules that are
  // used to construct port probes for the ports of a system to be monitored.
  // The port probes should be added to the SimulatorMonitorBuilder using the For() functions.
  struct Blueprint {
    using InputPortProbingRule =
        rules::PortProbingRule<drake::systems::InputPort<T>>;
    std::vector<InputPortProbingRule> input_port_probing_rules;

    using OutputPortProbingRule =
        rules::PortProbingRule<drake::systems::OutputPort<T>>;
    std::vector<OutputPortProbingRule> output_port_probing_rules;
  };

  // A visitor that is given to a Drake system or diagram, and visits that system or every system
  // in the diagram.
  // For each system, it iterates over all the ports the system contains and checks if any of them
  // match any of the rules stored in the blueprint that the worker is passed when it is
  // instantiated. If a rule matches a port, a port probe is constructed for that port and stored
  // in the worker. Yielding the result of the worker's visit to the system/diagram will return a
  // SimulatorMonitor<T> object instance that holds those probes.
  class Worker : public drake::systems::SystemVisitor<T> {
   public:
    // Constructor for the Worker object. It receives a blueprint to use when visiting the system.
    explicit Worker(const Blueprint& blueprint) : blueprint_(blueprint) {}

    // Visit a single Drake system.
    // This iterates over all input and output ports in the system, looking for ports that match
    // the rules contained in the blueprint so that probes can be added for each matching port.
    void VisitSystem(const drake::systems::System<T>& system) override {
      BuildInputPortProbes(system);
      BuildOutputPortProbes(system);
    }

    // Visit a Drake diagram.
    // This visits each system contained in the diagram, before visiting the diagram itself as a
    // system. In other words, it enables recursive visiting of composed systems.
    void VisitDiagram(const drake::systems::Diagram<T>& diagram) override {
      for (auto* system : diagram.GetSystems()) {
        system->Accept(this);
      }
      VisitSystem(diagram);
    }

    // Return a SimulatorMonitor<T> object instance containing the port probes that were
    // constructed during the visit to the Drake system.
    SimulatorMonitor<T> YieldOutput() {
      return SimulatorMonitor<T>{std::move(probes_)};
    }

   private:
    // Iterate over each input port in the system currently being visited.
    // For each port, iterate over the InputPort probing rules stored in the blueprint and see if
    // any match.
    // For those that match, call the related port probe builder to build a probe, and store that
    // probe in the input port probes list.
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

    // Iterate over each output port in the system currently being visited.
    // For each port, iterate over the OutputPort probing rules stored in the blueprint and see if
    // any match.
    // For those that match, call the related port probe builder to build a probe, and store that
    // probe in the output port probes list.
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

    // A reference to the blueprint.
    const Blueprint& blueprint_;

    // Port probes created during system visitation.
    std::vector<std::unique_ptr<probes::ProbeInterface<T>>> probes_{};
  };

  // The blueprint that stores the rules for matching input and output pors when the
  // SimulatorMonitor instance is constructed.
  Blueprint blueprint_{};
};

}  // namespace drake_ros_introspection
