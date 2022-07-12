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

#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/system.h>
#include <drake/systems/framework/system_visitor.h>
#include <drake_ros_introspection/introspector.h>

#include <rclcpp/rclcpp.hpp>
#include <example_interfaces/msg/empty.hpp>

#include <functional>
#include <iostream>
#include <regex>
#include <string>
#include <sstream>
#include <vector>

namespace drake_ros_introspection {

template<typename T>
class IntrospectorBuilder {
public:
  void introspect(const drake::systems::System<T> & system)
  {
    systems_.push_back(&system);
  }

  Introspector<T> build()
  {
    Worker worker;
    for (const auto * system : systems_) {
      system->Accept(&worker);
    }
    return worker.YieldOutput();
  }

private:
  class Worker : public drake::systems::SystemVisitor<T> {
  public:
    void VisitSystem(const drake::systems::System<T> & system) override
    {
      std::cout << "Visiting system " << system.GetSystemName() <<
        " (" << system.GetSystemPathname() << ")\n";

      auto [name_space, name] = build_node_name_and_namespace(
          namespaces_,
          sanitise_node_name(system.GetSystemName()));

      std::cout << "Creating node " << name << " in namespace " << name_space << '\n';
      auto system_node = rclcpp::Node::make_shared(name, name_space);
      nodes_.push_back(system_node);

      for (int ii = 0; ii < system.num_output_ports(); ++ii) {
        auto & port = system.get_output_port(ii);
        std::cout << "Adding publisher to topic " << sanitise_port_name(port.get_name()) << '\n';
        auto publisher = system_node->template create_publisher<example_interfaces::msg::Empty>(
          sanitise_port_name(port.get_name()),
          1);
        publishers_.push_back(publisher);
      }

      for (int ii = 0; ii < system.num_input_ports(); ++ii) {
        auto & port = system.get_input_port(ii);
        auto subscription = system_node->template create_subscription(
          sanitise_port_name(port.get_name()),
          1,
          std::bind(&Worker::subscription_callback, this, std::placeholders::_1));
        subscriptions_.push_back(subscription);
      }
    }

    void VisitDiagram(const drake::systems::Diagram<T> & diagram) override
    {
      std::cout << "Visiting diagram " << diagram.GetSystemName() <<
        " (" << diagram.GetSystemPathname() << ")\n";

      if (diagram.GetSystemName() == "") {
        std::stringstream diagram_name;
        diagram_name << "unknown_diagram_" << unknown_diagram_counter_++;
        namespaces_.push_back(diagram_name.str());
      } else {
        namespaces_.push_back(sanitise_node_name(diagram.GetSystemName()));
      }
      std::cout << "Entering namespace " << namespaces_.back() << '\n';

      for (auto * system : diagram.GetSystems()) {
        system->Accept(this);
      }
      VisitSystem(diagram);
      std::cout << "Leaving namespace " << namespaces_.back() << '\n';
      namespaces_.pop_back();
    }

    Introspector<T> YieldOutput()
    {
      return Introspector<T>(nodes_, publishers_, subscriptions_);
    }

  private:
    std::vector<rclcpp::Node::SharedPtr> nodes_;
    std::vector<rclcpp::Publisher<example_interfaces::msg::Empty>::SharedPtr> publishers_;
    std::vector<rclcpp::Subscription<example_interfaces::msg::Empty>::SharedPtr> subscriptions_;

    std::vector<std::string> namespaces_;
    unsigned int unknown_diagram_counter_ = 0;
    unsigned int unknown_system_counter_ = 0;
    unsigned int node_counter_ = 0;
    unsigned int topic_counter_ = 0;

    std::string sanitise_port_name(std::string name) {
      return "port" + name;
    }

    std::string sanitise_node_name(std::string name) {
      std::regex bad_chars_re("@|/");
      return std::regex_replace(name, bad_chars_re, "_");
    }

    std::pair<std::string, std::string> build_node_name_and_namespace(
      const std::vector<std::string> namespaces,
      std::string system_name)
    {
      std::stringstream namespaces_string;
      for (auto & ns : namespaces) {
        namespaces_string << ns << '/';
      }
      std::string name_space_string = namespaces_string.str();
      if (name_space_string != "") {
        name_space_string.pop_back();
        if (name_space_string[0] == '_') {
          name_space_string = "root" + name_space_string;
        }
      }

      std::string node_name;
      if (system_name == "") {
        std::stringstream node_name_str;
        node_name_str << "unknown_system_" << unknown_system_counter_++;
        node_name = node_name_str.str();
      } else {
        node_name = system_name;
      }

      return std::make_pair(name_space_string, node_name);
    }

    void subscription_callback(const example_interfaces::msg::Empty &) const {
    }
  };

  std::vector<const drake::systems::System<T> *> systems_;
};

}  // namespace drake_ros_introspection
