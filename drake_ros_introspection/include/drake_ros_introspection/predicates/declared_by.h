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

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <drake/systems/framework/system_base.h>

#include <iostream>

namespace drake_ros_introspection {
namespace predicates {

namespace internal {

// A function that returns true if the type of value, T, is a subclass of BaseT.
template <typename BaseT, typename T>
bool isinstance(const T& value) {
  return (dynamic_cast<const BaseT*>(&value) != nullptr);
}

// A function that returns true if the type of value, T, is a subclass of BaseT, NextBaseT, or one
// of the classes in the BasesT parameter package.
template <typename BaseT, typename NextBaseT, typename... BasesT, typename T>
bool isinstance(const T& value) {
  return isinstance<BaseT>(value) || isinstance<NextBaseT, BasesT...>(value);
}

// A function object that checks if the type of an object instance is a subclass of one or more
// classes.
// A type closure over a list of classes.
// This function object can handle the base classes being templates, although they must all accept
// the same template parameters.
template <template <typename> class Base, template <typename> class... Bases>
class IsTemplateInstanceOf {
 public:
  template <template <typename> class Class, typename... ArgsT>
  bool operator()(const Class<ArgsT...>& instance) const {
    return isinstance<Base<ArgsT...>, Bases<ArgsT...>...>(instance);
  }
};

// A function object that checks if a System instance is the same as a list of System instances
// provided when the function object is instantiated.
// A closure over System instances.
template <typename T>
class IsOneOf {
 public:
  explicit IsOneOf(std::initializer_list<T*> instances)
      : instances_(instances) {}

  bool operator()(const T& instance) const {
    auto it = std::find(instances_.begin(), instances_.end(), &instance);
    return it != instances_.end();
  }

 private:
  const std::vector<T*> instances_;
};

// A function object that checks if the name of a System instance is the same as a list
// provided when the function object is instantiated.
// A closure over System names (as std::string instances).
class SystemNameMatches {
 public:
  explicit SystemNameMatches(std::initializer_list<std::string> names)
      : names_(names) {}

  bool operator()(const drake::systems::SystemBase& instance) const {
    auto it = std::find(names_.begin(), names_.end(), instance.get_name());
    if (it == names_.end()) {
      it =
          std::find(names_.begin(), names_.end(), instance.GetSystemPathname());
    }
    return it != names_.end();
  }

 private:
  std::vector<std::string> names_;
};

}  // namespace internal

// Returns a function object
// The function object has the signature bool(Object&)
// The function object returns true if Object inherits from one of the types in the Systems list
template <template <typename> class System,
          template <typename> class... Systems>
internal::IsTemplateInstanceOf<System, Systems...>
DeclaredBy() {
  return internal::IsTemplateInstanceOf<System, Systems...>{};
}

// Returns a function object
// The function object has the signature bool(System&)
// The function object returns true if System is one of the systems given in the arguments list
// when the function object was instantiated (i.e. the arguments passed to DeclaredBy())
template <typename SystemT, typename... SystemsT>
internal::IsOneOf<const drake::systems::SystemBase>
DeclaredBy(
    const SystemT* system, const SystemsT*... systems) {
  return internal::IsOneOf<const drake::systems::SystemBase>{system,
                                                             systems...};
}

// Returns a function object
// The function object has the signature bool(System&)
// The function object returns true if System has the same name as one of the names passed in to
// DeclaredBy()
template <typename NameT, typename... NamesT>
internal::SystemNameMatches
DeclaredBy(const NameT& name, const NamesT&... names) {
  return internal::SystemNameMatches{name, names...};
}

}  // namespace predicates

}  // namespace drake_ros_introspection
