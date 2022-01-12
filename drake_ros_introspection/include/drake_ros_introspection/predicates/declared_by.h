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

namespace drake_ros_introspection {
namespace predicates {

namespace internal {

template <typename BaseT, typename T>
bool isinstance(const T& value) {
  return (dynamic_cast<const BaseT*>(&value) != nullptr);
}

template <typename BaseT, typename NextBaseT, typename... BasesT, typename T>
bool isinstance(const T& value) {
  return isinstance<BaseT>(value) || isinstance<NextBaseT, BasesT...>(value);
}

template <template <typename> class Base, template <typename> class... Bases>
class IsTemplateInstanceOf {
 public:
  template <template <typename> class Class, typename... ArgsT>
  bool operator()(const Class<ArgsT...>& instance) const {
    return isinstance<Base<ArgsT...>, Bases<ArgsT...>...>(instance);
  }
};

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

template <template <typename> class System,
          template <typename> class... Systems>
internal::IsTemplateInstanceOf<System, Systems...> DeclaredBy() {
  return internal::IsTemplateInstanceOf<System, Systems...>{};
}

template <typename SystemT, typename... SystemsT>
internal::IsOneOf<const drake::systems::SystemBase> DeclaredBy(
    const SystemT* system, const SystemsT*... systems) {
  return internal::IsOneOf<const drake::systems::SystemBase>{system,
                                                             systems...};
}

template <typename NameT, typename... NamesT>
internal::SystemNameMatches DeclaredBy(const NameT& name,
                                       const NamesT&... names) {
  return internal::SystemNameMatches{name, names...};
}

}  // namespace predicates

}  // namespace drake_ros_introspection
