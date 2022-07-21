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

#include <string>
#include <utility>

namespace drake_ros_introspection {
namespace predicates {

namespace internal {

struct any {
  template <typename T>
  bool operator()(const T&) const {
    return true;
  }
};

}  // namespace internal

class Named {
 public:
  explicit Named(const std::string& name) : name_(name) {}

  template <class T>
  bool operator()(const T& instance) const {
    return instance.get_name() == name_;
  }

 private:
  const std::string name_;
};

class AtIndex {
 public:
  explicit AtIndex(int index) : index_(index) {}

  template <class T>
  bool operator()(const T& instance) const {
    return instance.get_index() == index_;
  }

 private:
  int index_;
};

template <template <typename> class Port, typename SystemPredicateT,
          typename IdentityPredicateT>
class PortPredicate {
 public:
  PortPredicate(SystemPredicateT system_predicate,
                IdentityPredicateT identity_predicate)
      : system_predicate_(std::move(system_predicate)),
        identity_predicate_(std::move(identity_predicate)) {}

  template <typename T>
  bool operator()(const Port<T>& port) const {
    return (system_predicate_(port.get_system()) && identity_predicate_(port));
  }

 private:
  const SystemPredicateT system_predicate_;
  const IdentityPredicateT identity_predicate_;
};

template <template <typename> class Port, typename SystemPredicateT,
          typename IdentityPredicateT>
PortPredicate<Port, SystemPredicateT, IdentityPredicateT> Each(
    SystemPredicateT&& system_predicate, IdentityPredicateT&& port_predicate) {
  return PortPredicate<Port, SystemPredicateT, IdentityPredicateT>{
      std::forward<SystemPredicateT>(system_predicate),
      std::forward<IdentityPredicateT>(port_predicate)};
}

template <template <typename> class Port, typename SystemPredicateT>
auto Each(SystemPredicateT&& identity_predicate) {
  return Each<Port>(std::forward<SystemPredicateT>(identity_predicate),
                    internal::any{});
}

}  // namespace predicates
}  // namespace drake_ros_introspection
