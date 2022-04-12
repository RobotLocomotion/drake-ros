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

#include <iostream>

namespace drake_ros_introspection {
namespace predicates {

namespace internal {

// A function object port predicate that always returns true. Use it as the IdentityPredicate to
// match any port on a system.
struct any {
  template <typename T>
  bool operator()(const T&) const {
    return true;
  }
};

}  // namespace internal

// A function object that returns true if the provided port instance has a name matching the name
// passed in when the function object is instantiated.
// A closure over a port name as a std::string.
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

// A function object that returns true if the provided port instance is at the index on its system
// matching the index passed in when the function object is instantiated.
// A closure over a port index as an int.
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

// A function object that 
// A closure over a system predicate function object and a port identity predicate function object.
// When instantiated it receives instances of the system predicate and port identity predicate to
// use to check if a port matches.
// When called it returns true if both the system predicate and the identity predicate match for
// the port (i.e. they both return true).
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

// Returns a function object.
// The function object has the signature bool(Port<T>&).
// Port is the type of the input or output Drake port to call the function object on
// SystemPredicateT is the type of the predicate that is called on the port to match the system; it
// has the signature bool(Port<T>&).
// IdentityPredicateT is the type of the preidcate that is called on the port to match the identity
// of the port; it has the signature bool(Port<T>&). Available predicates are AtIndex and Named.
// The function object returns true if both the SystemPredicateT instance and the
// IdentityPredicateT instance return true when called and passed the Port<T> instance.
template <template <typename> class Port, typename SystemPredicateT,
          typename IdentityPredicateT>
PortPredicate<Port, SystemPredicateT, IdentityPredicateT> Each(
    SystemPredicateT&& system_predicate, IdentityPredicateT&& port_predicate) {
  return PortPredicate<Port, SystemPredicateT, IdentityPredicateT>{
      std::forward<SystemPredicateT>(system_predicate),
      std::forward<IdentityPredicateT>(port_predicate)};
}

// Returns a function object (the same type as returned by the above Each definition).
// The function object has the signature bool(Port<T>&).
// SystemPredicateT is the type of the predicate that is called on the port to match the system; it
// has the signature bool(Port<T>&).
// The function object is an instance of PortPredicate where the identity predicate is an instance
// of internal::any. This means it can match any port on a system where the system predicate
// matches.
template <template <typename> class Port, typename SystemPredicateT>
auto Each(SystemPredicateT&& identity_predicate) {
  return Each<Port>(std::forward<SystemPredicateT>(identity_predicate),
                    internal::any{});
}

}  // namespace predicates
}  // namespace drake_ros_introspection
