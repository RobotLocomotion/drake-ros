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
#include <memory>
#include <unordered_set>

#include <drake/systems/framework/leaf_system.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros_core/drake_ros.h"
#include "drake_ros_core/qos_pybind.h"
#include "drake_ros_core/ros_interface_system.h"
#include "drake_ros_core/ros_publisher_system.h"
#include "drake_ros_core/ros_subscriber_system.h"
#include "drake_ros_core/serializer_interface.h"
#include "drake_ros_core/geometry_conversions.h"

namespace drake_ros_core {
namespace drake_ros_core_py {
namespace {

using drake::systems::LeafSystem;
using drake::systems::TriggerType;

// A (de)serialization interface implementation for Python ROS messages
// that can be overriden from Python itself.
class PySerializerInterface : public py::wrapper<SerializerInterface> {
 public:
  using Base = py::wrapper<SerializerInterface>;

  PySerializerInterface() : Base() {}

  const rosidl_message_type_support_t* GetTypeSupport() const override {
    auto overload = [&]() -> py::capsule {
      PYBIND11_OVERLOAD_PURE(py::capsule, SerializerInterface, GetTypeSupport);
    };
    return static_cast<rosidl_message_type_support_t*>(overload());
  }

  std::unique_ptr<drake::AbstractValue> CreateDefaultValue() const override {
    PYBIND11_OVERLOAD_PURE(std::unique_ptr<drake::AbstractValue>,
                           SerializerInterface, CreateDefaultValue);
  }

  rclcpp::SerializedMessage Serialize(
      const drake::AbstractValue& abstract_value) const override {
    auto overload = [&]() -> py::bytes {
      PYBIND11_OVERLOAD_PURE(py::bytes, SerializerInterface, Serialize,
                             &abstract_value);
    };
    std::string bytes = overload();
    rclcpp::SerializedMessage serialized_message(bytes.size());
    rcl_serialized_message_t& rcl_serialized_message =
        serialized_message.get_rcl_serialized_message();
    std::copy(bytes.data(), bytes.data() + bytes.size(),
              rcl_serialized_message.buffer);
    rcl_serialized_message.buffer_length = bytes.size();
    return serialized_message;
  }

  void Deserialize(const rclcpp::SerializedMessage& serialized_message,
                   drake::AbstractValue* abstract_value) const override {
    const rcl_serialized_message_t& rcl_serialized_message =
        serialized_message.get_rcl_serialized_message();
    py::bytes serialized_message_bytes(
        reinterpret_cast<const char*>(rcl_serialized_message.buffer),
        rcl_serialized_message.buffer_length);
    PYBIND11_OVERLOAD_PURE(void, SerializerInterface, Deserialize,
                           serialized_message_bytes, abstract_value);
  }
};

PYBIND11_MODULE(_drake_ros_core, m) {
  m.doc() = "Python bindings for drake_ros_core";
  // Force module name in docstrings to match
  // that of the outer module.
  m.attr("__name__") = "drake_ros_core";

  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.multibody.plant");

  // TODD(hidmic): populate Python docstrings with
  // C++ docstrings. Consider using mkdoc to keep
  // them in sync, like pydrake does.
  py::class_<DrakeRos>(m, "DrakeRos");

  m.def(
      "init",
      [](std::vector<std::string> args) {
        std::vector<const char*> raw_args;
        raw_args.reserve(args.size());
        for (auto& arg : args) {
          raw_args.push_back(arg.c_str());
        }
        init(raw_args.size(), raw_args.data());
      },
      py::arg("args") = std::vector<std::string>{});
  m.def("shutdown", &shutdown);

  py::class_<RosInterfaceSystem, LeafSystem<double>>(m, "RosInterfaceSystem")
      .def(py::init([](const std::string& node_name) {
        return std::make_unique<RosInterfaceSystem>(
            std::make_unique<DrakeRos>(node_name));
      }))
      .def("get_ros_interface", &RosInterfaceSystem::get_ros_interface,
           py::return_value_policy::reference_internal);

  py::class_<SerializerInterface, PySerializerInterface>(m,
                                                         "SerializerInterface")
      .def(py::init([]() { return std::make_unique<PySerializerInterface>(); }))
      .def("CreateDefaultValue", &SerializerInterface::CreateDefaultValue)
      .def("GetTypeSupport",
           [](const SerializerInterface& self) {
             return py::capsule(self.GetTypeSupport());
           })
      .def("Serialize",
           [](const SerializerInterface& self,
              const drake::AbstractValue& abstract_value) {
             rclcpp::SerializedMessage serialized_message =
                 self.Serialize(abstract_value);
             const rcl_serialized_message_t& rcl_serialized_message =
                 serialized_message.get_rcl_serialized_message();
             return py::bytes(
                 reinterpret_cast<const char*>(rcl_serialized_message.buffer),
                 rcl_serialized_message.buffer_length);
           })
      .def("Deserialize",
           [](const SerializerInterface& self, py::bytes raw_bytes,
              drake::AbstractValue* abstract_value) {
             std::string bytes = raw_bytes;
             rclcpp::SerializedMessage serialized_message(bytes.size());
             rcl_serialized_message_t& rcl_serialized_message =
                 serialized_message.get_rcl_serialized_message();
             std::copy(bytes.data(), bytes.data() + bytes.size(),
                       rcl_serialized_message.buffer);
             rcl_serialized_message.buffer_length = bytes.size();
             self.Deserialize(serialized_message, abstract_value);
           });

  py::class_<RosPublisherSystem, LeafSystem<double>>(m, "RosPublisherSystem")
      .def(py::init([](std::unique_ptr<SerializerInterface> serializer,
                       const char* topic_name, QoS qos, DrakeRos* ros_interface,
                       std::unordered_set<TriggerType> publish_triggers,
                       double publish_period) {
        return std::make_unique<RosPublisherSystem>(
            std::move(serializer), topic_name, qos, ros_interface,
            publish_triggers, publish_period);
      }));

  py::class_<RosSubscriberSystem, LeafSystem<double>>(m, "RosSubscriberSystem")
      .def(py::init([](std::unique_ptr<SerializerInterface> serializer,
                       const char* topic_name, QoS qos,
                       DrakeRos* ros_interface) {
        return std::make_unique<RosSubscriberSystem>(
            std::move(serializer), topic_name, qos, ros_interface);
      }));

  // Python bindings for geometry conversions.
  // Vector / Translation functions.
  m.def("ros_point_to_vector3", &drake_ros_core::RosPointToVector3);
  m.def("vector3_to_ros_point", &drake_ros_core::Vector3ToRosPoint);
  m.def("ros_vector3_to_vector3", &drake_ros_core::RosVector3ToVector3);
  m.def("vector3_to_ros_vector3", &drake_ros_core::Vector3ToRosVector3);

  // Orientation
  m.def("ros_quaternion_to_quaternion", &drake_ros_core::RosQuaternionToQuaternion);
  m.def("quaternion_to_ros_quaternion", &drake_ros_core::QuaternionToRosQuaternion);
  m.def("ros_quaternion_to_rotation_matrix", &drake_ros_core::RosQuaternionToRotationMatrix);
  m.def("rotation_matrix_to_ros_quaternion", &RotationMatrixToRosQuaternion);

  // Pose
  m.def("ros_pose_to_rigid_transform", &RosPoseToRigidTransform);
  m.def("rigid_transform_to_ros_pose", &RigidTransformToRosPose);
  m.def("ros_transform_to_rigid_transform", &RosTransformToRigidTransform);
  m.def("rigid_transform_to_ros_transform", &RigidTransformToRosTransform);
  m.def("ros_pose_to_isometry3", &RosPoseToIsometry3);
  m.def("isometry3_to_ros_pose", &Isometry3ToRosPose);
  m.def("ros_transform_to_isometry3", &RosTransformToIsometry3);
  m.def("isometry3_to_ros_transform", &Isometry3ToRosTransform);

  // Spatial Velocity
  m.def("ros_twist_to_vector6", &RosTwistToVector6);
  m.def("vector6_to_ros_twist", &Vector6ToRosTwist);
  m.def("ros_twist_to_spatial_velocity", &RosTwistToSpatialVelocity);
  m.def("spatial_velocity_to_ros_twist", &SpatialVelocityToRosTwist);

  // Spatial Acceleration
  m.def("ros_accel_to_vector6", &RosAccelToVector6);
  m.def("vector6_to_ros_accel", &Vector6ToRosAccel);
  m.def("ros_accel_to_spatial_acceleration", &RosAccelToSpatialAcceleration);
  m.def("spatial_acceleration_to_ros_accel", &SpatialAccelerationToRosAccel);

  // Spatial Force
  m.def("ros_wrench_to_vector6", &RosWrenchToVector6);
  m.def("vector6_to_ros_wrench", &Vector6ToRosWrench);
  m.def("ros_wrench_to_spatial_force", &RosWrenchToSpatialForce);
  m.def("spatial_force_to_ros_wrench", &SpatialForceToRosWrench);
}

}  // namespace
}  // namespace drake_ros_core_py
}  // namespace drake_ros_core
