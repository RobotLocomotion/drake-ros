#include <memory>
#include <unordered_set>

#include <drake/systems/framework/leaf_system.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/core/geometry_conversions.h"
#include "drake_ros/core/geometry_conversions_pybind.h"
#include "drake_ros/core/qos_pybind.h"
#include "drake_ros/core/ros_interface_system.h"
#include "drake_ros/core/ros_publisher_system.h"
#include "drake_ros/core/ros_subscriber_system.h"
#include "drake_ros/core/serializer_interface.h"
#include "drake_ros/drake_ros_pybind.h"

namespace drake_ros {
namespace drake_ros_py DRAKE_ROS_NO_EXPORT {

using drake_ros::core::DrakeRos;
using drake_ros::core::init;
using drake_ros::core::RosInterfaceSystem;
using drake_ros::core::RosPublisherSystem;
using drake_ros::core::RosSubscriberSystem;
using drake_ros::core::SerializerInterface;
using drake_ros::core::shutdown;

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

void DefCore(py::module m) {
  m.doc() = "Python bindings for drake_ros.core";

  py::module::import("numpy");
  py::module::import("pydrake.systems.framework");
  py::module::import("pydrake.multibody.plant");
  py::module::import("pydrake.math");
  py::module::import("pydrake.common.eigen_geometry");

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
  m.def("RosPointToVector3", &drake_ros::core::RosPointToVector3,
        py::arg("point"));
  m.def("Vector3ToRosPoint", &drake_ros::core::Vector3ToRosPoint,
        py::arg("point"));
  m.def("RosVector3ToVector3", &drake_ros::core::RosVector3ToVector3,
        py::arg("point"));
  m.def("Vector3ToRosVector3", &drake_ros::core::Vector3ToRosVector3,
        py::arg("point"));

  // Orientation
  m.def("RosQuaternionToQuaternion",
        &drake_ros::core::RosQuaternionToQuaternion, py::arg("quat"));
  m.def("QuaternionToRosQuaternion",
        &drake_ros::core::QuaternionToRosQuaternion, py::arg("quat"));
  m.def("RosQuaternionToRotationMatrix",
        &drake_ros::core::RosQuaternionToRotationMatrix, py::arg("quat"));
  m.def("RotationMatrixToRosQuaternion",
        &drake_ros::core::RotationMatrixToRosQuaternion, py::arg("rotation"));

  // Pose
  m.def("RosPoseToRigidTransform", &drake_ros::core::RosPoseToRigidTransform,
        py::arg("pose"));
  m.def("RigidTransformToRosPose", &drake_ros::core::RigidTransformToRosPose,
        py::arg("transform"));
  m.def("RosTransformToRigidTransform",
        &drake_ros::core::RosTransformToRigidTransform, py::arg("transform"));
  m.def("RigidTransformToRosTransform",
        &drake_ros::core::RigidTransformToRosTransform, py::arg("transform"));
  m.def("RosPoseToIsometry3", &drake_ros::core::RosPoseToIsometry3,
        py::arg("pose"));
  m.def("Isometry3ToRosPose", &drake_ros::core::Isometry3ToRosPose,
        py::arg("isometry"));
  m.def("RosTransformToIsometry3", &drake_ros::core::RosTransformToIsometry3,
        py::arg("transform"));
  m.def("Isometry3ToRosTransform", &drake_ros::core::Isometry3ToRosTransform,
        py::arg("isometry"));

  // Spatial Velocity
  m.def("RosTwistToVector6", &drake_ros::core::RosTwistToVector6,
        py::arg("twist"));
  m.def("Vector6ToRosTwist", &drake_ros::core::Vector6ToRosTwist,
        py::arg("vector"));
  m.def("RosTwistToSpatialVelocity",
        &drake_ros::core::RosTwistToSpatialVelocity, py::arg("twist"));
  m.def("SpatialVelocityToRosTwist",
        &drake_ros::core::SpatialVelocityToRosTwist, py::arg("velocity"));

  // Spatial Acceleration
  m.def("RosAccelToVector6", &drake_ros::core::RosAccelToVector6,
        py::arg("accel"));
  m.def("Vector6ToRosAccel", &drake_ros::core::Vector6ToRosAccel,
        py::arg("vector"));
  m.def("RosAccelToSpatialAcceleration",
        &drake_ros::core::RosAccelToSpatialAcceleration, py::arg("accel"));
  m.def("SpatialAccelerationToRosAccel",
        &drake_ros::core::SpatialAccelerationToRosAccel, py::arg("accel"));

  // Spatial Force
  m.def("RosWrenchToVector6", &drake_ros::core::RosWrenchToVector6,
        py::arg("wrench"));
  m.def("Vector6ToRosWrench", &drake_ros::core::Vector6ToRosWrench,
        py::arg("vector"));
  m.def("RosWrenchToSpatialForce", &drake_ros::core::RosWrenchToSpatialForce,
        py::arg("wrench"));
  m.def("SpatialForceToRosWrench", &drake_ros::core::SpatialForceToRosWrench,
        py::arg("force"));
}
// clang-format off
}  // namespace drake_ros_py
// clang-format on
}  // namespace drake_ros
