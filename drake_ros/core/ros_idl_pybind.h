#pragma once

#include <regex>
#include <string>
#include <type_traits>

#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pybind11/detail/descr.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

namespace py = pybind11;

// Generic (C++ <-> Python) typecaster for all ROS 2 messages.
#define ROS_MSG_PYBIND_TYPECAST_ALL()                                     \
  namespace pybind11 {                                                    \
  namespace detail {                                                      \
  template <typename T>                                                   \
  struct type_caster<                                                     \
      T, std::enable_if_t<rosidl_generator_traits::is_message<T>::value>> \
      : public drake_ros::drake_ros_py::ros_message_type_caster<T> {};    \
  }                                                                       \
  }

// Generic (C++ <-> Python) typecaster for a specific ROS 2 message.
#define ROS_MSG_PYBIND_TYPECAST(T)                                     \
  namespace pybind11 {                                                 \
  namespace detail {                                                   \
  template <>                                                          \
  struct type_caster<T>                                                \
      : public drake_ros::drake_ros_py::ros_message_type_caster<T> {}; \
  }                                                                    \
  }

#define DRAKE_ROS_PYBIND_TYPECASTER(T, name)           \
  using handle = py::handle;                           \
  using none = py::none;                               \
  using return_value_policy = py::return_value_policy; \
  template <bool Condition, typename U>                \
  using enable_if_t = std::enable_if_t<Condition, U>;  \
  template <typename U>                                \
  using remove_cv_t = std::remove_cv_t<U>;             \
  PYBIND11_TYPE_CASTER(T, name)

namespace drake_ros {
namespace drake_ros_py {

// Get the name of the message as a string.
template <typename T>
std::string GetPythonMessageName() {
  std::string msg_name(rosidl_generator_traits::name<T>());
  auto pos = msg_name.find_last_of("/");
  return msg_name.substr(pos + 1);
}

// Get the package of the message as a string.
template <typename T>
std::string GetPythonMessagePackageName() {
  std::string msg_name(rosidl_generator_traits::name<T>());
  msg_name = std::regex_replace(msg_name, std::regex("/"), ".");
  auto pos = msg_name.find_last_of(".");
  return msg_name.substr(0, pos);
}

// Convert a Python ROS 2 message into a C++ equivalent.
template <typename T>
bool RosMessagePyToCpp(const py::handle& src, T* cpp_message) {
  py::handle cls = py::module::import(GetPythonMessagePackageName<T>().c_str())
                       .attr(GetPythonMessageName<T>().c_str());
  if (!isinstance(src, cls)) {
    return false;
  }
  py::object py_message = py::reinterpret_borrow<py::object>(src);

  // Check for type support
  py::object check_for_type_support =
      py::module::import("rclpy.type_support").attr("check_for_type_support");
  py::object msg_type =
      py::module::import(GetPythonMessagePackageName<T>().c_str())
          .attr(GetPythonMessageName<T>().c_str());
  check_for_type_support(msg_type);

  py::object py_serialize =
      py::module::import("rclpy.serialization").attr("serialize_message");
  py::bytes pybytes = py_serialize(py_message);
  const std::string content = pybytes;
  const auto content_size = content.size() + 1;

  rclcpp::SerializedMessage serialized_message(content_size);
  auto& rcl_handle = serialized_message.get_rcl_serialized_message();
  std::memcpy(rcl_handle.buffer, content.c_str(), content.size());
  rcl_handle.buffer[content.size()] = '\0';
  rcl_handle.buffer_length = content_size;

  rclcpp::Serialization<T> protocol;
  protocol.deserialize_message(&serialized_message, cpp_message);

  return true;
}

// Convert a C++ ROS 2 message into a Python equivalent.
template <typename T>
py::object RosMessageCppToPy(T src) {
  rclcpp::SerializedMessage serialized_message;
  rclcpp::Serialization<T> protocol;
  protocol.serialize_message(&src, &serialized_message);

  auto& rcl_handle = serialized_message.get_rcl_serialized_message();
  py::object py_deserialize =
      py::module::import("rclpy.serialization").attr("deserialize_message");
  py::object msg_type =
      py::module::import(GetPythonMessagePackageName<T>().c_str())
          .attr(GetPythonMessageName<T>().c_str());
  std::string content_string(reinterpret_cast<char*>(rcl_handle.buffer),
                             serialized_message.size());
  content_string.push_back('\0');
  py::object instance = py_deserialize(py::bytes(content_string), msg_type);
  instance.inc_ref();
  return instance;
}

template <typename T>
struct ros_message_type_caster {
 public:
  // TODO(Aditya): Apply constexpr message name to type caster macro
  // after https://github.com/ros2/rosidl/issues/734 is resolved.
  DRAKE_ROS_PYBIND_TYPECASTER(T, py::detail::_("RosTypeCaster[Unknown]"));

  bool load(py::handle src, bool) { return RosMessagePyToCpp<T>(src, &value); }

  static py::handle cast(T src, py::return_value_policy policy,
                         py::handle parent) {
    (void)policy;
    (void)parent;
    return RosMessageCppToPy<T>(src);
  }
};

}  // namespace drake_ros_py
}  // namespace drake_ros
