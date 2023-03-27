#include <regex>
#include <string>

#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pybind11/detail/descr.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <rclcpp/serialization.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

namespace py = pybind11;

// Generic (C++ <-> Python) typecaster for all ROS 2 messages.
#define ROS_MSG_PYBIND_TYPECAST_ALL()                                         \
  template <typename T>                                                       \
  struct type_caster<                                                         \
      T, std::enable_if_t<rosidl_generator_traits::is_message<T>::value>> {   \
   public:                                                                    \
    PYBIND11_TYPE_CASTER(T, _(""));                                           \
                                                                              \
    bool load(handle src, bool) { return RosMessagePyToCpp<T>(src, &value); } \
                                                                              \
    static handle cast(T src, return_value_policy policy, handle parent) {    \
      (void)policy;                                                           \
      (void)parent;                                                           \
      return RosMessageCppToPy<T>(src);                                       \
    }                                                                         \
  };

// Generic (C++ <-> Python) typecaster for a specific ROS 2 message.
#define ROS_MSG_PYBIND_TYPECAST(T)                                            \
  template <>                                                                 \
  struct type_caster<T> {                                                     \
   public:                                                                    \
    PYBIND11_TYPE_CASTER(T, _(""));                                           \
                                                                              \
    bool load(handle src, bool) { return RosMessagePyToCpp<T>(src, &value); } \
                                                                              \
    static handle cast(T src, return_value_policy policy, handle parent) {    \
      (void)policy;                                                           \
      (void)parent;                                                           \
      return RosMessageCppToPy<T>(src);                                       \
    }                                                                         \
  };

namespace PYBIND11_NAMESPACE {
namespace detail {

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
  py::handle cls = module::import(GetPythonMessagePackageName<T>().c_str())
                       .attr(GetPythonMessageName<T>().c_str());
  if (!isinstance(src, cls)) {
    return false;
  }
  py::object py_message = reinterpret_borrow<object>(src);

  // Check for type support
  object check_for_type_support =
      module::import("rclpy.type_support").attr("check_for_type_support");
  object msg_type = module::import(GetPythonMessagePackageName<T>().c_str())
                        .attr(GetPythonMessageName<T>().c_str());
  check_for_type_support(msg_type);

  py::object py_serialize =
      module::import("rclpy.serialization").attr("serialize_message");
  bytes pybytes = py_serialize(py_message);
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
      module::import("rclpy.serialization").attr("deserialize_message");
  py::object msg_type = module::import(GetPythonMessagePackageName<T>().c_str())
                            .attr(GetPythonMessageName<T>().c_str());
  std::string content_string(reinterpret_cast<char*>(rcl_handle.buffer),
                             serialized_message.size());
  content_string.push_back('\0');
  py::object instance = py_deserialize(py::bytes(content_string), msg_type);
  instance.inc_ref();
  return instance;
}

// Generic typecaster for all ROS 2 messages.
ROS_MSG_PYBIND_TYPECAST_ALL();

// Generic typecaster for specific ROS 2 messages.
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Quaternion);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Point);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Vector3);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Twist);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Accel);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Wrench);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Pose);
// ROS_MSG_PYBIND_TYPECAST(geometry_msgs::msg::Transform);

}  // namespace detail
}  // namespace PYBIND11_NAMESPACE
