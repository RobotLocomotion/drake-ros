#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <rclcpp/serialization.hpp>

namespace py = pybind11;

#define ROS_MSG_TYPECAST(cpp_msg_type, py_package, py_msg, py_msg_combined)\
  template <>\
  struct type_caster<cpp_msg_type> {\
   public:\
    PYBIND11_TYPE_CASTER(cpp_msg_type,\
                         _(py_msg_combined));\
  \
    bool load(handle src, bool) {\
      handle cls = module::import(py_package).attr(py_msg);\
      if (!isinstance(src, cls)) {\
        return false;\
      }\
      object source = reinterpret_borrow<object>(src);\
  \
      object py_serialize = module::import("rclpy.serialization").attr("serialize_message");\
      bytes pybytes = py_serialize(source);\
      const std::string content = pybytes;\
      const auto content_size = content.size() + 1;\
  \
      rclcpp::SerializedMessage serialized_message(content_size);\
      auto & rcl_handle = serialized_message.get_rcl_serialized_message();\
      std::memcpy(rcl_handle.buffer, content.c_str(), content.size());\
      rcl_handle.buffer[content.size()] = '\0';\
      rcl_handle.buffer_length = content_size;\
  \
      rclcpp::Serialization<cpp_msg_type> protocol;\
      protocol.deserialize_message(&serialized_message, &value);\
  \
      return true;\
    }\
  \
    static handle cast(cpp_msg_type src,\
                       return_value_policy policy, handle parent) {\
      (void)policy;\
      (void)parent;\
  \
      rclcpp::SerializedMessage serialized_message;\
      rclcpp::Serialization<cpp_msg_type> protocol;\
      protocol.serialize_message(&src, &serialized_message);\
  \
      auto & rcl_handle = serialized_message.get_rcl_serialized_message();\
      std::vector<uint8_t> content;\
      content.resize(serialized_message.size());\
      std::memcpy(content.data(), rcl_handle.buffer, serialized_message.size());\
  \
      object py_deserialize = module::import("rclpy.serialization").attr("deserialize_message");\
      object msg_type = module::import(py_package).attr(py_msg);\
      std::string content_string(content.begin(), content.end());\
      object instance = py_deserialize(bytes(content_string), msg_type);\
  \
      instance.inc_ref();\
      return instance;\
    }\
  };


namespace PYBIND11_NAMESPACE {
namespace detail {

ROS_MSG_TYPECAST(geometry_msgs::msg::Quaternion, "geometry_msgs.msg", "Quaternion", "geometry_msgs.msg.Quaternion")
ROS_MSG_TYPECAST(geometry_msgs::msg::Point, "geometry_msgs.msg", "Point", "geometry_msgs.msg.Point")
ROS_MSG_TYPECAST(geometry_msgs::msg::Vector3, "geometry_msgs.msg", "Vector3", "geometry_msgs.msg.Vector3")
ROS_MSG_TYPECAST(geometry_msgs::msg::Twist, "geometry_msgs.msg", "Twist", "geometry_msgs.msg.Twist")
ROS_MSG_TYPECAST(geometry_msgs::msg::Accel, "geometry_msgs.msg", "Accel", "geometry_msgs.msg.Accel")
ROS_MSG_TYPECAST(geometry_msgs::msg::Wrench, "geometry_msgs.msg", "Wrench", "geometry_msgs.msg.Wrench")
ROS_MSG_TYPECAST(geometry_msgs::msg::Pose, "geometry_msgs.msg", "Pose", "geometry_msgs.msg.Pose")
ROS_MSG_TYPECAST(geometry_msgs::msg::Transform, "geometry_msgs.msg", "Transform", "geometry_msgs.msg.Transform")

}  // namespace detail
}  // namespace PYBIND11_NAMESPACE
