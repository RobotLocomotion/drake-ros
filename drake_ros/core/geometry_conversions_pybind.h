#include <string>

#include <Eigen/Geometry>
#include <drake/common/eigen_types.h>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <rclcpp/serialization.hpp>

namespace py = pybind11;

#define ROS_MSG_TYPECAST(PKG_NAME, PKG_SUBDIR, MSG_NAME)                       \
  template <>                                                                  \
  struct type_caster<PKG_NAME::PKG_SUBDIR::MSG_NAME> {                         \
   public:                                                                     \
    PYBIND11_TYPE_CASTER(PKG_NAME::PKG_SUBDIR::MSG_NAME,                       \
                         _(#PKG_NAME "." #PKG_SUBDIR "." #MSG_NAME));          \
                                                                               \
    bool load(handle src, bool) {                                              \
      handle cls = module::import(#PKG_NAME "." #PKG_SUBDIR).attr(#MSG_NAME);  \
      if (!isinstance(src, cls)) {                                             \
        return false;                                                          \
      }                                                                        \
      object source = reinterpret_borrow<object>(src);                         \
                                                                               \
      object check_for_type_support =                                          \
          module::import("rclpy.type_support").attr("check_for_type_support"); \
      object msg_type =                                                        \
          module::import(#PKG_NAME "." #PKG_SUBDIR).attr(#MSG_NAME);           \
      check_for_type_support(msg_type);                                        \
      object py_serialize =                                                    \
          module::import("rclpy.serialization").attr("serialize_message");     \
      bytes pybytes = py_serialize(source);                                    \
      const std::string content = pybytes;                                     \
      const auto content_size = content.size() + 1;                            \
                                                                               \
      rclcpp::SerializedMessage serialized_message(content_size);              \
      auto& rcl_handle = serialized_message.get_rcl_serialized_message();      \
      std::memcpy(rcl_handle.buffer, content.c_str(), content.size());         \
      rcl_handle.buffer[content.size()] = '\0';                                \
      rcl_handle.buffer_length = content_size;                                 \
                                                                               \
      rclcpp::Serialization<PKG_NAME::PKG_SUBDIR::MSG_NAME> protocol;          \
      protocol.deserialize_message(&serialized_message, &value);               \
      return true;                                                             \
    }                                                                          \
                                                                               \
    static handle cast(PKG_NAME::PKG_SUBDIR::MSG_NAME src,                     \
                       return_value_policy policy, handle parent) {            \
      (void)policy;                                                            \
      (void)parent;                                                            \
                                                                               \
      rclcpp::SerializedMessage serialized_message;                            \
      rclcpp::Serialization<PKG_NAME::PKG_SUBDIR::MSG_NAME> protocol;          \
      protocol.serialize_message(&src, &serialized_message);                   \
                                                                               \
      auto& rcl_handle = serialized_message.get_rcl_serialized_message();      \
      object py_deserialize =                                                  \
          module::import("rclpy.serialization").attr("deserialize_message");   \
      object msg_type =                                                        \
          module::import(#PKG_NAME "." #PKG_SUBDIR).attr(#MSG_NAME);           \
      std::string content_string(reinterpret_cast<char*>(rcl_handle.buffer),   \
                                 serialized_message.size());                   \
      content_string.push_back('\0');                                          \
      object instance = py_deserialize(bytes(content_string), msg_type);       \
                                                                               \
      instance.inc_ref();                                                      \
      return instance;                                                         \
    }                                                                          \
  };

namespace PYBIND11_NAMESPACE {
namespace detail {

ROS_MSG_TYPECAST(geometry_msgs, msg, Quaternion)
ROS_MSG_TYPECAST(geometry_msgs, msg, Point)
ROS_MSG_TYPECAST(geometry_msgs, msg, Vector3)
ROS_MSG_TYPECAST(geometry_msgs, msg, Twist)
ROS_MSG_TYPECAST(geometry_msgs, msg, Accel)
ROS_MSG_TYPECAST(geometry_msgs, msg, Wrench)
ROS_MSG_TYPECAST(geometry_msgs, msg, Pose)
ROS_MSG_TYPECAST(geometry_msgs, msg, Transform)

}  // namespace detail
}  // namespace PYBIND11_NAMESPACE
