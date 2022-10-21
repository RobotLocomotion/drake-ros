/**
Provides mechanisms to convert messages.

Also provides affordances for declaring pybind11 value type casters for
messages. This is in contrast to binding the C++ messages themselves.
*/

namespace drake_ros_core {
namespace drake_ros_core_py {
namespace internal {

inline std::vector<uint8_t> SerializedToBytes(
    const rclcpp::SerializedMessage& serialized) {
  std::vector<uint8_t> bytes;
  const int size = serialized.size();
  bytes.resize(size);
  const rcl_serialized_message_t& rcl_serialized =
      serialized.get_rcl_serialized_message();
  const void* src = rcl_serialized.buffer;
  void* dest = bytes.data();
  std::copy(src, src + size, dest);
  return bytes;
}

inline rclcpp::SerializedMessage BytesToSerialized(
    const std::vector<uint8_t>& bytes) {
  const int size = bytes.size();
  rclcpp::SerializedMessage serialized(size);
  rcl_serialized_message_t& rcl_serialized =
      serialized.get_rcl_serialized_message();
  const void* src = bytes.data();
  void* dest = rcl_serialized.buffer;
  std::copy(src, src + size, dest);
  return serialized;
}

template <typename Message>
rclcpp::SerializedMessage MessageToSerialized(const Message& message) {
  rclcpp::SerializedMessage serialized;
  rclcpp::Serialization<CppMessage> protocol;
  protocol.serialize_message(&message, &serialized);
  return serialized;
}

template <typename Message>
void SerializedToMessage(
    const rclcpp::SerializedMessage& serialized, Message* message) {
  rclcpp::Serialization<CppMessage> protocol;
  protocol.deserialize_message(&serialized, message);
}

}  // internal

/**
Converts C++ message to a Python message.

Example usage:

  anzu_lifelong::msg::LifelongTaskState cpp_msg;
  py::object py_msg_cls =
      py::module_::import("anzu_lifelong.msg").attr("LifelongTaskState");
  py::object py_msg = RosMessageFromCpp(py_msg_cls, cpp_msg);
*/
template <typename CppMessage>
py::object RosMessageCppToPy(
    py::object py_message_cls, const CppMessage& cpp_message) {
  py::object py_deserialize = 
      py::module_::import("rclpy.serialization").attr("deserialize_message");
  rclcpp::SerializedMessage serialized =
      internal::MessageToSerialized(cpp_message);
  std::vector<uint8_t> bytes = internal::SerializedToBytes(serialized);
  return py_deserialize(bytes, py_message_cls);
}

/**
Converts C++ message to a Python message.

Example usage:

  py::object py_msg_cls =
      py::module_::import("anzu_lifelong.msg").attr("LifelongTaskState");
  auto msg = RosMessageToCpp(py_msg_cls, cpp_msg);
*/
template <typename CppMessage>
void RosMessagePyToCpp(py::object py_message, CppMesssage* cpp_message) {
  py::object py_serialize =
      py::module_::import("rclpy.serialization").attr("serialize_message");
  std::vector<uint8_t> bytes = serialize_message(py_message);
  rclcpp::SerializedMessage serialized = internal::BytesToSerialized(bytes);
  internal::SerializedToMessage(serialized, &cpp_message);
}

// TODO: Dunno exact spelling.
template <typename CppMessage>
using is_ros_message_v = std::something_something_t...;

/**
Given C++ type, returns Python message type.
*/
template <typename CppMessage>
py::object GetRosMessagePyCls() {
  static_assert(is_ros_message_v<CppMessage>, "Must be ROS message type.");
  // TODO: Dunno exact spelling of attributes.
  py::module_ m = py::module_::import(CppMessage::package_name);
  py::object cls = m.attr(CppMesssage::type_name);
  return cls;
}

namespace internal {

template <typename CppMessage>
struct message_type_caster {
  PYBIND11_TYPE_CASTER(CppMessage, py::detail::_(CppMessage::type_name));

  bool load(handle src, bool convert) {
    RosMessagePyToCpp(src, &value);
    return true;
  }

  static py::handle cast(
      const CppMessage& src,
      py::return_value_policy policy,
      py::handle parent) {
    py::object cls = GetRosMessagePyCls<CppMessage>();
    return RosMessageCppToPy(cls, src);
  }
};

}  // namespace internal
}  // namespace drake_ros_core_py
}  // namespace drake_ros_core

/**
Makes a single C++ message to be converted to/from Python using serilaizing and
deserializing.

@warning This may be very slow depending on size of your message. Use with
care.
@warning This prevents from accessing a C++ message by reference in Python;
instead, everything will be copied. See pybind11 type casting docs for more
details.
*/
#define DRAKE_ROS_PYBIND_MESSAGE_CASTER(CppMessage) \
    namespace pybind11 { namespace detail { \
    struct type_caster<CppMessage> : \
        public ::drake_ros::drake_ros_pybind::internal::message_type_caster< \
            CppMessage> \
    { }; \
    } }

/**
Makes all C++ messages be converted by serializing and deserializing.

@see DRAKE_ROS_PYBIND_MESSAGE_CASTER for warnings.
*/
#define DRAKE_ROS_PYBIND_MESSAGE_CASTER_ALL() \
    namespace pybind11 { namespace detail { \
    template < \
        typename T, \
        typename SFINEA = std::enable_if_t< \
            ::drake_ros::drake_ros_pybind::is_ros_message_v<T>> \
    struct type_caster : \
        public ::drake_ros::drake_ros_pybind::internal::message_type_caster<T> \
    { }; \
    } }
