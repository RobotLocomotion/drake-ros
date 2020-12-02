#ifndef PY_SERIALIZER_HPP_
#define PY_SERIALIZER_HPP_

#include <pybind11/eval.h>
#include <pybind11/pybind11.h>
#include <rmw/rmw.h>

#include <memory>

#include <drake_ros_systems/serializer_interface.hpp>

namespace py = pybind11;

namespace drake_ros_systems
{
// Serialize/Deserialize Python ROS types to rclcpp::SerializedMessage
class PySerializer : public SerializerInterface
{
public:
  PySerializer(py::object message_type)
    : message_type_(message_type)
  {
    // Check if message_type.__class__._TYPE_SUPPORT is None,
    py::object metaclass = message_type.attr("__class__");
    if (metaclass.attr("_TYPE_SUPPORT").is_none()) {
      // call message_type.__class__.__import_type_support__()
      metaclass.attr("__import_type_support__")();
    }
    // Get type support capsule and pointer
    auto typesupport = py::cast<py::capsule>(metaclass.attr("_TYPE_SUPPORT"));
    // TODO(sloretz) use get_pointer() in py 2.6+
    type_support_ = static_cast<decltype(type_support_)>(typesupport);

    auto convert_from_py =
      py::cast<py::capsule>(metaclass.attr("_CONVERT_FROM_PY"));
    // TODO(sloretz) use get_pointer() in py 2.6+
    convert_from_py_ = reinterpret_cast<decltype(convert_from_py_)>(
      static_cast<void *>(convert_from_py));

    auto convert_to_py =
      py::cast<py::capsule>(metaclass.attr("_CONVERT_TO_PY"));
    // TODO(sloretz) use get_pointer() in py 2.6+
    convert_to_py_ = reinterpret_cast<decltype(convert_to_py_)>(
      static_cast<void *>(convert_to_py));

    auto create_ros_message =
      py::cast<py::capsule>(metaclass.attr("_CREATE_ROS_MESSAGE"));
    // TODO(sloretz) use get_pointer() in py 2.6+
    create_ros_message_ = reinterpret_cast<decltype(create_ros_message_)>(
      static_cast<void *>(create_ros_message));

    auto destroy_ros_message =
      py::cast<py::capsule>(metaclass.attr("_DESTROY_ROS_MESSAGE"));
    // TODO(sloretz) use get_pointer() in py 2.6+
    destroy_ros_message_ = reinterpret_cast<decltype(destroy_ros_message_)>(
      static_cast<void *>(destroy_ros_message));
  }

  rclcpp::SerializedMessage
  serialize(const drake::AbstractValue & abstract_value) const override
  {
    // convert from inaccessible drake::pydrake::Object type
    py::dict scope;
    scope["av"] = &abstract_value;
    py::object message = py::eval("av.Clone().get_mutable_value()", scope);

    // Create  C ROS message
    auto c_ros_message = std::unique_ptr<void, decltype(destroy_ros_message_)>(
      create_ros_message_(), destroy_ros_message_);

    // Convert the Python message to a C ROS message
    if (!convert_from_py_(message.ptr(), c_ros_message.get())) {
      // TODO(sloretz) Error handling? Throw?
      return rclcpp::SerializedMessage();
    }

    // Serialize the C message
    rclcpp::SerializedMessage serialized_msg;
    const auto ret = rmw_serialize(
      c_ros_message.get(),
      type_support_,
      &serialized_msg.get_rcl_serialized_message());
    if (ret != RMW_RET_OK) {
      // TODO(sloretz) do something if serialization fails
      return rclcpp::SerializedMessage();
    }
    return serialized_msg;
  }

  bool
  deserialize(
    const rclcpp::SerializedMessage & serialized_message,
    drake::AbstractValue & abstract_value) const override
  {
    // TODO(sloretz) it would be so much more convenient if I didn't have to
    // care that the Python typesupport used the C type support internally.
    // Why isn't this encapsulated in the python type support itself?

    // Create a C type support version of the message
    auto c_ros_message = std::unique_ptr<void, decltype(destroy_ros_message_)>(
      create_ros_message_(), destroy_ros_message_);
    if (nullptr == c_ros_message.get()) {
      return false;
    }

    // Deserialize to C message type
    const auto ret = rmw_deserialize(
      &serialized_message.get_rcl_serialized_message(),
      type_support_,
      c_ros_message.get());

    if (RMW_RET_OK != ret) {
      return false;
    }

    // Convert C type to Python type
    PyObject * pymessage = convert_to_py_(c_ros_message.get());
    if (!pymessage) {
      return false;
    }

    // Store the Python message in the AbstractValue
    // convert to inaccessible drake::pydrake::Object type
    py::dict scope;
    scope["av"] = &abstract_value;
    scope["message"] = pymessage;
    py::exec("av.set_value(message)", scope);

    return true;
  }

  std::unique_ptr<drake::AbstractValue>
  create_default_value() const override
  {
    // convert to inaccessible drake::pydrake::Object type
    py::object scope = py::module::import("pydrake.common.value").attr("__dict__");
    py::object lambda = py::eval("lambda msg: AbstractValue.Make(msg)", scope);
    py::object result = lambda(message_type_());

    return py::cast<std::unique_ptr<drake::AbstractValue>>(result);
  }

  const rosidl_message_type_support_t *
  get_type_support() const override
  {
    return type_support_;
  }

private:
  py::object message_type_;
  rosidl_message_type_support_t * type_support_;

  bool (*convert_from_py_)(PyObject *, void *);
  PyObject * (*convert_to_py_)(void *);
  void * (*create_ros_message_)(void);
  void (*destroy_ros_message_)(void *);
};
}  // namespace drake_ros_systems
#endif  // PY_SERIALIZER_HPP_
