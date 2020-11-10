#ifndef DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_

#include <memory>

#include <drake/systems/framework/leaf_system.h>
#include <rmw/rmw.h>

#include <drake_ros_systems/drake_ros_interface.hpp>

#include <rclcpp/serialized_message.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace drake_ros_systems
{
/// PIMPL forward declaration
class RosPublisherSystemPrivate;

/// System that subscribes to a ROS topic and makes it available on an output port
class RosPublisherSystem: public drake::systems::LeafSystem<double>
{
public:

  /// Convenience method to make a subscriber system given a ROS message type
  template <typename MessageT>
  static
  std::unique_ptr<RosPublisherSystem>
  Make(std::shared_ptr<DrakeRosInterface> ros_interface)
  {
    // Assume C++ typesupport since this is a C++ template function
    return std::make_unique<RosPublisherSystem>(
        rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        ros_interface);
  }

  RosPublisherSystem(
    const rosidl_message_type_support_t & ts,
    std::shared_ptr<DrakeRosInterface> ros_interface);

  virtual ~RosPublisherSystem();

  /// Publish a ROS message
  template <typename MessageT>
  void
  publish(const MessageT & message)
  {
    rclcpp::SerializedMessage serialized_msg;
    const auto ret = rmw_serialize(
      &message,
      rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
      &serialized_msg.get_rcl_serialized_message());
    // TODO(sloretz) throw if failed to serialize
    publish(serialized_msg);
  }

  /// Publish a serialized ROS message
  void
  publish(const rclcpp::SerializedMessage & serialized_msg);

protected:
  /// Override as a place to schedule event to move ROS message into a context
  // void DoCalcNextUpdateTime(
  //     const Context<double>&,
  //     systems::CompositeEventCollection<double>*,
  //     double*) const override;

  std::unique_ptr<RosPublisherSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
