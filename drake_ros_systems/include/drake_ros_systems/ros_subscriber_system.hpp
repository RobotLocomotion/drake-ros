#ifndef DRAKE_ROS_SYSTEMS__ROS_SUBSCRIBER_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_SUBSCRIBER_SYSTEM_HPP_

#include <memory>

#include <drake/systems/framework/leaf_system.h>

#include <drake_ros_systems/drake_ros_interface.hpp>

#include <rosidl_typesupport_cpp/message_type_support.hpp>

namespace drake_ros_systems
{
// PIMPL forward declaration
class RosSubscriberSystemPrivate;

/// System that subscribes to a ROS topic and makes it available on an output port
class RosSubscriberSystem : public drake::systems::LeafSystem<double>
{
public:

  /// Convenience method to make a subscriber system given a ROS message type
  template <typename MessageT>
  static
  std::unique_ptr<RosSubscriberSystem>
  Make(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface)
  {
    // Assume C++ typesupport since this is a C++ template function
    return std::make_unique<RosSubscriberSystem>(
        *rosidl_typesupport_cpp::get_message_type_support_handle<MessageT>(),
        [](){return std::make_unique<drake::Value<MessageT>>(MessageT());},
        topic_name, qos, ros_interface);
  }

  RosSubscriberSystem(
    const rosidl_message_type_support_t & ts,
    std::function<std::unique_ptr<drake::AbstractValue>(void)> create_default_value,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface);

  virtual ~RosSubscriberSystem();

protected:
  /// Override as a place to schedule event to move ROS message into a context
  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>&,
      drake::systems::CompositeEventCollection<double>*,
      double*) const override;

  std::unique_ptr<RosSubscriberSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
