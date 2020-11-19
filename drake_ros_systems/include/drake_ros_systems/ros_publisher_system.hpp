#ifndef DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_PUBLISHER_SYSTEM_HPP_

#include <memory>

#include <drake/systems/framework/leaf_system.h>
#include <rmw/rmw.h>

#include <drake_ros_systems/drake_ros_interface.hpp>
#include <drake_ros_systems/serializer.hpp>
#include <drake_ros_systems/serializer_interface.hpp>

#include <rclcpp/serialized_message.hpp>

namespace drake_ros_systems
{
/// PIMPL forward declaration
class RosPublisherSystemPrivate;

/// Accepts ROS messages on an input port and publishes them to a ROS topic
class RosPublisherSystem: public drake::systems::LeafSystem<double>
{
public:
  /// Convenience method to make a publisher system given a ROS message type
  template <typename MessageT>
  static
  std::unique_ptr<RosPublisherSystem>
  Make(
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface)
  {
    // Assume C++ typesupport since this is a C++ template function
    std::unique_ptr<SerializerInterface> serializer = std::make_unique<Serializer<MessageT>>();
    return std::make_unique<RosPublisherSystem>(serializer, topic_name, qos, ros_interface);
  }

  RosPublisherSystem(
    std::unique_ptr<SerializerInterface> & serializer,
    const std::string & topic_name,
    const rclcpp::QoS & qos,
    std::shared_ptr<DrakeRosInterface> ros_interface);

  virtual ~RosPublisherSystem();

  /// Convenience method to publish a C++ ROS message
  template <typename MessageT>
  void
  publish(const MessageT & message)
  {
    static const Serializer<MessageT> serializer;
    publish(serializer->serialize(message));
  }

  /// Publish a serialized ROS message
  void
  publish(const rclcpp::SerializedMessage & serialized_msg);

protected:

  void
  publish_input(const drake::systems::Context<double> & context);

  std::unique_ptr<RosPublisherSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
