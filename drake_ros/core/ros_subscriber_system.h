#pragma once

#include <memory>
#include <string>
#include <utility>

#include <drake/systems/framework/leaf_system.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>

#include "drake_ros/core/drake_ros.h"
#include "drake_ros/core/serializer.h"
#include "drake_ros/core/serializer_interface.h"

namespace drake_ros {
namespace core {
/** A system that can subscribe to ROS messages.
 It subscribes to a ROS topic and makes ROS messages available on
 its sole output port.
 */
class RosSubscriberSystem : public drake::systems::LeafSystem<double> {
 public:
  /** Instantiates a ROS subscriber system for a given ROS message type.
   See `RosSubscriberSystem::RosSubscriberSystem` documentation for
   further reference on function arguments.

   @tparam MessageT C++ ROS message type.
   */
  template <typename MessageT, typename... ArgsT>
  static std::unique_ptr<RosSubscriberSystem> Make(ArgsT&&... args) {
    // Assume C++ typesupport since this is a C++ template function
    return std::make_unique<RosSubscriberSystem>(
        std::make_unique<Serializer<MessageT>>(), std::forward<ArgsT>(args)...);
  }

  /** A constructor for the ROS subscriber system.
   It takes a `serializer` to deal with incoming messages.

   @param[in] serializer a (de)serialization interface for the
     expected ROS message type.
   @param[in] topic_name Name of the ROS topic to subscribe to.
   @param[in] qos QoS profile for the underlying ROS subscription.
   @param[in] ros interface to a live ROS node to publish from.
   */
  RosSubscriberSystem(std::unique_ptr<SerializerInterface> serializer,
                      const std::string& topic_name, const rclcpp::QoS& qos,
                      DrakeRos* ros);

  ~RosSubscriberSystem() override;

 protected:
  void DoCalcNextUpdateTime(const drake::systems::Context<double>&,
                            drake::systems::CompositeEventCollection<double>*,
                            double*) const override;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace core
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_core = drake_ros::core;
