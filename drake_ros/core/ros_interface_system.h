#pragma once

#include <memory>

#include <drake/systems/framework/leaf_system.h>

#include "drake_ros/core/drake_ros.h"

namespace drake_ros {
namespace core {
/** A system that manages a Drake ROS interface. */
class RosInterfaceSystem : public drake::systems::LeafSystem<double> {
 public:
  /** A constructor that takes ownership of the `ros` interface. */
  explicit RosInterfaceSystem(std::unique_ptr<DrakeRos> ros);
  /** A constructor that takes a shared pointer to a ROS node. */
  explicit RosInterfaceSystem(rclcpp::Node::SharedPtr ros_node);

  ~RosInterfaceSystem() override;

  /** Returns a mutable reference to the underlying ROS interface. */
  DrakeRos* get_ros_interface() const;

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
