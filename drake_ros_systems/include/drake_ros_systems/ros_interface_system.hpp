#ifndef DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_

#include <memory>

#include <drake/systems/framework/leaf_system.h>

#include <drake_ros_systems/drake_ros_interface.hpp>

namespace drake_ros_systems
{
// PIMPL forward declaration
class RosInterfaceSystemPrivate;

/// System that takes care of calling spin() in Drake's systems framework
class RosInterfaceSystem : public drake::systems::LeafSystem<double>
{
public:
  RosInterfaceSystem(std::unique_ptr<DrakeRosInterface> ros);
  virtual ~RosInterfaceSystem();

  /// Return a handle for interacting with ROS
  std::shared_ptr<DrakeRosInterface>
  get_ros_interface() const;

protected:
  /// Override as a place to call rclcpp::spin()
  void DoCalcNextUpdateTime(
      const drake::systems::Context<double>&,
      drake::systems::CompositeEventCollection<double>*,
      double*) const override;

  std::unique_ptr<RosInterfaceSystemPrivate> impl_;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__ROS_INTERFACE_SYSTEM_HPP_
