#include <drake_ros_systems/ros_interface_system.hpp>

namespace drake_ros_systems
{
class RosInterfaceSystemPrivate
{
public:
  std::shared_ptr<DrakeRosInterface> ros_;
};


RosInterfaceSystem::RosInterfaceSystem(std::unique_ptr<DrakeRosInterface> ros)
: impl_(new RosInterfaceSystemPrivate())
{
  impl_->ros_ = std::move(ros);
}

RosInterfaceSystem::~RosInterfaceSystem()
{
}

/// Return a handle for interacting with ROS
std::shared_ptr<DrakeRosInterface>
RosInterfaceSystem::get_ros_interface() const
{
  return impl_->ros_;
}

/// Override as a place to call rclcpp::spin()
void
RosInterfaceSystem::DoCalcNextUpdateTime(
  const drake::systems::Context<double> &,
  drake::systems::CompositeEventCollection<double> *,
  double * time) const
{
  // Do work for at most 1ms so system doesn't get blocked if there's more work than it can handle
  const int max_work_time_millis = 1;
  impl_->ros_->spin(max_work_time_millis);
  // TODO(sloretz) Lcm system pauses time if some work was done, but ROS 2 API doesn't say if
  // any work was done. How to reconcile that?
  *time = std::numeric_limits<double>::infinity();
}
}  // namespace drake_ros_systems
