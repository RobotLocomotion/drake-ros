#include "drake_ros/core/ros_interface_system.h"

#include <limits>
#include <memory>
#include <utility>

namespace drake_ros {
namespace core {
struct RosInterfaceSystem::Impl {
  // Interface to ROS (through a node).
  std::unique_ptr<DrakeRos> ros;
};

RosInterfaceSystem::RosInterfaceSystem(std::unique_ptr<DrakeRos> ros)
    : impl_(new Impl()) {
  impl_->ros = std::move(ros);
}

RosInterfaceSystem::~RosInterfaceSystem() {}

DrakeRos* RosInterfaceSystem::get_ros_interface() const {
  return impl_->ros.get();
}

void RosInterfaceSystem::DoCalcNextUpdateTime(
    const drake::systems::Context<double>&,
    drake::systems::CompositeEventCollection<double>*, double* time) const {
  constexpr int kMaxWorkMillis = 0;  // Do not block.
  impl_->ros->Spin(kMaxWorkMillis);
  // TODO(sloretz) Lcm system pauses time if some work was done, but ROS 2 API
  // doesn't say if any work was done. How to reconcile that?
  // TODO(hidmic): test for subscription latency in context time, how does the
  // order of node spinning and message taking affects it?
  *time = std::numeric_limits<double>::infinity();
}
}  // namespace core
}  // namespace drake_ros
