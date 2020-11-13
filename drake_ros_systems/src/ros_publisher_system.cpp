#include <drake_ros_systems/ros_publisher_system.hpp>
#include <drake_ros_systems/serializer_interface.hpp>

#include "publisher.hpp"

namespace drake_ros_systems
{
class RosPublisherSystemPrivate
{
public:
  const rosidl_message_type_support_t * type_support_;
  std::unique_ptr<SerializerInterface> serializer_;
  std::unique_ptr<Publisher> pub_;
};

RosPublisherSystem::RosPublisherSystem(
  const rosidl_message_type_support_t & ts,
  std::unique_ptr<SerializerInterface> & serializer,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::shared_ptr<DrakeRosInterface> ros)
: impl_(new RosPublisherSystemPrivate())
{
  impl_->type_support_ = &ts;
  impl_->serializer_ = std::move(serializer);
  impl_->pub_ = ros->create_publisher(ts, topic_name, qos);

  DeclareAbstractInputPort("message", *(impl_->serializer_->create_default_value()));

  // TODO(sloretz) customizable triggers like lcm system
  DeclarePerStepEvent(
    drake::systems::PublishEvent<double>([this](
        const drake::systems::Context<double>& context,
        const drake::systems::PublishEvent<double>&) {
      publish_input(context);
    }));
}

RosPublisherSystem::~RosPublisherSystem()
{
}

void
RosPublisherSystem::publish(const rclcpp::SerializedMessage & serialized_msg)
{
  impl_->pub_->publish(serialized_msg);
}

void
RosPublisherSystem::publish_input(const drake::systems::Context<double> & context)
{
  const drake::AbstractValue & input = get_input_port().Eval<drake::AbstractValue>(context);
  impl_->pub_->publish(impl_->serializer_->serialize(input));
}
}  // namespace drake_ros_systems
