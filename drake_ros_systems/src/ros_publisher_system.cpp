#include <drake_ros_systems/ros_publisher_system.hpp>

#include "publisher.hpp"

namespace drake_ros_systems
{
class RosPublisherSystemPrivate
{
public:
  const rosidl_message_type_support_t * type_support_;
  std::function<void(const drake::AbstractValue &, rclcpp::SerializedMessage &)> serialize_abstract_value_;
  std::unique_ptr<Publisher> pub_;
};

RosPublisherSystem::RosPublisherSystem(
  const rosidl_message_type_support_t & ts,
  std::function<std::unique_ptr<drake::AbstractValue>(void)> create_default_value,
  std::function<void(const drake::AbstractValue &, rclcpp::SerializedMessage &)> serialize_abstract_value,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::shared_ptr<DrakeRosInterface> ros)
: impl_(new RosPublisherSystemPrivate())
{
  impl_->type_support_ = &ts;
  impl_->serialize_abstract_value_ = serialize_abstract_value;
  impl_->pub_ = ros->create_publisher(ts, topic_name, qos);

  DeclareAbstractInputPort("message", *create_default_value());

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
  // Converts the input into LCM message bytes.
  const drake::AbstractValue & input = get_input_port().Eval<drake::AbstractValue>(context);
  rclcpp::SerializedMessage serialized_msg;
  impl_->serialize_abstract_value_(input, serialized_msg);

  impl_->pub_->publish(serialized_msg);
}
}  // namespace drake_ros_systems
