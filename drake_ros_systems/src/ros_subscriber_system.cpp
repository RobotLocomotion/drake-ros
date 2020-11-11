#include <mutex>

#include <drake/systems/framework/abstract_values.h>

#include <drake_ros_systems/ros_subscriber_system.hpp>

#include "subscription.hpp"

namespace drake_ros_systems
{
// Index in AbstractState that deserialized message is stored
const int kStateIndexMessage = 0;

class RosSubscriberSystemPrivate
{
public:
  void
  handle_message(std::shared_ptr<rclcpp::SerializedMessage> callback)
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    // TODO(sloretz) Queue messages here? Lcm subscriber doesn't, so maybe lost messages are ok
    // Overwrite last message
    msg_ = callback;
  }

  std::shared_ptr<rclcpp::SerializedMessage>
  take_message()
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    return std::move(msg_);
  }

  const rosidl_message_type_support_t * type_support_;
  // A handle to a subscription - we're subscribed as long as this is alive
  std::unique_ptr<Subscription> sub_;
  // Mutex to prevent multiple threads from modifying this class
  std::mutex mutex_;
  // The last received message that has not yet been put into a context.
  std::shared_ptr<rclcpp::SerializedMessage> msg_;
};

RosSubscriberSystem::RosSubscriberSystem(
  const rosidl_message_type_support_t & ts,
  std::function<std::unique_ptr<drake::AbstractValue>(void)> create_default_value,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::shared_ptr<DrakeRosInterface> ros)
: impl_(new RosSubscriberSystemPrivate())
{
  impl_->type_support_ = &ts;
  impl_->sub_ = ros->create_subscription(ts, topic_name, qos,
    std::bind(&RosSubscriberSystemPrivate::handle_message, impl_.get(), std::placeholders::_1));

  DeclareAbstractOutputPort(
      create_default_value,
      [](const drake::systems::Context<double> & context, drake::AbstractValue * output_value) {
        // Transfer message from state to output port
        output_value->SetFrom(context.get_abstract_state().get_value(kStateIndexMessage));
      });

  static_assert(kStateIndexMessage == 0, "");
  DeclareAbstractState(create_default_value());
}

RosSubscriberSystem::~RosSubscriberSystem()
{
}

void
RosSubscriberSystem::DoCalcNextUpdateTime(
  const drake::systems::Context<double> & context,
  drake::systems::CompositeEventCollection<double> * events,
  double * time) const
{
  // Vvv Copied from LcmSubscriberSystem vvv

  // We do not support events other than our own message timing events.
  LeafSystem<double>::DoCalcNextUpdateTime(context, events, time);
  DRAKE_THROW_UNLESS(events->HasEvents() == false);
  DRAKE_THROW_UNLESS(std::isinf(*time));

  std::shared_ptr<rclcpp::SerializedMessage> message = impl_->take_message();

  // Do nothing unless we have a new message.
  if (nullptr == message.get()) {
    return;
  }

  // Create a unrestricted event and tie the handler to the corresponding
  // function.
  drake::systems::UnrestrictedUpdateEvent<double>::UnrestrictedUpdateCallback
      callback = [this, serialized_message{std::move(message)}, ts{impl_->type_support_}](
          const drake::systems::Context<double>&,
          const drake::systems::UnrestrictedUpdateEvent<double>&,
          drake::systems::State<double>* state)
      {
        // Deserialize the message and store it in the abstract state on the context
        drake::systems::AbstractValues & abstract_state = state->get_mutable_abstract_state();
        const auto ret = rmw_deserialize(
          &serialized_message->get_rcl_serialized_message(), ts,
          &abstract_state.get_mutable_value(kStateIndexMessage));
        if (ret != RMW_RET_OK) {
          return drake::systems::EventStatus::Failed(this, "Failed to deserialize ROS message");
        }
        return drake::systems::EventStatus::Succeeded();
      };

  // Schedule an update event at the current time.
  *time = context.get_time();
  drake::systems::EventCollection<drake::systems::UnrestrictedUpdateEvent<double>> & uu_events =
      events->get_mutable_unrestricted_update_events();
  uu_events.add_event(
      std::make_unique<drake::systems::UnrestrictedUpdateEvent<double>>(
          drake::systems::TriggerType::kTimed, callback));
}
}  // namespace drake_ros_systems
