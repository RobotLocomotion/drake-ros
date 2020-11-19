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

  std::unique_ptr<SerializerInterface> serializer_;
  // A handle to a subscription
  // TODO(sloretz) unique_ptr that unsubscribes in destructor
  std::shared_ptr<Subscription> sub_;
  // Mutex to prevent multiple threads from modifying this class
  std::mutex mutex_;
  // The last received message that has not yet been put into a context.
  std::shared_ptr<rclcpp::SerializedMessage> msg_;
};

RosSubscriberSystem::RosSubscriberSystem(
  std::unique_ptr<SerializerInterface> & serializer,
  const std::string & topic_name,
  const rclcpp::QoS & qos,
  std::shared_ptr<DrakeRosInterface> ros)
: impl_(new RosSubscriberSystemPrivate())
{
  impl_->serializer_ = std::move(serializer);
  impl_->sub_ = ros->create_subscription(*impl_->serializer_->get_type_support(), topic_name, qos,
    std::bind(&RosSubscriberSystemPrivate::handle_message, impl_.get(), std::placeholders::_1));

  DeclareAbstractOutputPort(
      [serializer{impl_->serializer_.get()}](){return serializer->create_default_value();},
      [](const drake::systems::Context<double> & context, drake::AbstractValue * output_value) {
        // Transfer message from state to output port
        output_value->SetFrom(context.get_abstract_state().get_value(kStateIndexMessage));
      });

  static_assert(kStateIndexMessage == 0, "");
  DeclareAbstractState(impl_->serializer_->create_default_value());
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
      callback = [this, serialized_message{std::move(message)}](
          const drake::systems::Context<double>&,
          const drake::systems::UnrestrictedUpdateEvent<double>&,
          drake::systems::State<double>* state)
      {
        // Deserialize the message and store it in the abstract state on the context
        drake::systems::AbstractValues & abstract_state = state->get_mutable_abstract_state();
        auto & abstract_value = abstract_state.get_mutable_value(kStateIndexMessage);
        const bool ret = impl_->serializer_->deserialize(*serialized_message, abstract_value);
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
