#ifndef DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_
#define DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_

namespace drake_ros_systems
{
class SerializerInterface
{
public:
  virtual
  rclcpp::SerializedMessage
  serialize(const drake::AbstractValue & abstract_value) const = 0;

  virtual
  void
  deserialize(
    const rclcpp::SerializedMessage & message,
    drake::AbstractValue & abstract_value) const = 0;

  virtual
  std::unique_ptr<drake::AbstractValue>
  create_default_value() const = 0;
};
}  // namespace drake_ros_systems
#endif  // DRAKE_ROS_SYSTEMS__SERIALIZER_INTERFACE_HPP_
