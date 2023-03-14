#include "drake_ros/tf2/scene_tf_broadcaster_system.h"

#include <memory>
#include <unordered_set>

#include <drake/systems/framework/diagram_builder.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/qos.hpp>

#include "drake_ros/tf2/scene_tf_system.h"

namespace drake_ros {
namespace tf2 {

class SceneTfBroadcasterSystem::Impl {
 public:
  SceneTfSystem* scene_tf;
  drake::systems::InputPortIndex graph_query_port_index;
};

SceneTfBroadcasterSystem::SceneTfBroadcasterSystem(
    drake_ros::core::DrakeRos* ros, SceneTfBroadcasterParams params)
    : impl_(new Impl()) {
  drake::systems::DiagramBuilder<double> builder;

  impl_->scene_tf = builder.AddSystem<SceneTfSystem>();

  using drake_ros::core::RosPublisherSystem;
  auto scene_tf_publisher =
      builder.AddSystem(RosPublisherSystem::Make<tf2_msgs::msg::TFMessage>(
          params.tf_topic_name, tf2_ros::DynamicBroadcasterQoS(), ros,
          params.publish_triggers, params.publish_period));

  builder.Connect(impl_->scene_tf->get_scene_tf_output_port(),
                  scene_tf_publisher->get_input_port());

  impl_->graph_query_port_index = builder.ExportInput(
      impl_->scene_tf->get_graph_query_input_port(), "graph_query");

  builder.BuildInto(this);
}

SceneTfBroadcasterSystem::~SceneTfBroadcasterSystem() {}

void SceneTfBroadcasterSystem::RegisterMultibodyPlant(
    const drake::multibody::MultibodyPlant<double>* plant) {
  impl_->scene_tf->RegisterMultibodyPlant(plant);
}

void SceneTfBroadcasterSystem::ComputeFrameHierarchy() {
  impl_->scene_tf->ComputeFrameHierarchy();
}

const drake::systems::InputPort<double>&
SceneTfBroadcasterSystem::get_graph_query_input_port() const {
  return get_input_port(impl_->graph_query_port_index);
}

}  // namespace tf2
}  // namespace drake_ros
