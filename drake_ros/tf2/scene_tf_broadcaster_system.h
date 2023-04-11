#pragma once

#include <memory>
#include <string>
#include <unordered_set>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/leaf_system.h>
#include <drake_ros/core/drake_ros.h>

namespace drake_ros {
namespace tf2 {

/** Set of parameters that configure a SceneTfBroadcasterSystem. */
struct SceneTfBroadcasterParams {
  /** Publish triggers for tf broadcasting. */
  std::unordered_set<drake::systems::TriggerType> publish_triggers{
      drake::systems::TriggerType::kForced,
      drake::systems::TriggerType::kPerStep};

  /** Period for periodic tf broadcasting. */
  double publish_period{0.0};

  /** Topic name to be used by the broadcaster. */
  std::string tf_topic_name{"/tf"};
};

/** System for tf2 transform broadcasting.

 This system is a subdiagram aggregating a SceneTfSystem and a
 RosPublisherSystem to broadcast SceneGraph frame transforms.
 Messages are published to the `/tf` ROS topic.

 It exports one input port:
 - *graph_query* (abstract): expects a QueryObject from the SceneGraph.
*/
class SceneTfBroadcasterSystem : public drake::systems::Diagram<double> {
 public:
  /** A constructor for the scene tf2 broadcaster system.
   @param[in] ros interface to a live ROS node to publish from.
   @param[in] params optional broadcasting configuration.
   */
  explicit SceneTfBroadcasterSystem(drake_ros::core::DrakeRos* ros,
                                    SceneTfBroadcasterParams params = {});
  ~SceneTfBroadcasterSystem() override;

  /** Forwarded to SceneTfSystem::RegisterMultibodyPlant(). */
  void RegisterMultibodyPlant(
      const drake::multibody::MultibodyPlant<double>* plant);

  /** Forwarded to SceneTfSystem::ComputeFrameHierarchy(). */
  void ComputeFrameHierarchy();

  const drake::systems::InputPort<double>& get_graph_query_input_port() const;

 private:
  class Impl;

  std::unique_ptr<Impl> impl_;
};
}  // namespace tf2
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_tf2 = drake_ros::tf2;
