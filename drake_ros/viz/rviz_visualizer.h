#pragma once

#include <memory>
#include <unordered_set>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/diagram.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/viz/defaults.h>

namespace drake_ros {
namespace viz {

/// Set of parameters that configure an RvizVisualizer.
struct RvizVisualizerParams {
  /// Publish triggers for scene markers and tf broadcasting.
  std::unordered_set<drake::systems::TriggerType> publish_triggers{
      kDefaultPublishTriggers};

  /// Period for periodic scene markers and tf broadcasting.
  double publish_period{kDefaultPublishPeriod};

  /// Whether to perform tf broadcasting or not.
  bool publish_tf{true};
};

/// System for SceneGraph visualization in RViz.
///
/// This system is a subdiagram aggregating a SceneMarkersSystem, a
/// RosPublisherSystem, and optionally a TfBroadcasterSystem to enable
/// SceneGraph visualization in RViz. All scene geometries are published
/// as a visualization_msgs/msg/MarkerArray message to a `/scene_markers`
/// ROS topic. If `publish_tf` is `true`, all SceneGraph frames are
/// broadcasted as tf2 transforms.
///
/// It exports one input port:
/// - *graph_query* (abstract): expects a QueryObject from the SceneGraph.
class RvizVisualizer : public drake::systems::Diagram<double> {
 public:
  /** A constructor for the RViz vizualizer system.
    @param[in] ros interface to a live ROS node to publish from.
    @param[in] params optional rviz visualizer configurations.
    */
  explicit RvizVisualizer(drake_ros::core::DrakeRos* ros,
                          RvizVisualizerParams params = {});

  ~RvizVisualizer() override;

  // Forwarded to SceneTfSystem::RegisterMultibodyPlant
  // and SceneTfBroadcasterSystem::RegisterMultibodyPlant
  void RegisterMultibodyPlant(
      const drake::multibody::MultibodyPlant<double>* plant);

  void ComputeFrameHierarchy();

  const drake::systems::InputPort<double>& get_graph_query_input_port() const;

 private:
  class RvizVisualizerPrivate;

  std::unique_ptr<RvizVisualizerPrivate> impl_;
};

}  // namespace viz
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_viz = drake_ros::viz;
