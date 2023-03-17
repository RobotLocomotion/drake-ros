#include "drake_ros/viz/rviz_visualizer.h"

#include <memory>
#include <unordered_set>
#include <utility>

#include <drake/systems/framework/diagram_builder.h>
#include <drake_ros/core/drake_ros.h>
#include <drake_ros/core/ros_publisher_system.h>
#include <drake_ros/tf2/scene_tf_broadcaster_system.h>
#include <rclcpp/qos.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "drake_ros/viz/scene_markers_system.h"

namespace drake_ros {
namespace viz {

class RvizVisualizer::RvizVisualizerPrivate {
 public:
  SceneMarkersSystem* scene_visual_markers;
  SceneMarkersSystem* scene_collision_markers;
  drake_ros::tf2::SceneTfBroadcasterSystem* scene_tf_broadcaster{nullptr};
};

RvizVisualizer::RvizVisualizer(drake_ros::core::DrakeRos* ros,
                               RvizVisualizerParams params)
    : impl_(new RvizVisualizerPrivate()) {
  drake::systems::DiagramBuilder<double> builder;

  using drake_ros::core::RosPublisherSystem;
  auto scene_visual_markers_publisher = builder.AddSystem(
      RosPublisherSystem::Make<visualization_msgs::msg::MarkerArray>(
          "/scene_markers/visual", rclcpp::QoS(1), ros, params.publish_triggers,
          params.publish_period));

  impl_->scene_visual_markers =
      builder.AddSystem<SceneMarkersSystem>(SceneMarkersParams::Illustration());

  builder.Connect(impl_->scene_visual_markers->get_markers_output_port(),
                  scene_visual_markers_publisher->get_input_port());

  builder.ExportInput(impl_->scene_visual_markers->get_graph_query_input_port(),
                      "graph_query");

  auto scene_collision_markers_publisher = builder.AddSystem(
      RosPublisherSystem::Make<visualization_msgs::msg::MarkerArray>(
          "/scene_markers/collision", rclcpp::QoS(1), ros,
          params.publish_triggers, params.publish_period));

  impl_->scene_collision_markers =
      builder.AddSystem<SceneMarkersSystem>(SceneMarkersParams::Proximity());

  builder.Connect(impl_->scene_collision_markers->get_markers_output_port(),
                  scene_collision_markers_publisher->get_input_port());

  builder.ConnectInput(
      "graph_query",
      impl_->scene_collision_markers->get_graph_query_input_port());

  if (params.publish_tf) {
    impl_->scene_tf_broadcaster =
        builder.AddSystem<drake_ros::tf2::SceneTfBroadcasterSystem>(
            ros, drake_ros::tf2::SceneTfBroadcasterParams{
                     params.publish_triggers, params.publish_period});

    builder.ConnectInput(
        "graph_query",
        impl_->scene_tf_broadcaster->get_graph_query_input_port());
  }

  builder.BuildInto(this);
}

RvizVisualizer::~RvizVisualizer() {}

void RvizVisualizer::RegisterMultibodyPlant(
    const drake::multibody::MultibodyPlant<double>* plant) {
  impl_->scene_visual_markers->RegisterMultibodyPlant(plant);
  impl_->scene_collision_markers->RegisterMultibodyPlant(plant);
  if (impl_->scene_tf_broadcaster) {
    impl_->scene_tf_broadcaster->RegisterMultibodyPlant(plant);
  }
}

void RvizVisualizer::ComputeFrameHierarchy() {
  if (impl_->scene_tf_broadcaster) {
    impl_->scene_tf_broadcaster->ComputeFrameHierarchy();
  }
}

const drake::systems::InputPort<double>&
RvizVisualizer::get_graph_query_input_port() const {
  return get_input_port();
}

}  // namespace viz
}  // namespace drake_ros
