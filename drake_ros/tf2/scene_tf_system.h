#pragma once

#include <memory>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>
#include <tf2_msgs/msg/tf_message.hpp>

namespace drake_ros {
namespace tf2 {
/** System for SceneGraph frame transforms aggregation as a ROS tf2 message.

 This system outputs a `tf2_msgs/msg/TFMessage` populated with the
 rigid transforms from the world frame to each other frame in the scene,
 using Context time to timestamp each `geometry_msgs/msg/TransformStamped`
 message.

 It has one input port:
 - *graph_query* (abstract): expects a QueryObject from the SceneGraph.

 It has one output port:
 - *scene_tf* (abstract): rigid transforms w.r.t. the world frame for all
   frames in the scene, as a tf2_msgs::msg::TFMessage message.
*/
class SceneTfSystem : public drake::systems::LeafSystem<double> {
 public:
  SceneTfSystem();
  virtual ~SceneTfSystem();

  /** Register a MultibodyPlant present in the scene.

   This provides the system with additional information
   to generate semantically meaningful frame string IDs.

   @param[in] plant multibody plant instance to be registered. Registered
   multibody plants must outlive this SceneTfSystem instance since it will store
   the pointer for the remainder of its life.
   @pre `plant` is associated with the same SceneGraph
   whose query output port this system connects to.
  */
  void RegisterMultibodyPlant(
      const drake::multibody::MultibodyPlant<double>* plant);

  /** Compute the frame hierarchy of all the registered Multibody Plants.

   Call this after you have registered all your finalised Multibody Plants.
   It will calculate the frame hierarchy of the plants for use in providing
   a TF tree that accurately represents the hierarchy of frames in your System.
  */
  void ComputeFrameHierarchy();

  const drake::systems::InputPort<double>& get_graph_query_input_port() const;

  const drake::systems::OutputPort<double>& get_scene_tf_output_port() const;

 private:
  void CalcSceneTf(const drake::systems::Context<double>& context,
                   tf2_msgs::msg::TFMessage* output_value) const;

  // PIMPL forward declaration
  class Impl;

  std::unique_ptr<Impl> impl_;
};
}  // namespace tf2
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_tf2 = drake_ros::tf2;
