#pragma once

#include <string>
#include <unordered_set>

#include <drake/geometry/scene_graph_inspector.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace drake_ros {
namespace tf2 {

/** Retrieve conventional tf frame name for a given scene frame.

 @param[in] inspector inspector for a given SceneGraph's data.
 @param[in] plants a set of MultibodyPlant instances from which to derive
   semantically meaningful tf frame names, if possible.
 @param[in] frame_id target frame ID.
 @returns fully qualified tf frame name
 @pre `frame_id` is associated with a registered frame in the SceneGraph
   implied by `inspector`.
 @pre all `plants` are associated with the SceneGraph implied by `inspector`.
 */
std::string GetTfFrameName(
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const std::unordered_set<const drake::multibody::MultibodyPlant<double>*>&
        plants,
    const drake::geometry::FrameId& frame_id);

/** Retrieve conventional tf frame name for the frame
 affixed to a given scene geometry.

 @param[in] inspector inspector for a given SceneGraph's data.
 @param[in] plants a set of MultibodyPlant instances from which to derive
   semantically meaningful tf frame names, if possible.
 @param[in] geometry_id target geometry ID.
 @returns fully qualified tf frame name
 @pre `geometry_id` is associated with a registered geometry in the SceneGraph
   implied by `inspector`.
 @pre all `plants` are associated with the SceneGraph implied by `inspector`.
 */
std::string GetTfFrameName(
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const std::unordered_set<const drake::multibody::MultibodyPlant<double>*>&
        plants,
    const drake::geometry::GeometryId& geometry_id);

std::string GetTfFrameName(
    const drake::multibody::Body<double>& body,
    const drake::multibody::MultibodyPlant<double>* plant,
    const drake::geometry::FrameId& frame_id);
}  // namespace tf2
}  // namespace drake_ros

// Legacy spelling for backwards compatibility.
namespace drake_ros_tf2 = drake_ros::tf2;
