#include "drake_ros/tf2/name_conventions.h"

#include <sstream>
#include <string>
#include <unordered_set>

#include "internal_name_conventions.h"  // NOLINT

namespace drake_ros {
namespace tf2 {

std::string GetTfFrameName(
    const drake::multibody::Body<double>& body,
    const drake::multibody::MultibodyPlant<double>* plant,
    const drake::geometry::FrameId& frame_id) {
  return internal::CalcTfFrameName(
      plant->GetModelInstanceName(body.model_instance()), body.name(),
      body.index(), frame_id.get_value());
}

std::string GetTfFrameName(
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const std::unordered_set<const drake::multibody::MultibodyPlant<double>*>&
        plants,
    const drake::geometry::FrameId& frame_id) {
  // Special case: world frame is always world
  if (frame_id == inspector.world_frame_id()) {
    return "world";
  }

  for (auto* plant : plants) {
    const drake::multibody::Body<double>* body =
        plant->GetBodyFromFrameId(frame_id);
    if (!body) {
      continue;
    }

    return internal::CalcTfFrameName(
        plant->GetModelInstanceName(body->model_instance()), body->name(),
        body->index(), frame_id.get_value());
  }

  return internal::CalcTfFrameName(inspector.GetName(frame_id),
                                   frame_id.get_value());
}

std::string GetTfFrameName(
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const std::unordered_set<const drake::multibody::MultibodyPlant<double>*>&
        plants,
    const drake::geometry::GeometryId& geometry_id) {
  return GetTfFrameName(inspector, plants, inspector.GetFrameId(geometry_id));
}

}  // namespace tf2
}  // namespace drake_ros
