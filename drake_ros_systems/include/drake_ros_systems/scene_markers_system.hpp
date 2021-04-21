// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#ifndef DRAKE_ROS_SYSTEMS__SCENE_MARKERS_SYSTEM_HPP_
#define DRAKE_ROS_SYSTEMS__SCENE_MARKERS_SYSTEM_HPP_

#include <memory>

#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/rgba.h>
#include <drake/systems/framework/leaf_system.h>

#include <visualization_msgs/msg/marker_array.hpp>


namespace drake_ros_systems
{

class SceneMarkersSystem : public drake::systems::LeafSystem<double>
{
public:
  SceneMarkersSystem(
    const drake::geometry::Role & role = drake::geometry::Role::kIllustration,
    const drake::geometry::Rgba & default_color = {0.9, 0.9, 0.9, 1.0});
  virtual ~SceneMarkersSystem();

private:
  void
  PopulateSceneMarkersMessage(
    const drake::systems::Context<double> & context,
    visualization_msgs::msg::MarkerArray * output_value) const;

  const visualization_msgs::msg::MarkerArray &
  EvalSceneMarkers(const drake::systems::Context<double> & context) const;

  void
  CalcSceneMarkers(
    const drake::systems::Context<double> & context,
    visualization_msgs::msg::MarkerArray * output_value) const;

  // PIMPL forward declaration
  class SceneMarkersSystemPrivate;

  std::unique_ptr<SceneMarkersSystemPrivate> impl_;
};

}  // namespace drake_ros_systems

#endif  // DRAKE_ROS_SYSTEMS__SCENE_MARKERS_SYSTEM_HPP_
