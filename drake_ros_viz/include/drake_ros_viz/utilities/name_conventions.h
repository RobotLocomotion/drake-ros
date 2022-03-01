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
#pragma once

#include <string>
#include <unordered_set>

#include <drake/geometry/scene_graph_inspector.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace drake_ros_viz {

/** Retrieve conventional marker namespace for a given scene geometry.
 @param[in] inspector inspector for a given SceneGraph's data.
 @param[in] plants a set of MultibodyPlant instances from which to derive
   semantically meaningful marker namespaces, if possible.
 @param[in] geometry_id target geometry ID.
 @returns the namespace for the marker
 @pre `geomtry_id` is associated with a registered geometry in the SceneGraph
   implied by `inspector`.
 @pre all `plants` are associated with the SceneGraph implied by `inspector`.
 */
std::string GetMarkerNamespace(
    const drake::geometry::SceneGraphInspector<double>& inspector,
    const std::unordered_set<const drake::multibody::MultibodyPlant<double>*>&
        plants,
    const drake::geometry::GeometryId& geometry_id);

}  // namespace drake_ros_viz
