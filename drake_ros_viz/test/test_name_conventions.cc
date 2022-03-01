// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "utilities/internal_name_conventions.h"
#include <gtest/gtest.h>

TEST(NameConventions, CalcMarkerNamespaceWithBody) {
  // All names are present
  EXPECT_EQ("/model_name/body_name/456",
            drake_ros_viz::internal::CalcMarkerNamespace<int>(
                "model_name", "body_name", 123, 456));

  // Scoped model name and body name
  EXPECT_EQ("/model_scope/model_name/body_scope/body_name/456",
            drake_ros_viz::internal::CalcMarkerNamespace(
                "model_scope::model_name", "body_scope::body_name", 123, 456));

  // Model name with empty body name
  EXPECT_EQ(
      "/model_name/unnamed_body_123/456",
      drake_ros_viz::internal::CalcMarkerNamespace("model_name", "", 123, 456));

  // Scoped model name with empty body name
  EXPECT_EQ("/model_scope/model_name/unnamed_body_123/456",
            drake_ros_viz::internal::CalcMarkerNamespace(
                "model_scope::model_name", "", 123, 456));
}

TEST(NameConventions, CalcMarkerNamespaceWithoutBody) {
  // Geometry source and name is not empty
  EXPECT_EQ("/geometry_source/geometry_name",
            drake_ros_viz::internal::CalcMarkerNamespace("geometry_source",
                                                         "geometry_name", 123));

  // Geometry name is empty
  EXPECT_EQ(
      "/geometry_source/unnamed_geometry_123",
      drake_ros_viz::internal::CalcMarkerNamespace("geometry_source", "", 123));

  // Geometry name with / delimiters
  EXPECT_EQ("/geometry_source/scope_one/scope_two/geometry_name",
            drake_ros_viz::internal::CalcMarkerNamespace(
                "geometry_source", "scope_one/scope_two/geometry_name", 123));

  // Geometry name with :: delimiters
  EXPECT_EQ("/geometry_source/scope_one/scope_two/geometry_name",
            drake_ros_viz::internal::CalcMarkerNamespace(
                "geometry_source", "scope_one::scope_two::geometry_name", 123));

  // Geometry name with mixed delimiters
  EXPECT_EQ("/geometry_source/scope_one/scope_two/geometry_name",
            drake_ros_viz::internal::CalcMarkerNamespace(
                "geometry_source", "scope_one/scope_two::geometry_name", 123));

  // Geometry name with only :: delimiter
  EXPECT_EQ("/geometry_source/unnamed_geometry_123",
            drake_ros_viz::internal::CalcMarkerNamespace("geometry_source",
                                                         "::", 123));

  // Geometry name with only / delimiter
  EXPECT_EQ("/geometry_source/unnamed_geometry_123",
            drake_ros_viz::internal::CalcMarkerNamespace("geometry_source", "/",
                                                         123));
}
