#include <string>

#include "internal_name_conventions.h"  // NOLINT
#include <gtest/gtest.h>

TEST(NameConventions, CalcMarkerNamespace) {
  EXPECT_EQ("prefix_geometry_owning_source_name",
            drake_ros::viz::internal::CalcMarkerNamespace(
                "prefix_", "geometry_owning_source_name"));

  EXPECT_EQ("geometry_owning_source_name",
            drake_ros::viz::internal::CalcMarkerNamespace(
                "", "geometry_owning_source_name"));

  EXPECT_EQ("prefix/model/geometry_owning_source_name",
            drake_ros::viz::internal::CalcMarkerNamespace(
                "prefix/", "model::geometry_owning_source_name"));
}

TEST(NameConventions, CalcHierarchicalMarkerNamespaceWithBody) {
  // All names are present
  EXPECT_EQ("prefix/model_name/body_name/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "prefix/", "model_name", "body_name", "geometry_name"));

  // Scoped model name and body name
  EXPECT_EQ("prefix/model_scope/model_name/body_scope/body_name/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "prefix/", "model_scope::model_name", "body_scope::body_name",
                "geometry_name"));

  // Model name with empty body name
  EXPECT_EQ("model_name/unnamed_body/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "", "model_name", "", "geometry_name"));

  // Scoped model name with empty body name and empty geometry name
  EXPECT_EQ("model_scope/model_name/unnamed_body/unnamed_geometry",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "", "model_scope::model_name", "", ""));
}

TEST(NameConventions, CalcHierarchicalMarkerNamespaceWithoutBody) {
  // Geometry source and name is not empty
  EXPECT_EQ("prefix/geometry_source/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "prefix/", "geometry_source", "geometry_name"));

  // Geometry name is empty
  EXPECT_EQ("prefix/geometry_source/unnamed_geometry",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "prefix/", "geometry_source", ""));

  // Geometry name with / delimiters
  EXPECT_EQ("geometry_source/scope_one/scope_two/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "", "geometry_source", "scope_one/scope_two/geometry_name"));

  // Geometry name with :: delimiters
  EXPECT_EQ("geometry_source/scope_one/scope_two/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "", "geometry_source", "scope_one::scope_two::geometry_name"));

  // Geometry name with mixed delimiters
  EXPECT_EQ("geometry_source/scope_one/scope_two/geometry_name",
            drake_ros::viz::internal::CalcHierarchicalMarkerNamespace(
                "", "geometry_source", "scope_one/scope_two::geometry_name"));
}
