#pragma once

#include <sstream>
#include <string>
#include <unordered_set>

#include "drake_ros/viz/name_conventions.h"

namespace drake_ros {
namespace viz {
namespace internal {

std::string ReplaceAllOccurrences(std::string string, const std::string& target,
                                  const std::string& replacement) {
  std::string::size_type n = 0;
  while ((n = string.find(target, n)) != std::string::npos) {
    string.replace(n, target.size(), replacement);
    n += replacement.size();
  }
  return string;
}

/* Formulate marker namespace with just the provided prefix as well as the
  owning source name of the geometry.
  @param[in] prefix user defined prefix for this marker namespace
  @param[in] geometry_owning_source_name owning source name for the geometry
 */
std::string CalcMarkerNamespace(
    const std::string& prefix, const std::string& geometry_owning_source_name) {
  return prefix + ReplaceAllOccurrences(geometry_owning_source_name, "::", "/");
}

/* Formulate marker namespace given the model instance name, body name, body
  index, geometry name and geometry ID value.
  @param[in] prefix user defined prefix for this marker namespace
  @param[in] model_instance_name name of a given model instance.
  @param[in] body_name name of a given body.
  @param[in] geometry_name name of a given geometry.
  @returns formulated marker namespace.
 */
std::string CalcHierarchicalMarkerNamespace(
    const std::string& prefix, const std::string& model_instance_name,
    const std::string& body_name, const std::string& geometry_name) {
  std::stringstream ss;
  ss << prefix << ReplaceAllOccurrences(model_instance_name, "::", "/") << "/";

  if (body_name.empty()) {
    ss << "unnamed_body/";
  } else {
    ss << ReplaceAllOccurrences(body_name, "::", "/") << "/";
  }

  if (geometry_name.empty()) {
    ss << "unnamed_geometry";
  } else {
    ss << geometry_name;
  }

  return ss.str();
}

/* Formulate marker namespace given the geometry source name, geometry name
  and geometry ID value.
  @param[in] prefix user defined prefix for this marker namespace
  @param[in] geometry_source_name name of the source owning a given geometry.
  @param[in] geometry_name name of a given geometry.
  @returns formulated marker namespace.
 */
std::string CalcHierarchicalMarkerNamespace(
    const std::string& prefix, const std::string& geometry_source_name,
    const std::string& geometry_name) {
  std::stringstream ss;
  ss << prefix << ReplaceAllOccurrences(geometry_source_name, "::", "/") << "/";

  if (geometry_name.empty()) {
    ss << "unnamed_geometry";
  } else {
    ss << ReplaceAllOccurrences(geometry_name, "::", "/");
  }

  return ss.str();
}

}  // namespace internal
}  // namespace viz
}  // namespace drake_ros
