#pragma once

#include <filesystem>
#include <optional>
#include <string>

namespace bazel_ros2_rules {

void EnforceUniqueROSEnvironment(
    std::optional<std::string> unique_identifier = std::nullopt,
    std::optional<std::string> scratch_directory = std::nullopt,
    std::optional<std::filesystem::path> temp_dir = std::nullopt);

}  // namespace bazel_ros2_rules
