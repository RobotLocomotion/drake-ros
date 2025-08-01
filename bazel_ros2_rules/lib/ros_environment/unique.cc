#include <stdlib.h>
#include <unistd.h>
#include <algorithm>

#include "unique.h"

namespace bazel_ros2_rules {

namespace {

std::string replace(std::string str, const std::string& from, const std::string& to) {
  size_t pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.length(), to);
    pos += to.length();
  }
  return str;
}

std::string ltrim(std::string str, char pad = ' ') {
  str.erase(str.begin(), std::find_if(str.begin(), str.end(), [&](char c) {
    return c != pad;
  }));
  return str;
}

}  // namespace

void EnforceUniqueROSEnvironment(
    std::optional<std::string> unique_identifier,
    std::optional<std::string> scratch_directory,
    std::optional<std::filesystem::path> temp_dir)
{
  if (!unique_identifier.has_value()) {
    const char* test_target = getenv("TEST_TARGET");
    if (test_target != nullptr) {
      unique_identifier = ltrim(replace(test_target, "//", "/"), '/');
    } else {
      unique_identifier = std::to_string(getpid());
    }
  }

  if (!temp_dir.has_value()) {
    const char* test_tmpdir = getenv("TEST_TMPDIR");
    if (test_tmpdir != nullptr) {
      temp_dir = std::filesystem::path(test_tmpdir);
    } else {
      temp_dir = std::filesystem::temp_directory_path();
    }
  }

  if (!scratch_directory.has_value()) {
    scratch_directory = (temp_dir.value() / unique_identifier.value()).string();
    if (!std::filesystem::is_directory(scratch_directory.value())) {
      std::filesystem::create_directories(scratch_directory.value());
    }
  }

  // ROS wants to write text logs, so point it to the temporary test directory via ROS_HOME.
  const std::filesystem::path ros_home_directory = 
    std::filesystem::path(scratch_directory.value()) / "ros";
  setenv("ROS_HOME", ros_home_directory.c_str(), 1);
}

}  // namespace bazel_ros2_rules
