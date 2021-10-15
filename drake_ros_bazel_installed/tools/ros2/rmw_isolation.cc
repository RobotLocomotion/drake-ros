#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>

#include <rclcpp/rclcpp.hpp>
#include <rmw/rmw.h>

#include "tools/cpp/runfiles/runfiles.h"
#include "tools/ros2/rmw_isolation.h"

using bazel::tools::cpp::runfiles::Runfiles;

namespace ros2 {

void isolate_rmw_by_path(const std::string& argv0, const std::string& path)
{
  if (rclcpp::ok())
  {
    throw std::runtime_error(
        "middleware already initialized, too late for isolation");
  }
  std::string error;
  std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv0, &error));
  if (!runfiles) {
    throw std::runtime_error(error);
  }
  // NOTE(hidmic): no way around hard-coding this. Bazel's $(rootpath)
  // won't prepend the workspace name (drake_ros in this case). Maybe
  // relocate to an external repository?
  std::string isolated_env_rmw_location =
      runfiles->Rlocation("drake_ros/tools/ros2/isolated_rmw_env");
  std::string env_file_path{"isolated_rmw.env"};
  std::stringstream sstr;
  sstr << isolated_env_rmw_location << " --scratch-directory " << path
       << " --rmw-implementation " << rmw_get_implementation_identifier()
       << " --output " << env_file_path << " " << path;
  std::string command = sstr.str();
  // NOTE(hidmic): no way to redirect stderr here. No feedback on error.
  // Use POSIX popen?
  int status = std::system(command.c_str());
  if (status < 0) {
    throw std::system_error(errno, std::system_category(), strerror(errno));
  }
  if (status != 0) {
    throw std::runtime_error(isolated_env_rmw_location + " exited abnormally");
  }
  std::ifstream env_file{env_file_path};
  for(std::string line; std::getline(env_file, line); ) {
    const size_t separator_index = line.find("=");
    const std::string name = line.substr(0, separator_index);
    const std::string value =
        separator_index != std::string::npos ?
        line.substr(separator_index + 1) : "";
    if (setenv(name.c_str(), value.c_str(), 1) != 0) {
      throw std::system_error(errno, std::system_category(), strerror(errno));
    }
  }
}

}  // namespace ros2
