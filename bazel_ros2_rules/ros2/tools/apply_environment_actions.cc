#include "apply_environment_actions.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <sstream>

#include "tools/cpp/runfiles/runfiles.h"

namespace bazel_ros2_rules {

using bazel::tools::cpp::runfiles::Runfiles;
using std::cerr;
using std::getenv;

#define DEMAND(condition)                                                     \
  do {                                                                        \
    if (!(condition)) {                                                       \
      cerr << "Failure at " << __FILE__ << ":" << __LINE__ << ": condition '" \
           << #condition << "' failed.";                                      \
      ::abort();                                                              \
    }                                                                         \
  } while (0)

void ApplyEnvironmentActions(
    const std::string& argv0, const std::vector<const char*>& names,
    const std::vector<std::vector<const char*>>& actions) {
  DEMAND(names.size() == actions.size());

  // Check whether this environment has already been shimmed.
  // If so, then don't try to do it again.
  const char* const kShimmedSentinel = "_BAZEL_ROS2_RULES_SHIMMED";
  if (getenv(kShimmedSentinel) != nullptr) {
    return;
  }
  setenv(kShimmedSentinel, "", 1);

  // Find our runfiles.
  std::unique_ptr<Runfiles> runfiles;
  {
    std::string error;
    runfiles.reset(Runfiles::Create(argv0, &error));
    if (!runfiles && std::filesystem::is_symlink(argv0)) {
      const std::string follow = std::filesystem::read_symlink(argv0);
      runfiles.reset(Runfiles::Create(follow, &error));
    }
    if (!runfiles) {
      cerr << "Failure at " << __FILE__ << ":" << __LINE__ << ": "
           << "could not create Runfiles: " << error << "\n";
      ::abort();
    }
  }

  // Actions indicate how to change environment variables
  const std::string kReplace = "replace";
  const std::string kSetIfNotSet = "set-if-not-set";
  const std::string kPathReplace = "path-replace";
  const std::string kPathPrepend = "path-prepend";

  // Apply each action in turn.
  for (size_t i = 0; i < names.size(); ++i) {
    const char* const name = names[i];
    DEMAND(name != nullptr);
    const char* const prior_value = getenv(name);
    const std::vector<const char*>& action_list = actions[i];
    DEMAND(action_list.size() >= 1);
    const char* const action = action_list.front();
    DEMAND(action != nullptr);

    // Compute the updated value.
    std::stringstream value_stream;
    if (action == kReplace) {
      DEMAND(action_list.size() == 2);
      value_stream << action_list[1];
    } else if (action == kSetIfNotSet) {
      DEMAND(action_list.size() == 2);
      if (prior_value != nullptr) {
        continue;
      }
      value_stream << action_list[1];
    } else if (action == kPathReplace) {
      DEMAND(action_list.size() == 2);
      value_stream << runfiles->Rlocation(action_list[1]);
    } else if (action == kPathPrepend) {
      DEMAND(action_list.size() >= 2);
      for (size_t j = 1; j < action_list.size(); ++j) {
        value_stream << runfiles->Rlocation(action_list[j]) << ":";
      }
      if (prior_value != nullptr) {
        value_stream << prior_value;
      }
    } else {
      cerr << "Failure at " << __FILE__ << ":" << __LINE__ << ": "
           << "unknown action '" << action << "' for '" << name << "'\n";
      std::abort();
    }
    std::string value = value_stream.str();

    // Do some magic.
    std::string::size_type location;
    if ((location = value.find("$PWD")) != std::string::npos) {
      value.replace(location, 4, std::filesystem::current_path());
    }

    // Set the new variable value.
    if (setenv(name, value.c_str(), 1) != 0) {
      cerr << "Failure at " << __FILE__ << ":" << __LINE__ << ": "
           << "failed to set '" << name << "' to '" << value << "'\n";
    }
  }
}

}  // namespace bazel_ros2_rules
