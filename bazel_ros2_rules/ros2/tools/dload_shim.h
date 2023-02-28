#pragma once

#include <string>
#include <vector>

namespace bazel_ros2_rules {

/** Updates environment variables as specified by the given names and actions.
This is meant to be called just prior to main().

The names and actions are pairwise vectors.

The valid actions are:

["replace", new_value] -- overwrites the environment varible with `new_value`.

["set-if-not-set", new_value] -- if the environment varible is not set, then
sets it to `new_value`.

["path-replace", resource_path] -- overwrites the environment varible with the
Bazel runfiles location of the given `resource_path`.

["path-prepend", resource_path, ...] -- prepends to the environment varible the
Bazel runfiles location of the given `resource_path`s, in order. Each prepended
path will be followed by a colon to separate it from the remainder.

For all actions, if the new value contains the literal string `$PWD`, the first
instance of that string will be replaced by the current working directory path,
without a trailing slash.

@param names environment variables to be modified.
@param actions actions to be performed on each named environment variable. */
void ApplyEnvironmentActions(
    const std::string& argv0, const std::vector<const char*>& names,
    const std::vector<std::vector<const char*>>& actions);

}  // namespace bazel_ros2_rules
