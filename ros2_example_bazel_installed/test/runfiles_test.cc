/*
At present, this works via `bazel test` and `bazel run`.
However, it fails with `bazel-bin/...`.

It needs to work in this mode :(
*/
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <filesystem>
#include <stdexcept>

#include "tools/cpp/runfiles/runfiles.h"

using bazel::tools::cpp::runfiles::Runfiles;

// Simplified version of
// https://github.com/RobotLocomotion/drake/blob/v1.5.0/common/find_runfiles.cc#L36-L123
std::unique_ptr<Runfiles> CreateRunfiles() {
  std::unique_ptr<Runfiles> runfiles;
  std::string bazel_error;

  if (std::getenv("TEST_SRCDIR")) {
    // I think this is how it works.
    runfiles.reset(Runfiles::CreateForTest(&bazel_error));
  } else if ((std::getenv("RUNFILES_MANIFEST_FILE") != nullptr) ||
             (std::getenv("RUNFILES_DIR") != nullptr)) {
    // Dunno if this mode gets used - however, it probably should.
    runfiles.reset(Runfiles::Create({}, &bazel_error));
  } else {
    // This mode fails.
    const std::string& argv0 = std::filesystem::read_symlink({
        "/proc/self/exe"}).string();
    runfiles.reset(Runfiles::Create(argv0, &bazel_error));
  }
  if (runfiles == nullptr) {
    throw std::runtime_error(bazel_error);
  }
  return runfiles;
}

int main() {
  auto runfiles = CreateRunfiles();
  std::string file_path = runfiles->Rlocation(
      "ros2_example_bazel_installed/test/runfiles_test_data.txt");
  std::cout << file_path << std::endl << "Good!" << std::endl;
  return 0;
}
