# -*- python -*-

"""
The purpose of these macros is to support the propagation of runtime information
that is key for proper execution from C/C++ libraries to C/C++ binaries and
tests.
"""

load(
    "//tools/skylark:dload.bzl",
    "do_dload_shim",
    "get_dload_shim_attributes",
)

DLOAD_CC_SHIM_TEMPLATE = """\
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include "tools/cpp/runfiles/runfiles.h"

using bazel::tools::cpp::runfiles::Runfiles;

int main(int argc, const char * argv[]) {{
  std::string error;
  std::unique_ptr<Runfiles> runfiles(Runfiles::Create(argv[0], &error));
  if (!runfiles) {{
    std::cerr << "ERROR: " << error << std::endl;
    return -1;
  }}

  std::vector<std::string> names = {names};
  std::vector<std::vector<std::string>> actions = {actions};  // NOLINT
  for (size_t i = 0; i < names.size(); ++i) {{
    std::stringstream value;
    if (actions[i][0] == "replace") {{
      assert(actions[i].size() == 2);
      value << actions[i][1];
    }} else if (actions[i][0] == "path-replace") {{
      assert(actions[i].size() == 2);
      value << runfiles->Rlocation(actions[i][1]);
    }} else if (actions[i][0] == "path-prepend") {{
      assert(actions[i].size() >= 2);
      for (size_t j = 1; j < actions[i].size(); ++j) {{
        value << runfiles->Rlocation(actions[i][j]) << ":";
      }}

      const char * raw_value = getenv(names[i].c_str());
      if (raw_value != nullptr) {{
        value << raw_value;
      }}
    }} else {{
      assert(false);  // should never get here
    }}

    if (setenv(names[i].c_str(), value.str().c_str(), 1) != 0) {{
      std::cerr << "ERROR: failed to set " << names[i] << std::endl;
    }}
  }}

  const std::string executable_path =
      runfiles->Rlocation("{executable_path}");  // NOLINT

  char ** other_argv = new char*[argc + 1];
  other_argv[0] = strdup(executable_path.c_str());
  for (int i = 1; i < argc; ++i) {{
    other_argv[i] = strdup(argv[i]);
  }}
  other_argv[argc] = NULL;
  int ret = execv(other_argv[0], other_argv);
  std::cout << "ERROR: " << strerror(errno) << std::endl;
  return ret;
}}
"""

def to_cc_list(collection):
    """Turn collection into a C++ aggregate initializer expression."""
    return "{" + ", ".join(collection) + "}"

def _dload_cc_shim_impl(ctx):
    return do_dload_shim(ctx, DLOAD_CC_SHIM_TEMPLATE, to_cc_list)

dload_cc_shim = rule(
    attrs = get_dload_shim_attributes(),
    implementation = _dload_cc_shim_impl,
    output_to_genfiles = True,
)
"""
This rule() generates a C++ shim that can carry runtime information.
See do_dload_shim() documentation for further reference.
"""
