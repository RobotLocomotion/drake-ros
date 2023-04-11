# -*- python -*-

"""
The purpose of these rules is to support the propagation of runtime information
that is necessary for the execution of C/C++ binaries and tests that require
it.
"""

load(
    "//tools:dload.bzl",
    "do_dload_shim",
    "get_dload_shim_attributes",
)

_REEXEC_TEMPLATE = """\
#include "ros2/tools/dload_shim.h"

int main(int argc, const char * argv[]) {{
  const char * executable_path = "{executable_path}";
  std::vector<const char *> names = {names};
  std::vector<std::vector<const char *>> actions = {actions};
  return bazel_ros2_rules::ReexecMain(
      argc, argv, executable_path, names, actions);
}}
"""

def _to_cc_list(collection):
    """Turn collection into a C++ aggregate initializer expression."""
    return "{" + ", ".join(collection) + "}"

def _dload_cc_reexec_impl(ctx):
    return do_dload_shim(ctx, _REEXEC_TEMPLATE, _to_cc_list)

dload_cc_reexec = rule(
    doc = """\
    Generates C++ source code for a program that injects runtime environment
    information for C/C++ binaries that have such requirements. Using a C++
    program for C++ binaries simplifies UX during debugging sessions, as
    fork-follow behavior in common debuggers like gdb and lldb makes it
    transparent.

    This code uses a sentinel environment variable so that it only modifies the
    environment once. Any nested programs will use the top-level program's
    environment.

    See do_dload_shim() documentation for further reference.
    """,
    attrs = get_dload_shim_attributes(),
    implementation = _dload_cc_reexec_impl,
    output_to_genfiles = True,
)

_LDWRAP_TEMPLATE = """\
#include "ros2/tools/dload_shim.h"

extern "C" int __real_main(int argc, char** argv);

extern "C" int __wrap_main(int argc, char** argv) {{
  std::vector<const char*> names = {names};
  std::vector<std::vector<const char*>> actions = {actions};
  bazel_ros2_rules::ApplyEnvironmentActions(argv[0], names, actions);
  return __real_main(argc, argv);
}}
"""

def _dload_cc_ldwrap_impl(ctx):
    return do_dload_shim(ctx, _LDWRAP_TEMPLATE, _to_cc_list)

dload_cc_ldwrap = rule(
    doc = """\
    Generates C++ source code with a __wrap_main function. The function will
    set the runtime environment variables and then call __real_main. This is
    designed for use with `linkopts = ["-Wl,--wrap=main"]` when linking a
    binary or test.
    """,
    attrs = get_dload_shim_attributes(),
    implementation = _dload_cc_ldwrap_impl,
    output_to_genfiles = True,
)
