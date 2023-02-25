# -*- python -*-

load(
    "//tools:dload.bzl",
    "do_dload_shim",
    "get_dload_shim_attributes",
)

_TEMPLATE = """\
#include "ros2/tools/dload_shim.h"

extern "C" int __real_main(int argc, char** argv);

extern "C" int __wrap_main(int argc, char** argv) {{
  std::vector<const char*> names = {names};
  std::vector<std::vector<const char*>> actions = {actions};
  bazel_ros2_rules::ApplyEnvironmentActions(argv[0], names, actions);
  return __real_main(argc, argv);
}}
"""

def _to_cc_list(collection):
    """Turn collection into a C++ aggregate initializer expression."""
    return "{" + ", ".join(collection) + "}"

def _dload_cc_shim_impl(ctx):
    return do_dload_shim(ctx, _TEMPLATE, _to_cc_list)

dload_cc_main = rule(
    doc = """\
    Generates a C++ source file with a __wrap_main function. The function will
    set the shim attributes in the environment and then call __real_main. This
    is designed for use with `linkopts = ["-Wl,--wrap=main"]` when linking a
    binary or test.
    """,
    attrs = get_dload_shim_attributes(),
    implementation = _dload_cc_shim_impl,
    output_to_genfiles = True,
)
