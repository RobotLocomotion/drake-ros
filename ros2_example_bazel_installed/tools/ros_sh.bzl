# -*- python -*-
# vi: set ft=python :

# Adapted from Anzu.

load(
    "@ros2//:ros_py.bzl",
    "ros_py_binary",
)
load(":generate_file.bzl", "generate_file")

_TEMPLATE = """
import os, sys
from ros2_example_bazel_installed.common.runfiles import (
    SubstituteMakeVariableLocation,
    UpdateRunfilesEnviron,
)
build_file_args = {args}
constant_args = list(map(SubstituteMakeVariableLocation, build_file_args))
args = constant_args + sys.argv[1:]
os.execve(args[0], args, env=UpdateRunfilesEnviron(os.environ))
""".lstrip()


def expand_location_target_if_needed(binary):
    """
    If supplied relative path, replace with absolute for use with
    `SubstituteMakeVariableLocation`.
    https://docs.bazel.build/versions/master/be/make-variables.html#predefined_label_variables
    """  # noqa
    prefix = "$(location :"
    suffix = ")"
    if binary.startswith(prefix) and binary.endswith(suffix):
        rel_name = binary[len(prefix):-len(suffix)]
        return "$(location //{}:{})".format(native.package_name(), rel_name)
    else:
        return binary


def ros_sh_alias(
        name,
        target,
        py_binary_rule = ros_py_binary,
        deps = None,
        **kwargs):
    """Use this in lieu of `sh_binary` to forward aliases.

    This is because using `sh_binary` to forward binaries does not work well
    for Python binaries (i.e., Bazel complains about the binary having
    more than one source file).
    """

    # TODO(drake-ros#118): Use more robust mechanism to leverage interface
    # manifest / indexing.
    if deps != None:
        fail("You cannot supply `deps` to this target.")

    script = "{}_ros_sh_args_script.py".format(name)
    args = ["$(location {})".format(expand_location_target_if_needed(target))]
    generate_file(
        name = script,
        content = _TEMPLATE.format(args = repr(args)),
    )
    py_binary_rule(
        name = name,
        main = script,
        srcs = [script],
        data = [target],
        deps = ["//common:runfiles_py"],
        **kwargs
    )
