# -*- python -*-
# vi: set ft=python :

"""
The purpose of these macros is to support the propagation of runtime information
that is key for proper execution from Python libraries to Python binaries and
tests.
"""

load(
    "//tools/skylark:dload.bzl",
    "do_dload_shim",
    "do_dload_aware_library",
    "get_dload_shim_attributes",
    "get_dload_aware_target_attributes",
)

DLOAD_PY_SHIM_TEMPLATE = """\
import os
import sys

from bazel_tools.tools.python.runfiles import runfiles

r = runfiles.Create()
# NOTE(hidmic): unlike its C++ equivalent, Python runfiles'
# builtin tools will only look for runfiles in the manifest
# if there is a manifest
runfiles_dir = r.EnvVars()['RUNFILES_DIR']

def rlocation(path):
    return r.Rlocation(path) or os.path.join(runfiles_dir, path)

for name, action in zip({names}, {actions}):  # noqa
    action_type, action_args = action[0], action[1:]
    if action_type == 'replace':
        assert len(action_args) == 1
        value = action_args[0]
    elif action_type == 'path-replace':
        assert len(action_args) == 1
        value = rlocation(action_args[0])
    elif action_type == 'path-prepend':
        assert len(action_args) > 0
        value = ':'.join([rlocation(path) for path in action_args])
        if name in os.environ:
            value += ':' + os.environ[name]
    else:
        assert False  # should never get here
    os.environ[name] = value

executable_path = r.Rlocation('{executable_path}')  # noqa
argv = [executable_path] + sys.argv[1:]
os.execv(executable_path, argv)
"""

def to_py_list(collection):
    """Turn collection into a Python list expression."""
    return "[" + ", ".join(collection) + "]"

def _dload_py_shim_impl(ctx):
    return do_dload_shim(ctx, DLOAD_PY_SHIM_TEMPLATE, to_py_list)

dload_py_shim = rule(
    attrs = get_dload_shim_attributes(),
    output_to_genfiles = True,
    implementation = _dload_py_shim_impl,
)
"""
This rule() generates a Python shim that can carry runtime information.
See do_dload_shim() documentation for further reference.
"""

def _dload_aware_py_library_impl(ctx):
    return do_dload_aware_library(ctx, PyInfo)

dload_aware_py_library = rule(
    attrs = get_dload_aware_target_attributes(),
    implementation = _dload_aware_py_library_impl,
)
"""
This rule() decorates a py_library() rule to carry runtime information.
See do_dload_aware_library() documentation for further reference.
"""
