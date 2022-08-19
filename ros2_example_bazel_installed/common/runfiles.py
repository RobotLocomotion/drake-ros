"""Helper functions for dealing with Bazel runfiles, i.e., declared data
dependencies of libraries and programs.

Adapted from Anzu.
"""

import os

from bazel_tools.tools.python.runfiles.runfiles import Create as _Create

_runfiles = _Create()
_workspace = "ros2_example_bazel_installed"


def Rlocation(respath):
    return _runfiles.Rlocation(respath)


def SubstituteMakeVariableLocation(arg):
    """Given a string argument that might be a $(location //foo) substitution,
    looks up ands return the specified runfile location for $(location //foo)
    if the argument is in such a form, or if not just returns the argument
    unchanged.  Only absolute labels ("//foo" or "@drake//bar") are supported.
    It is an error if the argument looks any other $(...).  For details see
    https://docs.bazel.build/versions/master/be/make-variables.html.
    """
    if arg.startswith("$(location "):
        label = arg[11:-1]
        assert label.startswith("@") or label.startswith("//"), label
        if not label.startswith("@"):
            label = f"@{_workspace}{label}"
        elif label.startswith("@//"):
            label = label.replace("@//", f"@{_workspace}//")
        normalized = label[1:]  # Strip the leading @.
        normalized = normalized.replace("//:", "/")
        normalized = normalized.replace("//", "/")
        normalized = normalized.replace(":", "/")
        arg = Rlocation(normalized)
    assert not arg.startswith("$("), arg
    return arg


def UpdateRunfilesEnviron(env):
    """Returns a copy of env with the Runfiles environment variables for child
    processes set.

    Do not abuse this to purely access RUNFILES_DIR or other variables to then
    access specific files.
    """
    result = dict(env)
    result.update(_runfiles.EnvVars())
    return result
