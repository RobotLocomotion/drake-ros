"""
Utility functions to aid repository sandboxing of dependencies
external to the enclosing Bazel workspace.
"""

import os


def make_symlink_forest_mapping(name, mapping):
    """
    Makes a function that maps paths outside a Bazel workspace
    to paths within a repository in that workspace.

    All provided outer directory paths are symlinked at their
    corresponding inner directory paths. Paths that cannot be
    mapped will be returned unchanged. For paths to be found
    among runfiles in runtime, set the ``external`` keyword
    argument to True.

    :param name: name of the enclosing repository
    :param mapping: a key-value mapping from outer directory paths
      to inner directory paths
    :returns: callable that takes a path outside the workspace
      and yields a path inside the workspace, and optionall takes
      an ``external`` keyword argument.
    """
    def _map(path, external=False):
        path = os.path.normpath(path)
        for outer_path, inner_path in mapping.items():
            if path.startswith(outer_path):
                path = os.path.normpath(
                    path.replace(outer_path, inner_path)
                )
                if external:
                    path = os.path.join(name, path)
                return path
        return path

    for outer_path, inner_path in mapping.items():
        if inner_path == '.':
            continue
        os.symlink(outer_path, inner_path, target_is_directory=True)

    return _map
