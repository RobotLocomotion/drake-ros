# -*- python -*-

"""
The purpose of these macros is to support configuration of the runtime
environment in which executables are run.

The primary macro of interest is `do_dload_shim`, which aids language
specific shim generation.
"""

load(
    "//tools:ament_index.bzl",
    "AggregatedAmentIndexes",
    "ament_index_prefixes",
)

MAGIC_VARIABLES = {
    "${LOAD_PATH}": "LD_LIBRARY_PATH",  # for Linux
}

def _unique(input_list):
    """Extracts unique values from input list, while preserving their order."""
    output_list = []
    for item in input_list:
        if item not in output_list:
            output_list.append(item)
    return output_list

def _normpath(path):
    """
    Normalizes a path by removing redundant separators and up-level references.

    Equivalent to Python's os.path.normpath().
    """
    path_parts = path.split("/")
    normalized_path_parts = [path_parts[0]]
    for part in path_parts[1:]:
        if part == "." or part == "":
            continue
        if part == "..":
            normalized_path_parts.pop()
            continue
        normalized_path_parts.append(part)
    return "/".join(normalized_path_parts)

def get_dload_shim_attributes():
    """
    Yields attributes common to all dload_shim-based rules.

    This macro aids rule declaration and as such it is not meant
    to be used in any other context (like a BUILD.bazel file).
    """
    return {
        "target": attr.label(
            aspects = [ament_index_prefixes],
            mandatory = True,
            allow_files = True,
            executable = True,
            cfg = "target",
        ),
        "env_changes": attr.string_list_dict(),
    }

def _workaround_issue311(ament_prefixes, env_changes):
    # Work around drake-ros#311 and ensure that we have our
    # ${AMEND_PREFIX_PATH}/lib libraries also exposed on LD_LIBRARY_PATH.
    lib_folders = []
    for prefix in ament_prefixes:
        if prefix.endswith("/lib"):
            lib_folders.append(prefix)
    if len(lib_folders) > 0:
        if "LD_LIBRARY_PATH" not in env_changes:
            env_changes["LD_LIBRARY_PATH"] = ["path-prepend"]
    lib_folders = depset(lib_folders).to_list()
    env_changes["LD_LIBRARY_PATH"].extend(lib_folders)

def do_dload_shim(ctx, template, to_list):
    """
    Implements common dload_shim rule functionality.

    This macro is a parametrized rule implementation and as such it is not
    meant to be used in any other context (like a BUILD.bazel file).

    Args:
        ctx: context of a Bazel rule
        template: string template for the shim
        to_list: macro for list interpolation

    It expects the following attributes on ctx:

        target: executable target to be shimmed
        env_changes: runtime environment changes to be applied, as a mapping
          from environment variable names to actions to be performed on them.
          Actions are (action_type, action_args) tuples. Supported action types
          are: 'path-prepend', 'path-replace', 'set-if-not-set', and 'replace'.
          Paths are resolved relative to the runfiles directory of the
          downstream executable.
          Also, see MAGIC_VARIABLES for platform-independent runtime
          environment specification.

    You may use get_dload_shim_attributes() on rule definition.
    """
    executable_file = ctx.executable.target

    env_changes = {
        MAGIC_VARIABLES.get(name, default = name): (
            _parse_runtime_environment_action(ctx, action)
        )
        for name, action in ctx.attr.env_changes.items()
    }

    # Add ament resource index paths from targets we depend on
    if AggregatedAmentIndexes in ctx.attr.target:
        if "AMENT_PREFIX_PATH" not in env_changes:
            env_changes["AMENT_PREFIX_PATH"] = ["path-prepend"]
        if env_changes["AMENT_PREFIX_PATH"][0] != "path-prepend":
            fail("failed assumption - AMENT_PREFIX_PATH was not prepended to")
        ament_prefixes = ctx.attr.target[AggregatedAmentIndexes].prefixes

        # Deduplicate entries to avoid hitting 'Argument list too long' errors.
        ament_prefixes = depset(ament_prefixes).to_list()
        env_changes["AMENT_PREFIX_PATH"].extend(ament_prefixes)

        _workaround_issue311(ament_prefixes, env_changes)

    envvars = env_changes.keys()

    # TODO(eric.cousineau): Should deduplicate entries for path-prepend and
    # path-append.
    actions = env_changes.values()

    shim_content = template.format(
        # Deal with usage in external workspaces' BUILD.bazel files
        executable_path = _normpath("{}/{}".format(
            ctx.workspace_name,
            executable_file.short_path,
        )),
        names = to_list([repr(name) for name in envvars]),
        actions = to_list([
            to_list([
                repr(field)
                for field in action
            ])
            for action in actions
        ]),
    )
    shim = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(shim, shim_content, True)
    return [DefaultInfo(
        files = depset([shim]),
        data_runfiles = ctx.runfiles(files = [shim]),
    )]

def _resolve_runfile_path(ctx, path):
    """
    Resolves a package relative path into an (expected) runfiles directory
    relative path.

    All `$(rootpath...)` templates in the given path, if any, will be expanded.
    """
    return _normpath(ctx.expand_location(path))

def _resolve_runfile_path_with_bang_suffix(ctx, path):
    """Same as above, but preserve bang (!) suffix."""
    suffix = ""
    if path.endswith("!"):
        suffix = "!"
        path = path[:-1]
    return _resolve_runfile_path(ctx, path) + suffix

def _parse_runtime_environment_action(ctx, action):
    """
    Parses a runtime environment action, validating types and resolving paths.
    """
    action_type, action_args = action[0], action[1:]
    if action_type == "path-prepend":
        if len(action_args) == 0:
            tpl = "'{}' action requires at least one argument"
            fail(msg = tpl.format(action_type))
        action_args = _unique([
            _resolve_runfile_path_with_bang_suffix(ctx, path)
            for path in action_args
        ])
    elif action_type in ("path-replace", "set-if-not-set", "replace"):
        if len(action_args) != 1:
            tpl = "'{}' action requires exactly one argument"
            fail(msg = tpl.format(action_type))
        if action_type.startswith("path"):
            action_args = [_resolve_runfile_path(ctx, action_args[0])]
    else:
        fail(msg = "'{}' action is unknown".format(action_type))
    return [action_type] + action_args
