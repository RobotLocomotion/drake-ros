# -*- python -*-

"""
The purpose of these macros is to support the propagation of runtime information
that is key for proper execution from libraries to executables that depend on
them.

The two primary macros of interest are `do_dload_shim`, which aids language
specific shim generation, and `do_dload_aware_target`, which can decorate
existing targets with runtime information.
"""

MAGIC_VARIABLES = {
    "${LOAD_PATH}": "LD_LIBRARY_PATH"  # for Linux
}

RuntimeInfo = provider(fields = ['env_changes'])
"""
This provider carries runtime information through the build graph.

Attributes

    env_changes: runtime environment changes to be applied, as a mapping from
        environment variable names to actions to be performed on them.
        Actions are (action_type, action_args) tuples. Supported action types
        are: 'path-prepend', 'path-replace', and 'replace'. Paths are resolved
        relative to the runfiles directory of the downstream executable.
        Also, see MAGIC_VARIABLES for platform-independent runtime environment
        specification.
"""

def unique(input_list):
    """Extracts unique values from input list, while preserving their order."""
    output_list = []
    for item in input_list:
        if item not in output_list:
            output_list.append(item)
    return output_list

def merge_runtime_environment_changes(base, head):
    """
    Merges runtime environment head changes into base.

    Merging runtime environment actions other than 'path-prepend' is not
    allowed as results would silently vary depending on build order.
    """
    for envvar, head_action in head.items():
        if envvar not in base:
            base[envvar] = head_action
            continue

        base_action = base[envvar]

        base_action_type = base_action[0]
        head_action_type = head_action[0]

        if "replace" in base_action_type or "replace" in head_action_type:
            tpl = "Got '{}' and '{}' actions, results depend on build order"
            fail(msg = tpl.format(base_action_type, head_action_type))

        if base_action_type != head_action_type or \
           base_action_type != "path-prepend":
            tpl = "Expected 'path-prepend' actions, got '{}' and '{}'"
            fail(msg = tpl.format(base_action_type, head_action_type))

        merged_action_type = base_action_type

        base_action_args = base_action[1:]
        head_action_args = head_action[1:]
        merged_action_args = unique(base_action_args + head_action_args)

        base[envvar] = [merged_action_type] + merged_action_args
    return base


def merge_runtime_info(base_info, head_info):
    """
    Merges 'head' runtime information into 'base' runtime information,
    then returns the latter.
    """
    merge_runtime_environment_changes(
        base_info.env_changes, head_info.env_changes
    )
    return base_info

def collect_runtime_info(targets):
    """Returns all targets' runtime information merged into one."""
    runtime_info = RuntimeInfo(env_changes = {})
    for target in targets:
        if RuntimeInfo not in target:
            continue
        merge_runtime_info(runtime_info, target[RuntimeInfo])
    return runtime_info

def get_runtime_environment_changes(runtime_info):
    """
    Returns changes to be applied to the runtime environment as
    an (envvars, actions) tuple.
    """
    actions = []
    for action in runtime_info.env_changes.values():
        action_type = action[0]
        if action_type == "path-prepend":
            action_args = action[1:]
            common_action_args = []
            important_action_args = []
            for path in action_args:
                if path.endswith("!"):
                    important_action_args.append(path[:-1])
                else:
                    common_action_args.append(path)
            action_args = unique(important_action_args + common_action_args)
            action = [action_type] + action_args
        actions.append(action)
    return list(runtime_info.env_changes.keys()), actions

def normpath(path):
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
    """Yields attributes common to all dload_shim-based rules."""
    return {
        "target": attr.label(
            mandatory = True,
            allow_files = True,
            executable = True,
            cfg = "target",
        ),
        "data": attr.label_list(allow_files = True),
        "deps": attr.label_list(),
    }

def do_dload_shim(ctx, template, to_list):
    """
    Implements common dload_shim rule functionality.

    All runtime information is merged, and made available to the
    executable by the shim.

    Args:
        ctx: context of a Bazel rule
        template: string template for the shim
        to_list: macro for list interpolation

    It expects the following attributes on ctx:

        target: executable target to be shimmed
        data: executable data dependencies, may provide RuntimeInfo
        deps: executable dependencies, may provide RuntimeInfo

    You may use get_dload_shim_attributes() on rule definition.
    """
    executable_file = ctx.executable.target

    runtime_info = merge_runtime_info(
        collect_runtime_info(ctx.attr.data),
        collect_runtime_info(ctx.attr.deps)
    )

    envvars, actions = get_runtime_environment_changes(runtime_info)

    shim_content = template.format(
        # Deal with usage in external workspaces' BUILD.bazel files
        executable_path=normpath("{}/{}".format(
            ctx.workspace_name, executable_file.short_path
        )),
        names=to_list([repr(name) for name in envvars]),
        actions=to_list([
            to_list([
                repr(field) for field in action
            ]) for action in actions
        ]),
    )
    shim = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(shim, shim_content, True)
    return [DefaultInfo(
        files = depset([shim]),
        data_runfiles = ctx.runfiles(files = [shim]),
    )]

def resolve_runfile_path(ctx, path):
    """
    Resolves a package relative path into an (expected) runfiles directory
    relative path.

    All `$(rootpath...)` templates in the given path, if any, will be expanded.
    """
    path = normpath(ctx.expand_location(path))
    path = path.lstrip("@")
    if path.startswith("/"):
        path = ctx.workspace_name + path
    return path

def parse_runtime_environment_action(ctx, action):
    """
    Parses a runtime environment action, validating types and resolving paths.
    """
    action_type, action_args = action[0], action[1:]
    if action_type == "path-prepend":
        if len(action_args) == 0:
            tpl = "'{}' action requires at least one argument"
            fail(msg = tpl.format(action_type))
        action_args = unique([
            resolve_runfile_path(ctx, path[:-1]) + "!" if path.endswith("!")
            else resolve_runfile_path(ctx, path) for path in action_args
        ])
    elif action_type in ("path-replace", "replace"):
        if len(action_args) != 1:
            tpl = "'{}' action requires exactly one argument"
            fail(msg = tpl.format(action_type))
        if action_type.startswith("path"):
            action_args = [resolve_runfile_path(ctx, action_args[0])]
    else:
        fail(msg = "'{}' action is unknown".format(action_type))
    return [action_type] + action_args

def get_dload_aware_target_attributes():
    """
    Yields attributes common to all dload_aware_target rules.

    See do_dload_aware_target() documentation for further reference.
    """
    return {
        "base": attr.label(mandatory = True),
        "data": attr.label_list(allow_files = True),
        "deps": attr.label_list(),
        "runenv": attr.string_list_dict(),
    }

def do_dload_aware_target(ctx):
    """
    Implements common dload_aware_target rule functionality.

    Any specified runtime info is augmented with that of build and runtime
    dependencies, and then propagated along with base target runfiles
    i.e. its DefaultInfo provider.

    Args:
        ctx: context of a Bazel rule

    It expects the following attributes on ctx:

      base: target to extend with runtime information
      data: base target's data dependencies, may provide RuntimeInfo
      deps: base target's dependencies, may provide RuntimeInfo
      runenv: runtime environment changes for this target

    You may use get_dload_aware_target_attributes() on rule definition.
    """
    runtime_info = RuntimeInfo(env_changes = {
        MAGIC_VARIABLES.get(name, default=name):
        parse_runtime_environment_action(ctx, action)
        for name, action in ctx.attr.runenv.items()
    })
    merge_runtime_info(runtime_info, collect_runtime_info(ctx.attr.data))
    merge_runtime_info(runtime_info, collect_runtime_info(ctx.attr.deps))
    return [
        # Recreate base's DefaultInfo to workaround
        # https://github.com/bazelbuild/bazel/issues/9442
        DefaultInfo(
            files = ctx.attr.base[DefaultInfo].files,
            data_runfiles = ctx.attr.base[DefaultInfo].data_runfiles,
            default_runfiles = ctx.attr.base[DefaultInfo].default_runfiles,
        ),
        runtime_info
    ]

def do_dload_aware_library(ctx, kind):
    """
    Implements common dload_aware_library rule functionality.

    It builds on top of dload_aware_target functionality, also propagating
    library specific providers e.g. CcInfo providers for cc_library base
    targets.
    """
    providers = do_dload_aware_target(ctx)
    providers.append(ctx.attr.base[kind])
    return providers

dload_aware_target = rule(
    attrs = get_dload_aware_target_attributes(),
    implementation = do_dload_aware_target,
)
