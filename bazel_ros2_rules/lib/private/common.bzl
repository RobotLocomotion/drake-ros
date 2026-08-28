# -*- python -*-

load("//:distro.bzl", "REPOSITORY_ROOT")

def share_filegroup(name, share_directories):
    native.filegroup(
        name = name,
        srcs = [path for path in native.glob(
            include = [
                "{}/**".format(dirpath)
                for dirpath in share_directories
            ],
            exclude = [
                "*/cmake/**",
                "*/environment/**",
                "*/*.sh",
                "*/*.bash",
                "*/*.dsv",
            ],
            allow_empty = True,
        ) if " " not in path],
        # NOTE(hidmic): workaround lack of support for spaces.
        # See https://github.com/bazelbuild/bazel/issues/4327.
    )

def interfaces_filegroup(name, share_directory):
    native.filegroup(
        name = name + "_defs",
        srcs = native.glob(include = [
            "{}/**/*.json".format(share_directory),
            "{}/**/*.idl".format(share_directory),
            "{}/**/*.msg".format(share_directory),
            "{}/**/*.srv".format(share_directory),
            "{}/**/*.action".format(share_directory),
        ], allow_empty = True),
    )

def _generate_file_impl(ctx):
    out = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.write(out, ctx.attr.content, ctx.attr.is_executable)
    return [DefaultInfo(
        files = depset([out]),
        data_runfiles = ctx.runfiles(files = [out]),
    )]

generate_file = rule(
    attrs = {
        "content": attr.string(mandatory = True),
        "is_executable": attr.bool(default = False),
    },
    output_to_genfiles = True,
    implementation = _generate_file_impl,
)
"""Writes a string to a file at build time."""

def incorporate_rmw_implementation(
        kwargs,
        env_changes,
        rmw_implementation,
        rmw_implementation_explicit = False):
    """
    Args:
        rmw_implementation_explicit: if False (the default, and today's
            behavior, unchanged), the RMW implementation is selected at
            runtime via the RMW_IMPLEMENTATION environment variable, which
            rmw_implementation's dispatch library resolves with dlopen().

            If True, a build-time-renamed copy of the chosen backend
            (`<rmw_implementation>_as_rmw_implementation`) is linked in
            place of the dispatch library instead: librcl.so's (and
            librclcpp.so's, etc.) dependency on librmw_implementation.so is
            satisfied directly, with no dlopen() and no dependency on
            RMW_IMPLEMENTATION being set correctly at runtime. Only
            supported for RMW implementation packages that ship their own
            librmw_<name>.so (checked at BUILD_FILE generation time; see
            configure_package_rmw_implementation_override).
    """
    env_changes = dict(env_changes)
    if rmw_implementation_explicit:
        override = REPOSITORY_ROOT + ":%s_as_rmw_implementation" % rmw_implementation
        kwargs["data"] = kwargs.get("data", []) + [override]

        # REPOSITORY_ROOT is "@@<canonical repo name>//"; the ${LOAD_PATH}
        # path-prepend entries below are runfiles-relative paths rooted at
        # <canonical repo name>, matching the convention already used by
        # every other entry in RUNTIME_ENVIRONMENT (see distro.bzl).
        canonical_repo_name = REPOSITORY_ROOT[2:-2]
        override_runfiles_path = "{}/{}/librmw_implementation.so".format(
            canonical_repo_name, rmw_implementation)
        env_changes["${LOAD_PATH}"] = (
            ["path-prepend", override_runfiles_path] +
            env_changes["${LOAD_PATH}"][1:]
        )
    else:
        target = REPOSITORY_ROOT + ":%s_cc" % rmw_implementation
        kwargs["data"] = kwargs.get("data", []) + [target]
        env_changes.update({
            "RMW_IMPLEMENTATION": ["replace", rmw_implementation],
        })
    return kwargs, env_changes
