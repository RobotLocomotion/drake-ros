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

def incorporate_rmw_implementation(kwargs, env_changes, rmw_implementation):
    target = REPOSITORY_ROOT + ":%s_cc" % rmw_implementation
    kwargs["data"] = kwargs.get("data", []) + [target]
    env_changes = dict(env_changes)
    env_changes.update({
        "RMW_IMPLEMENTATION": ["replace", rmw_implementation],
    })
    return kwargs, env_changes
