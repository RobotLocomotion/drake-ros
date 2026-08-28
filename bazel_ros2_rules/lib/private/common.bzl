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
        rmw_implementation_explicit: if True, links a build-time-renamed
            copy of the backend in place of rmw_implementation's dispatch
            library, so librcl.so resolves it directly with no dlopen() and
            no RMW_IMPLEMENTATION env var needed. Defaults to False (today's
            unchanged runtime dlopen()-based selection). Only works for
            backends that ship their own librmw_<name>.so; see
            configure_package_rmw_implementation_override.
    """
    env_changes = dict(env_changes)
    if rmw_implementation_explicit:
        # Also depend on the backend's own _cc library, for its transitive
        # shared library deps (libddsc.so.0, iceoryx, ...).
        backend = REPOSITORY_ROOT + ":%s_cc" % rmw_implementation
        override = REPOSITORY_ROOT + ":%s_as_rmw_implementation" % rmw_implementation
        kwargs["data"] = kwargs.get("data", []) + [backend, override]

        # ${LOAD_PATH} entries must be directories, not the .so file itself;
        # rooted at REPOSITORY_ROOT's canonical repo name, like every other
        # RUNTIME_ENVIRONMENT entry (see distro.bzl).
        canonical_repo_name = REPOSITORY_ROOT[2:-2]
        override_runfiles_dir = "{}/{}".format(
            canonical_repo_name, rmw_implementation)
        env_changes["${LOAD_PATH}"] = (
            ["path-prepend", override_runfiles_dir] +
            env_changes["${LOAD_PATH}"][1:]
        )
    else:
        target = REPOSITORY_ROOT + ":%s_cc" % rmw_implementation
        kwargs["data"] = kwargs.get("data", []) + [target]
        env_changes.update({
            "RMW_IMPLEMENTATION": ["replace", rmw_implementation],
        })
    return kwargs, env_changes
