# -*- mode: python -*-
# vi: set ft=python :

# Write out a repository that contains:
# - An empty BUILD file, to define a package.
# - An environ.bzl file with variable assignments for each ENV_NAMES item.
def _impl(repository_ctx):
    envvars = repository_ctx.attr.envvars
    bzl_content = []
    for key in envvars:
        value = repository_ctx.os.environ.get(key, "")
        bzl_content.append("{}='{}'\n".format(key, value))
    repository_ctx.file(
        "BUILD.bazel",
        content = "\n",
        executable = False,
    )
    repository_ctx.file(
        "environ.bzl",
        content = "".join(bzl_content),
        executable = False,
    )

_string_list = attr.string_list()

def environment_repository(name = None, envvars = []):
    """Provide specific environment variables for use in a WORKSPACE file.
    The `envvars` are the environment variables to provide.

    Example:
        environ_repository(name = "foo", envvars = ["BAR", "BAZ"])
        load("@foo//:environ.bzl", "BAR", "BAZ")
        print(BAR)
    """
    rule = repository_rule(
        implementation = _impl,
        attrs = {
            "envvars": _string_list,
        },
        local = True,
        environ = envvars,
    )
    rule(
        name = name,
        envvars = envvars,
    )
