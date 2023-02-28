def _make_solib_name(name):
    repo_name = native.repository_name()
    if repo_name == "@":
        repo_name = "drake_ros"
    pieces = [repo_name] + native.package_name().split("/") + [name]
    solib = "lib{}.so.1".format("-".join(pieces))
    return solib

def drake_ros_cc_shared_library(
        name,
        hdrs = [],
        srcs = [],
        deps = [],
        data = [],
        include_prefix = None,
        visibility = None,
        **kwargs):
    """Creates a shared library, primarily for usage with Python bindings.
    `name` must be `shared_library`.
    """
    if name != "shared_library":
        fail("Name must be `shared_library`")

    solib = _make_solib_name(name)
    hdrs_target = "_" + name + "_hdrs"

    # Create header library with transitive deps.
    native.cc_library(
        name = hdrs_target,
        hdrs = hdrs,
        deps = deps,
        include_prefix = include_prefix,
        **kwargs
    )

    # Create main shared library.
    # TODO(eric.cousineau): Hoist logic to reuse compiled targets from existing
    # package deps.
    native.cc_binary(
        name = solib,
        srcs = srcs,
        linkshared = 1,
        linkstatic = 1,
        data = data,
        deps = deps + [":" + hdrs_target],
        **kwargs
    )

    # Expose shared library and headers for transitive dependencies.
    native.cc_library(
        name = name,
        srcs = [solib],
        deps = [":" + hdrs_target],
        visibility = visibility,
        **kwargs
    )
