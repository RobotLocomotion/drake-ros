load(":extract_cc.bzl", "extract_package_cc_hdrs_srcs_data")

def _make_solib_name(name):
    repo_name = native.repository_name()
    if repo_name == "@":
        repo_name = "drake_ros"
    inner = native.package_name().split("/")
    if inner == [""]:
        inner = []
    pieces = [repo_name] + inner + [name]
    solib = "lib{}.so.1".format("-".join(pieces))
    return solib

def _extract_shared_library_package_deps(
        original_name,
        package_deps):
    hdrs, srcs, data = extract_package_cc_hdrs_srcs_data(
        name = original_name + "_extract_hdrs_srcs_data",
        package_deps = package_deps,
        reuse_object_files = True,
    )
    return hdrs, srcs, data

def drake_ros_cc_shared_library(
        name,
        package_deps,
        deps = [],
        include_prefix = None,
        visibility = None,
        **kwargs):
    """Recompiles object code as a shared library, primarily for usage with
    Python bindings. `name` must end with `shared_library`.
    """
    if not name.endswith("shared_library"):
        fail("Name must end with `shared_library`")

    solib = _make_solib_name(name)
    hdrs_target = "_" + name + "_hdrs"

    hdrs, srcs, data = _extract_shared_library_package_deps(
        original_name = name,
        package_deps = package_deps,
    )
    # TODO(eric.cousineau): Add linting against `deps` to ensure they are
    # a precise remapping of the dependencies of `package_deps`.

    # Create header library with transitive deps.
    native.cc_library(
        name = hdrs_target,
        hdrs = hdrs,
        deps = deps,
        include_prefix = include_prefix,
        **kwargs
    )

    # Create main shared library.
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
