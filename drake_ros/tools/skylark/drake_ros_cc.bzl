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

def _workspace_name():
    repository = native.repository_name()
    if repository[0] != "@":
        fail("Unexpected repository")
    workspace = repository[1:]
    return workspace

def drake_ros_cc_recompile_as_shared_library(
        name,
        package_deps,
        deps = [],
        include_prefix = None,
        visibility = None,
        **kwargs):
    """
    Given a series of targets specified by `package_deps`, which should be part
    of this repository, this will extract the *direct* header files and source
    objects for those targets, as well as the transitive data, and repackage
    these components into a shared library.

    Arguments:
        name: Target name which must end with `shared_library`.
        package_deps: Targets that must be within this repository.
        deps: Dependencies for the resultant shared library.
        include_prefix: For the header files.

    This assumes `package_deps` are compiled in a way that allows their object
    code to be relinked into a shared library.
    """
    if not name.endswith("shared_library"):
        fail("Name must end with `shared_library`")

    solib = _make_solib_name(name)
    hdrs_target = "_" + name + "_hdrs"

    this_workspace = _workspace_name()
    bad_label_message = (
        "package_deps must only be in the current workspace '{}'\n" +
        "  first bad label: {}"
    )
    for dep in package_deps:
        dep_label = Label(dep)
        if dep_label.workspace_name != this_workspace:
            fail(bad_label_message.format(this_workspace, dep_label))

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
