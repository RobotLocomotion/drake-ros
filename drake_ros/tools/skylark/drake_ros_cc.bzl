load(":extract_cc.bzl", "extract_direct_cc_hdrs_srcs_and_transitive_data")

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
        deps_to_relink):
    hdrs, srcs, data = extract_direct_cc_hdrs_srcs_and_transitive_data(
        name = original_name + "_extract_hdrs_srcs_data",
        deps = deps_to_relink,
        reuse_object_files = True,
    )
    return hdrs, srcs, data

def _workspace_name():
    repository = native.repository_name()
    if repository[0] != "@":
        fail("Unexpected repository")
    workspace = repository[1:]
    return workspace

def drake_ros_cc_relink_as_shared_library(
        name,
        deps_to_relink,
        deps = [],
        include_prefix = None,
        visibility = None,
        **kwargs):
    """
    Given a series of targets specified by `deps_to_relink`, this will extract
    the *direct* header files and source objects for those targets, as well as
    the transitive data, and repackage these components into a shared library.

    Arguments:
        name: Target name which must end with `shared_library`.
        deps_to_relink: Targets to relink. These labels must reside within the
            calling workspace.
        deps: Dependencies for the resultant shared library.
        include_prefix: For the header files.

    This assumes `deps_to_relink` are compiled in a way that allows their
    object code to be relinked into a shared library.
    """
    if not name.endswith("shared_library"):
        fail("Name must end with `shared_library`")

    solib = _make_solib_name(name)
    hdrs_target = "_" + name + "_hdrs"

    this_workspace = _workspace_name()
    bad_label_message = (
        "deps_to_relink must reside in the calling workspace '{}'\n" +
        "  First bad label: {}"
    )
    for dep in deps_to_relink:
        dep_label = Label(dep)
        if dep_label.workspace_name != this_workspace:
            fail(bad_label_message.format(this_workspace, dep_label))

    hdrs, srcs, data = _extract_shared_library_package_deps(
        original_name = name,
        deps_to_relink = deps_to_relink,
    )
    # TODO(eric.cousineau): Add linting against `deps` to ensure they are
    # a precise remapping of the dependencies of `deps_to_relink`.

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
